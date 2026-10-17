// ============================================================================
//  Voice assistant over Wi-Fi
//  Mic -> Whisper STT -> Ollama LLM -> streamed Kokoro TTS -> speaker
//  ESP32-S3 (N16R8) | INMP441 mic + MAX98357A DAC | Arduino-ESP32 core 3.x
//
//  CONVERSATION FLOW
//    1. greet the user (printed + spoken)
//    2. listen: energy VAD with adaptive noise floor and 400 ms pre-roll
//    3. upload the utterance (WAV, held in PSRAM) to Whisper   -> transcript printed
//    4. send the transcript (+ a short history) to Ollama       -> reply printed
//    5. speak the reply, streamed byte-for-byte into the I2S DMA (no response buffer), then listen again
//    The mic is deaf during steps 3-5 so the ESP never transcribes its own voice.
//
//  Everything runs from loop() as non-blocking state machines. The only bounded-blocking calls are
//  client.connect() (<= CONNECT_TIMEOUT_MS) and the <= 4 KB slices of the Whisper upload.
//
//  Serial monitor (115200). Plain text = chat with the LLM without speaking. Commands:
//    /say <text>      speak <text> directly (no LLM)
//    /stop            abort whatever is in progress
//    /reset           forget the conversation history
//    /vol 0-100       speaker volume
//    /listen on|off   enable/disable voice activity detection
//    /rec [seconds]   record immediately for N s (default 5) and transcribe - bypasses the VAD
//    /gain 1-64       mic gain (default MIC_GAIN_DEFAULT)
//    /level           toggle a once-per-second mic level / VAD threshold print (for tuning)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s_std.h>

// ================================ USER CONFIG ===============================
static const char *WIFI_SSID = "<WIFI>";
static const char *WIFI_PASS = "<PASSWORD>";

// Sent as "Authorization: Bearer <API_KEY>" on BOTH the TTS and the Whisper requests.
// Set to "" if your servers don't use authentication (then no Authorization header is sent).
static const char *API_KEY = "<Whisper_API>";
// --- TTS (Kokoro) ---
#define TTS_HOST   "192.168.1.4"
#define TTS_PORT   8880
#define TTS_PATH   "/v1/audio/speech"
#define TTS_MODEL  "kokoro"
#define TTS_VOICE  "af_bella"
#define TTS_SPEED  1.0f

#define GREETING_TEXT  "Hello! I am ready. What would you like to talk about?"

// --- STT (Whisper, OpenAI-compatible) ---
#define STT_HOST      "192.168.1.4"
#define STT_PORT      3000
#define STT_PATH      "/api/v1/audio/transcriptions"
#define STT_MODEL     "whisper-1"     // use the model name your server exposes
                                      // (e.g. "Systran/faster-whisper-small" for faster-whisper-server/speaches)
#define STT_LANGUAGE  "en"            // "" = let Whisper auto-detect
#define STT_BOUNDARY  "----ESP32S3AudioBoundary7d3f1c"

// --- LLM (Ollama, native /api/chat) ---
#define LLM_HOST           "192.168.1.4"
#define LLM_PORT           11434
#define LLM_PATH           "/api/chat"
#define LLM_MODEL          "llama3.2:1b"
#define LLM_MAX_TOKENS     160        // caps the reply length - a spoken answer should be short
#define LLM_KEEP_ALIVE     "30m"      // keep the model loaded in the server between questions
#define LLM_USE_API_KEY    0          // Ollama has no auth by default; set 1 if a proxy in front of it wants the Bearer key
#define LLM_HISTORY_TURNS  3          // remembered question/answer pairs (minimum 1); /reset clears them
#define LLM_SYSTEM_PROMPT  "You are a friendly voice assistant speaking through a small speaker. " \
                           "Answer in one to three short, plain sentences. " \
                           "Never use markdown, lists, emojis or special symbols."
static const uint32_t LLM_RESPONSE_TIMEOUT_MS = 90000;   // the first request may have to load the model into memory

// --- MAX98357A (I2S0, TX).  Avoid GPIO 26-37 on N16R8 (flash/octal PSRAM), 19/20 (USB), 0/3/45/46 (strapping).
#define SPK_BCLK_PIN   15
#define SPK_LRC_PIN    16
#define SPK_DIN_PIN     7

// --- INMP441 (I2S1, RX).  Tie the mic's L/R pin to GND -> data arrives in the LEFT slot.
#define MIC_SCK_PIN         4
#define MIC_WS_PIN          5
#define MIC_SD_PIN          6
#define MIC_SAMPLE_RATE     16000     // what Whisper wants; the WAV header sent to the server says the same
#define MIC_GAIN_DEFAULT    8         // integer software gain applied after DC removal (INMP441 is quiet)

// --- Voice activity detection (levels are 16-bit units AFTER gain; tune with /level) ---
#define VAD_MIN_RMS          250      // never trigger below this, however quiet the room is
#define VAD_START_FRAMES     3        // consecutive loud 20 ms frames needed to start (60 ms)
#define VAD_PREROLL_FRAMES   20       // 400 ms of audio kept from before the trigger, so first syllables aren't clipped
#define VAD_END_SILENCE_MS   800      // this much quiet ends the utterance
#define VAD_MIN_SPEECH_MS    300      // shorter bursts (clicks, coughs) are discarded without bothering Whisper
#define VAD_TRAIL_KEEP_MS    200      // trailing silence kept in the upload
#define UTT_MAX_SECONDS      20       // utterance buffer size (PSRAM); falls back to a few seconds without PSRAM
#define MIC_MUTE_TAIL_MS     600      // stay deaf this long after the speaker stops (DMA tail + room echo)
static const float VAD_START_RATIO = 3.0f;    // speech starts at noise-floor x3  (~+10 dB)
static const float VAD_END_RATIO   = 2.0f;    // speech ends below noise-floor x2

#define VOLUME_PCT_DEFAULT  80        // speaker digital gain, 0..100

// --- TTS / I2S tunables ---
static const uint32_t DEFAULT_RATE         = 24000;   // Kokoro's native rate; reconfigured from the WAV header if different
static const uint32_t TX_DMA_DESC          = 16;      // 16 x 512 frames x 4 B = 32 KB internal RAM ...
static const uint32_t TX_DMA_FRAMES        = 512;     // ... = 8192 frames = ~341 ms @ 24 kHz
static const uint32_t PREROLL_MS           = 120;     // jitter headroom: leading silence queued before the first audio
static const uint32_t CONNECT_TIMEOUT_MS   = 3000;
static const uint32_t RESPONSE_TIMEOUT_MS  = 30000;   // TTS: waiting for HTTP headers
static const uint32_t STALL_TIMEOUT_MS     = 15000;   // TTS: waiting for more audio mid-stream
static const size_t   RX_CHUNK             = 1024;    // max bytes taken from the socket per read()
static const size_t   OUT_BUF_SIZE         = RX_CHUNK * 2 + 16;  // worst case: mono->stereo doubles the data
static const size_t   ERR_DUMP_MAX         = 300;     // echo at most this much of an HTTP error body

// --- STT tunables ---
static const size_t   STT_SLICE               = 4096;   // bytes per upload step (one slice per loop pass)
static const uint32_t STT_SEND_TIMEOUT_MS     = 10000;  // upload makes no progress
static const uint32_t STT_RESPONSE_TIMEOUT_MS = 60000;  // Whisper inference time (large models on CPU can be slow)

#define MIC_FRAME_SAMPLES  (MIC_SAMPLE_RATE / 50)       // 20 ms
static const uint32_t FRAME_MS = 20;

// ================================== STATE ===================================
enum TtsState : uint8_t { TTS_IDLE, TTS_HEADERS, TTS_BODY };
enum Framing  : uint8_t { FRAME_CLOSE, FRAME_LENGTH, FRAME_CHUNKED };
enum ChunkSt  : uint8_t { CHK_SIZE, CHK_DATA, CHK_CRLF };
enum WavSt    : uint8_t { WAV_RIFF, WAV_CHUNK_HDR, WAV_FMT, WAV_SKIP, WAV_DATA };
enum VadSt    : uint8_t { VAD_IDLE, VAD_CAPTURE };
enum SttSt    : uint8_t { STT_IDLE, STT_SEND_HEAD, STT_SEND_WAVHDR, STT_SEND_PCM, STT_SEND_TAIL, STT_WAIT };
enum LlmSt    : uint8_t { LLM_IDLE, LLM_WAIT };

static const char *const FRAMING_NAME[] = {"until-close", "content-length", "chunked"};

// ---- TTS
static WiFiClient client;
static TtsState   ttsState = TTS_IDLE;

static char     lineBuf[160];
static size_t   lineLen;
static bool     gotStatus, isChunked, hasLength;
static int      httpStatus;
static uint32_t contentLength, bodyRemain;
static Framing  framing;
static ChunkSt  chState;
static uint32_t chRemain;
static bool     chGotDigit, chInExt;
static bool     streamDone, streamError;
static uint32_t netWaitSince;
static size_t   errDumped;

static WavSt    wavSt;
static uint8_t  hdrBuf[16];
static size_t   hdrNeed, hdrHave;
static uint32_t skipRemain, fmtExtra;
static bool     fmtOk;
static uint16_t wavChannels;
static uint32_t wavRate;
static bool     haveOdd;
static uint8_t  oddByte;

static uint8_t  rxBuf[RX_CHUNK];
static uint8_t  outBuf[OUT_BUF_SIZE];
static size_t   outLen, outOff;
static uint32_t silenceOwed;                       // bytes of leading silence still to be queued
static uint32_t gainQ8 = (uint32_t)VOLUME_PCT_DEFAULT * 256 / 100;

static uint64_t pcmBytes;
static uint32_t t0;

// ---- I2S
static i2s_chan_handle_t txChan = nullptr;
static i2s_chan_handle_t rxChan = nullptr;
static uint32_t txRate = 0;

// ---- Mic / VAD
static int16_t  ring[VAD_PREROLL_FRAMES][MIC_FRAME_SAMPLES];   // pre-roll ring (12.8 KB)
static size_t   ringHead, ringCount;
static int16_t  frameBuf[MIC_FRAME_SAMPLES];
static size_t   frameFill;
static int16_t *uttBuf = nullptr;                              // utterance buffer (PSRAM)
static size_t   uttCap, uttLen, lastVoiceEnd;
static VadSt    vadState = VAD_IDLE;
static float    noiseFloor = -1.0f, lvStartThr, lvRmsMax;
static uint32_t startRun, silenceRun, voicedFrames;
static bool     forcedRec, micListen = true, micLevelPrint;
static uint32_t forceUntil, micMuteUntil;
static int32_t  dcEst;
static int32_t  micGain = MIC_GAIN_DEFAULT;

// ---- STT
static SttSt      sttState = STT_IDLE;
static WiFiClient sttClient;
static String     sttHead, sttTail;
static uint8_t    sttWavHdr[44];
static size_t     sttOff, sttPcmBytes;
static uint32_t   sttT0, sttLastProgress;
// shared by the Whisper and Ollama clients (they never run at the same time)
static char       httpResp[4096];                              // raw HTTP response (small JSON)
static size_t     httpRespLen;
static char       httpBody[4096];                              // de-chunked body (only used for chunked replies)

// ---- LLM (Ollama)
static LlmSt      llmState = LLM_IDLE;
static WiFiClient llmClient;
static String     llmPending;                                  // the question currently being answered
static uint32_t   llmT0;
static String     histUser[LLM_HISTORY_TURNS], histBot[LLM_HISTORY_TURNS];
static int        histCount;

static bool llmAsk(const String &user);   // defined in the LLM section; called whenever a transcript arrives

// ================================= HELPERS ==================================
static inline size_t   minz(size_t a, size_t b) { return a < b ? a : b; }
static inline uint16_t le16(const uint8_t *p)   { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static inline void put32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)(v >> 24);
}
static inline int hexval(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Appends "Authorization: Bearer <API_KEY>\r\n" to a raw HTTP request (we speak HTTP over WiFiClient directly,
// so this is the equivalent of http.addHeader("Authorization", authHeader)). No-op if API_KEY is empty.
static void addAuthHeader(String &req) {
  if (!API_KEY[0]) return;
  String authHeader = "Bearer ";
  authHeader += API_KEY;
  req += "Authorization: ";
  req += authHeader;
  req += "\r\n";
}

// ESP-IDF 5.4 renamed i2s_chan_config_t::auto_clear -> auto_clear_after_cb. Support both.
// (Auto-clear makes the DMA emit silence instead of repeating old data on an underrun.)
struct AutoClear {
  template <typename T>
  static auto set(T &c, int) -> decltype((void)(c.auto_clear_after_cb = true)) { c.auto_clear_after_cb = true; }
  template <typename T>
  static void set(T &c, long) { c.auto_clear = true; }
};

static String jsonEscape(const String &s) {
  String o;
  o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '"':  o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n";  break;
      case '\r': o += "\\r";  break;
      case '\t': o += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", (unsigned)c); o += b; }
        else o += c;
    }
  }
  return o;
}

// ============================== I2S (speaker + mic) =========================
// timeout 0 -> never blocks; returns however many bytes the DMA ring could take right now
static inline size_t i2sWrite(const uint8_t *d, size_t len) {
  size_t w = 0;
  i2s_channel_write(txChan, d, len, &w, 0);
  return w;
}

static bool setTxRate(uint32_t rate) {
  if (rate == txRate) return true;

  // Only reached if the server's rate differs from the current one:
  // let the tail of the previous audio leave the DMA ring first (bounded, ~0.35 s).
  static uint8_t zeros[512];
  size_t left = TX_DMA_DESC * TX_DMA_FRAMES * 4;
  while (left) {
    size_t w = 0;
    i2s_channel_write(txChan, zeros, minz(left, sizeof zeros), &w, 300);
    if (!w) break;
    left -= w;
  }

  i2s_channel_disable(txChan);
  i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
  esp_err_t e = i2s_channel_reconfig_std_clock(txChan, &clk);
  i2s_channel_enable(txChan);
  if (e != ESP_OK) { Serial.printf("[I2S] clock reconfig failed: %s\n", esp_err_to_name(e)); return false; }
  txRate = rate;
  return true;
}

static bool audioBegin() {
  esp_err_t e;

  // ---------- Speaker: I2S0, TX, 16-bit stereo ----------
  i2s_chan_config_t tcc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  tcc.dma_desc_num  = TX_DMA_DESC;
  tcc.dma_frame_num = TX_DMA_FRAMES;
  AutoClear::set(tcc, 0);
  e = i2s_new_channel(&tcc, &txChan, nullptr);
  if (e != ESP_OK) { Serial.printf("[I2S] TX channel: %s\n", esp_err_to_name(e)); return false; }

  i2s_std_clk_config_t  txClk  = I2S_STD_CLK_DEFAULT_CONFIG(DEFAULT_RATE);
  i2s_std_slot_config_t txSlot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  i2s_std_config_t txCfg = {};
  txCfg.clk_cfg       = txClk;
  txCfg.slot_cfg      = txSlot;
  txCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  txCfg.gpio_cfg.bclk = (gpio_num_t)SPK_BCLK_PIN;
  txCfg.gpio_cfg.ws   = (gpio_num_t)SPK_LRC_PIN;
  txCfg.gpio_cfg.dout = (gpio_num_t)SPK_DIN_PIN;
  txCfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  e = i2s_channel_init_std_mode(txChan, &txCfg);
  if (e != ESP_OK) { Serial.printf("[I2S] TX init: %s\n", esp_err_to_name(e)); return false; }
  e = i2s_channel_enable(txChan);
  if (e != ESP_OK) { Serial.printf("[I2S] TX enable: %s\n", esp_err_to_name(e)); return false; }
  txRate = DEFAULT_RATE;

  // ---------- Mic: I2S1, RX, 32-bit slot (INMP441 = 24-bit left-justified), mono/left ----------
  i2s_chan_config_t rcc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  rcc.dma_desc_num  = 8;                           // 8 x 256 frames x 4 B = 8 KB = 128 ms of mic audio
  rcc.dma_frame_num = 256;
  e = i2s_new_channel(&rcc, nullptr, &rxChan);
  if (e != ESP_OK) { Serial.printf("[I2S] RX channel: %s\n", esp_err_to_name(e)); return false; }

  i2s_std_clk_config_t  rxClk  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE);
  i2s_std_slot_config_t rxSlot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  rxSlot.slot_mask = I2S_STD_SLOT_LEFT;
  i2s_std_config_t rxCfg = {};
  rxCfg.clk_cfg       = rxClk;
  rxCfg.slot_cfg      = rxSlot;
  rxCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  rxCfg.gpio_cfg.bclk = (gpio_num_t)MIC_SCK_PIN;
  rxCfg.gpio_cfg.ws   = (gpio_num_t)MIC_WS_PIN;
  rxCfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  rxCfg.gpio_cfg.din  = (gpio_num_t)MIC_SD_PIN;
  e = i2s_channel_init_std_mode(rxChan, &rxCfg);
  if (e != ESP_OK) { Serial.printf("[I2S] RX init: %s\n", esp_err_to_name(e)); return false; }
  e = i2s_channel_enable(rxChan);
  if (e != ESP_OK) { Serial.printf("[I2S] RX enable: %s\n", esp_err_to_name(e)); return false; }
  return true;
}

// ============================ OUTPUT STAGE (to DMA) =========================
// Queue the owed leading silence. Returns true when fully queued.
static bool paySilence() {
  static const uint8_t zeros[1024] = {0};
  while (silenceOwed) {
    size_t n = minz(silenceOwed, sizeof zeros);
    size_t w = i2sWrite(zeros, n);
    silenceOwed -= (uint32_t)w;
    if (w < n) return false;
  }
  return true;
}

// Push the converted PCM slice into the DMA ring. Returns true when it was accepted completely.
static bool flushOut() {
  if (outOff < outLen) {
    outOff += i2sWrite(outBuf + outOff, outLen - outOff);
    if (outOff < outLen) return false;
  }
  outOff = outLen = 0;
  return true;
}

// One 16-bit sample in -> gain -> 1 (stereo source) or 2 (mono source) samples out
static inline void emitSample(int16_t s) {
  if (outLen + 4 > OUT_BUF_SIZE) return;           // cannot happen: OUT_BUF_SIZE covers the worst case
  int32_t v = ((int32_t)s * (int32_t)gainQ8) >> 8;
  if (v > 32767) v = 32767; else if (v < -32768) v = -32768;
  uint16_t u = (uint16_t)(int16_t)v;
  uint8_t lo = u & 0xFF, hi = u >> 8;
  outBuf[outLen++] = lo; outBuf[outLen++] = hi;
  if (wavChannels == 1) { outBuf[outLen++] = lo; outBuf[outLen++] = hi; }   // MAX98357A: duplicate to L+R
}

static void pcmFeed(const uint8_t *p, size_t n) {
  pcmBytes += n;
  size_t i = 0;
  if (haveOdd && n) { emitSample((int16_t)(oddByte | (p[0] << 8))); haveOdd = false; i = 1; }
  for (; i + 1 < n; i += 2) emitSample((int16_t)(p[i] | (p[i + 1] << 8)));
  if (i < n) { oddByte = p[i]; haveOdd = true; }
}

// ============================== STREAM CONTROL ==============================
static void resetStream() {
  lineLen = 0;
  gotStatus = isChunked = hasLength = false;
  httpStatus = 0;
  contentLength = bodyRemain = 0;
  framing = FRAME_CLOSE;
  chState = CHK_SIZE; chRemain = 0; chGotDigit = chInExt = false;
  streamDone = streamError = false;
  netWaitSince = 0; errDumped = 0;
  wavSt = WAV_RIFF; hdrNeed = 12; hdrHave = 0; skipRemain = 0; fmtExtra = 0; fmtOk = false;
  wavChannels = 0; wavRate = 0; haveOdd = false;
  pcmBytes = 0;
  outLen = outOff = 0; silenceOwed = 0;
}

static void fail(const char *why) {
  if (!streamError) Serial.printf("[TTS] error: %s\n", why);
  streamError = true;
}

// ================================ WAV PARSER ================================
static void onDataStart() {
  Serial.printf("[TTS] WAV %lu Hz, %u ch, 16-bit  (first audio after %lu ms)\n",
                (unsigned long)wavRate, (unsigned)wavChannels, (unsigned long)(millis() - t0));
  if (!setTxRate(wavRate)) { fail("I2S clock reconfig failed"); return; }
  silenceOwed = (uint32_t)(((uint64_t)wavRate * PREROLL_MS) / 1000) * 4;   // stereo 16-bit frames
}

// Walks RIFF -> chunks until the "data" chunk starts. Works for any header size (LIST chunks etc.),
// not just 44 bytes. The data-size field is ignored: streaming servers write 0 / 0xFFFFFFFF there.
// Returns the number of bytes consumed.
static size_t wavFeed(const uint8_t *p, size_t n) {
  size_t used = 0;
  while (used < n && wavSt != WAV_DATA && !streamError) {
    if (wavSt == WAV_SKIP) {
      size_t t = minz(n - used, skipRemain);
      used += t; skipRemain -= (uint32_t)t;
      if (!skipRemain) { wavSt = WAV_CHUNK_HDR; hdrNeed = 8; hdrHave = 0; }
      continue;
    }

    size_t t = minz(n - used, hdrNeed - hdrHave);       // accumulate one fixed-size header piece
    memcpy(hdrBuf + hdrHave, p + used, t);
    hdrHave += t; used += t;
    if (hdrHave < hdrNeed) break;                        // need more bytes from the network
    hdrHave = 0;

    switch (wavSt) {
      case WAV_RIFF:
        if (memcmp(hdrBuf, "RIFF", 4) || memcmp(hdrBuf + 8, "WAVE", 4)) { fail("response is not RIFF/WAVE"); break; }
        wavSt = WAV_CHUNK_HDR; hdrNeed = 8;
        break;

      case WAV_CHUNK_HDR: {
        uint32_t sz = le32(hdrBuf + 4);
        if (!memcmp(hdrBuf, "fmt ", 4)) {
          if (sz < 16) { fail("bad fmt chunk"); break; }
          fmtExtra = (sz - 16) + (sz & 1);
          wavSt = WAV_FMT; hdrNeed = 16;
        } else if (!memcmp(hdrBuf, "data", 4)) {
          if (!fmtOk) { fail("data chunk before fmt chunk"); break; }
          wavSt = WAV_DATA;
          onDataStart();
        } else {                                         // LIST, fact, ... -> skip (chunks are word-aligned)
          skipRemain = sz + (sz & 1);
          wavSt = WAV_SKIP;
        }
        break;
      }

      case WAV_FMT: {
        uint16_t tag = le16(hdrBuf), ch = le16(hdrBuf + 2), bits = le16(hdrBuf + 14);
        uint32_t rate = le32(hdrBuf + 4);
        if ((tag != 1 && tag != 0xFFFE) || bits != 16 || (ch != 1 && ch != 2) || rate < 8000 || rate > 96000) {
          Serial.printf("[TTS] fmt tag=%u ch=%u rate=%lu bits=%u\n", tag, ch, (unsigned long)rate, bits);
          fail("unsupported WAV (need 16-bit PCM, 1-2 channels)");
          break;
        }
        wavChannels = ch; wavRate = rate; fmtOk = true;
        skipRemain = fmtExtra;
        wavSt = WAV_SKIP;
        break;
      }

      default: break;
    }
  }
  return used;
}

// ============================== HTTP RESPONSE ===============================
// Everything that comes out of the HTTP framing layer (i.e. real body bytes) lands here.
static void feedPayload(const uint8_t *p, size_t n) {
  if (httpStatus != 200) {                               // error body: echo a little for diagnostics
    size_t room = ERR_DUMP_MAX - minz(errDumped, ERR_DUMP_MAX);
    size_t k = minz(n, room);
    if (k) { Serial.write(p, k); errDumped += k; }
    return;
  }
  if (wavSt != WAV_DATA) {
    size_t u = wavFeed(p, n);
    p += u; n -= u;
  }
  if (!streamError && wavSt == WAV_DATA && n) pcmFeed(p, n);
}

// HTTP framing: strips chunked-transfer metadata (otherwise "1F4\r\n" lines would become clicks in the audio)
static void deliverBody(const uint8_t *p, size_t n) {
  switch (framing) {
    case FRAME_CLOSE:
      feedPayload(p, n);
      break;

    case FRAME_LENGTH: {
      size_t take = minz(n, bodyRemain);
      feedPayload(p, take);
      bodyRemain -= (uint32_t)take;
      if (bodyRemain == 0) streamDone = true;
      break;
    }

    case FRAME_CHUNKED:
      while (n && !streamDone && !streamError) {
        if (chState == CHK_SIZE) {
          uint8_t c = *p++; n--;
          if (c == '\n') {
            if (chGotDigit) {
              chGotDigit = false; chInExt = false;
              if (chRemain == 0) streamDone = true;      // last chunk (trailers ignored)
              else chState = CHK_DATA;
            }
          } else if (!chInExt) {
            int v = hexval(c);
            if (v >= 0) { chRemain = (chRemain << 4) | (uint32_t)v; chGotDigit = true; }
            else if (c == ';') chInExt = true;           // chunk extension
          }
        } else if (chState == CHK_DATA) {
          size_t take = minz(n, chRemain);
          feedPayload(p, take);
          p += take; n -= take; chRemain -= (uint32_t)take;
          if (chRemain == 0) chState = CHK_CRLF;
        } else {                                         // CHK_CRLF: eat the CRLF after the chunk data
          uint8_t c = *p++; n--;
          if (c == '\n') chState = CHK_SIZE;
        }
      }
      break;
  }
}

static void onHeaderLine() {
  lineBuf[lineLen] = 0;

  if (!gotStatus) {                                      // "HTTP/1.1 200 OK"
    const char *sp = strchr(lineBuf, ' ');
    int code = sp ? atoi(sp + 1) : 0;
    if (strncmp(lineBuf, "HTTP/", 5) != 0 || code < 100) { fail("bad HTTP status line"); return; }
    httpStatus = code;
    gotStatus = true;
    return;
  }

  if (lineLen == 0) {                                    // blank line -> headers finished
    framing = isChunked ? FRAME_CHUNKED : (hasLength ? FRAME_LENGTH : FRAME_CLOSE);
    if (framing == FRAME_LENGTH) { bodyRemain = contentLength; if (!bodyRemain) streamDone = true; }
    ttsState = TTS_BODY;
    netWaitSince = 0;
    Serial.printf("[TTS] HTTP %d, body framing: %s\n", httpStatus, FRAMING_NAME[framing]);
    if (httpStatus != 200) Serial.print("[TTS] server says: ");
    return;
  }

  for (size_t i = 0; i < lineLen; i++) lineBuf[i] = (char)tolower((unsigned char)lineBuf[i]);
  if (!strncmp(lineBuf, "transfer-encoding:", 18) && strstr(lineBuf + 18, "chunked")) isChunked = true;
  else if (!strncmp(lineBuf, "content-length:", 15)) { contentLength = strtoul(lineBuf + 15, nullptr, 10); hasLength = true; }
}

// Consume one slice of raw socket bytes (headers first, then body).
static void consume(const uint8_t *p, size_t n) {
  while (n && !streamError && !streamDone) {
    if (ttsState == TTS_HEADERS) {
      uint8_t c = *p++; n--;
      if (c == '\n') { onHeaderLine(); lineLen = 0; }
      else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = (char)c;
    } else {
      deliverBody(p, n);
      return;
    }
  }
}

// ============================== PUBLIC TTS API ==============================
static bool ttsSpeak(const String &text) {
  if (ttsState != TTS_IDLE) return false;
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[TTS] WiFi not connected"); return false; }

  resetStream();
  client.stop();
  if (!client.connect(TTS_HOST, TTS_PORT, CONNECT_TIMEOUT_MS)) { Serial.println("[TTS] connect failed"); return false; }
  client.setNoDelay(true);

  String body = String("{\"model\":\"" TTS_MODEL "\",\"input\":\"") + jsonEscape(text) +
                "\",\"voice\":\"" TTS_VOICE "\",\"response_format\":\"wav\",\"speed\":" +
                String(TTS_SPEED, 1) + "}";

  String req;
  req.reserve(body.length() + 192);
  req  = "POST " TTS_PATH " HTTP/1.1\r\n";
  req += "Host: " TTS_HOST ":";  req += String(TTS_PORT);  req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "Accept: */*\r\n";
  addAuthHeader(req);
  req += "Connection: close\r\n";
  req += "Content-Length: ";     req += String(body.length());  req += "\r\n\r\n";
  req += body;

  if (client.write((const uint8_t *)req.c_str(), req.length()) != req.length()) {
    Serial.println("[TTS] request send failed");
    client.stop();
    return false;
  }

  t0 = millis();
  netWaitSince = t0;
  ttsState = TTS_HEADERS;
  Serial.printf("[TTS] POST sent (%u bytes)\n", (unsigned)req.length());
  return true;
}

static void finishStream() {
  client.stop();
  ttsState = TTS_IDLE;
  micMuteUntil = millis() + MIC_MUTE_TAIL_MS;            // DMA tail is still playing: keep the mic deaf a bit longer
  if (httpStatus != 200) {
    Serial.println();
  } else if (!streamError && wavRate && wavChannels) {
    float secs = (float)pcmBytes / ((float)wavRate * wavChannels * 2.0f);
    Serial.printf("[TTS] done: %.2f s of audio (%lu bytes PCM) in %lu ms\n",
                  secs, (unsigned long)pcmBytes, (unsigned long)(millis() - t0));
  } else {
    Serial.println("[TTS] stream ended without audio");
  }
}

static void ttsCancel() {
  if (ttsState != TTS_IDLE) { client.stop(); ttsState = TTS_IDLE; }
  outLen = outOff = 0;
  silenceOwed = 0;
  i2s_channel_disable(txChan);                           // drops whatever is still queued in the DMA ring
  i2s_channel_enable(txChan);
  micMuteUntil = millis() + 200;
  Serial.println("[TTS] stopped");
}

// Call from loop() as often as possible. Never blocks. Returns true if it moved data.
static bool ttsService() {
  if (ttsState == TTS_IDLE) return false;
  bool worked = false;

  for (int pass = 0; pass < 4; pass++) {
    // 1+2: get everything we already converted into the DMA ring first (back-pressure point)
    if (!paySilence()) { netWaitSince = 0; break; }      // ring full -> don't read the socket; TCP throttles the server
    if (!flushOut())   { netWaitSince = 0; break; }

    // 3: finished (or failed) and everything is queued
    if (streamDone || streamError) { finishStream(); return true; }

    // 4: pull whatever the Wi-Fi stack has for us right now
    int avail = client.available();
    if (avail <= 0) {
      if (!client.connected()) {                         // server closed the socket
        if (ttsState == TTS_HEADERS) {
          fail("connection closed before response");
        } else {
          if (framing != FRAME_CLOSE) Serial.println("[TTS] warning: connection closed before end of body");
          streamDone = true;
        }
        continue;
      }
      uint32_t now = millis();
      if (!netWaitSince) netWaitSince = now;
      else if (now - netWaitSince > (ttsState == TTS_HEADERS ? RESPONSE_TIMEOUT_MS : STALL_TIMEOUT_MS)) fail("network timeout");
      break;
    }
    netWaitSince = 0;

    int n = client.read(rxBuf, minz((size_t)avail, RX_CHUNK));
    if (n <= 0) break;
    worked = true;
    consume(rxBuf, (size_t)n);                           // -> HTTP framing -> WAV parser -> outBuf
  }
  return worked;
}

// ============================ STT: WHISPER CLIENT ===========================
// Every finished transcript lands here and is handed to the LLM.
static void onTranscript(const String &text, uint32_t ms) {
  if (text.length() < 2) { Serial.printf("[STT] (nothing recognised)  (%lu ms)\n", (unsigned long)ms); return; }
  Serial.printf("[STT] you said: %s  (%lu ms)\n", text.c_str(), (unsigned long)ms);
  llmAsk(text);
}

static void appendUtf8(String &s, uint32_t cp) {
  if (cp < 0x80) s += (char)cp;
  else if (cp < 0x800) { s += (char)(0xC0 | (cp >> 6)); s += (char)(0x80 | (cp & 0x3F)); }
  else if (cp < 0x10000) { s += (char)(0xE0 | (cp >> 12)); s += (char)(0x80 | ((cp >> 6) & 0x3F)); s += (char)(0x80 | (cp & 0x3F)); }
  else { s += (char)(0xF0 | (cp >> 18)); s += (char)(0x80 | ((cp >> 12) & 0x3F)); s += (char)(0x80 | ((cp >> 6) & 0x3F)); s += (char)(0x80 | (cp & 0x3F)); }
}

static bool hex4(const char *p, uint32_t *v) {
  uint32_t r = 0;
  for (int i = 0; i < 4; i++) {
    int h = hexval((uint8_t)p[i]);                       // a NUL terminator is not hex -> stops safely
    if (h < 0) return false;
    r = (r << 4) | (uint32_t)h;
  }
  *v = r;
  return true;
}

// Pulls the string value of "<key>" out of a JSON object (handles \n, \", \uXXXX incl. surrogate pairs).
static bool jsonGetString(const char *s, const char *key, String &out) {
  char pat[32];
  snprintf(pat, sizeof pat, "\"%s\"", key);
  const char *k = strstr(s, pat);
  if (!k) return false;
  k += strlen(pat);
  while (*k == ' ' || *k == ':' || *k == '\t' || *k == '\r' || *k == '\n') k++;
  if (*k != '"') return false;
  k++;
  out = "";
  while (*k && *k != '"') {
    if (*k != '\\') { out += *k++; continue; }
    k++;                                                 // escape sequence
    switch (*k) {
      case 'n': case 'r': case 't': out += ' '; k++; break;
      case 'b': case 'f': k++; break;
      case 'u': {
        uint32_t cp;
        if (!hex4(k + 1, &cp)) return true;              // malformed escape: keep what we have
        k += 5;
        if (cp >= 0xD800 && cp < 0xDC00 && k[0] == '\\' && k[1] == 'u') {
          uint32_t lo;
          if (hex4(k + 2, &lo) && lo >= 0xDC00 && lo < 0xE000) { cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00); k += 6; }
        }
        appendUtf8(out, cp);
        break;
      }
      case 0: return true;
      default: out += *k++; break;                       // \"  \\  \/
    }
  }
  return true;
}

// Non-destructive chunked-transfer decode of a (possibly still incomplete) body.
static size_t dechunk(const char *in, size_t n, char *out, size_t cap, bool *complete) {
  size_t r = 0, w = 0;
  *complete = false;
  while (r < n) {
    size_t sz = 0;
    bool ext = false, any = false;
    while (r < n && in[r] != '\n') {
      uint8_t c = (uint8_t)in[r++];
      if (ext) continue;
      int v = hexval(c);
      if (v >= 0) { sz = (sz << 4) | (size_t)v; any = true; }
      else if (c == ';') ext = true;
    }
    if (r >= n) break;                                   // size line not complete yet
    r++;                                                 // the '\n'
    if (!any) continue;                                  // stray blank line
    if (sz == 0) { *complete = true; break; }
    size_t take = minz(sz, n - r);
    size_t cp = minz(take, cap - w);
    memcpy(out + w, in + r, cp);
    w += cp; r += take;
    if (take < sz) break;                                // chunk data not complete yet
    if (r < n && in[r] == '\r') r++;
    if (r < n && in[r] == '\n') r++;
  }
  return w;
}

static void sttFinish() {
  sttClient.stop();
  sttState = STT_IDLE;
  micMuteUntil = millis() + 150;                         // brief guard before listening again
}

// Parses the buffered HTTP response (shared by the Whisper and Ollama clients).
// 0 = need more data, 1 = complete (status/body valid), 2 = connection ended without a usable response
static int httpParse(bool final, int *status, const char **body) {
  httpResp[httpRespLen] = 0;
  char *hdrEnd = strstr(httpResp, "\r\n\r\n");
  if (!hdrEnd) return final ? 2 : 0;
  char *rawBody = hdrEnd + 4;
  size_t rawLen = (size_t)(httpResp + httpRespLen - rawBody);

  *hdrEnd = 0;                                           // headers -> lowercase C-string for easy matching
  for (char *c = httpResp; *c; c++) *c = (char)tolower((unsigned char)*c);
  const char *sp = strchr(httpResp, ' ');
  *status = sp ? atoi(sp + 1) : 0;
  bool chunked = strstr(httpResp, "transfer-encoding:") && strstr(httpResp, "chunked");
  const char *cl = strstr(httpResp, "content-length:");
  long clen = cl ? atol(cl + 15) : -1;
  *hdrEnd = '\r';

  size_t blen;
  bool complete = false;
  if (chunked) {
    blen = dechunk(rawBody, rawLen, httpBody, sizeof httpBody - 1, &complete);
    httpBody[blen] = 0;
    *body = httpBody;
  } else {
    blen = rawLen;
    if (clen >= 0) { complete = rawLen >= (size_t)clen; if (complete) blen = (size_t)clen; }
    rawBody[blen] = 0;
    *body = rawBody;
  }
  return (complete || final) ? 1 : 0;
}

// Non-blocking: pull whatever the server has sent into httpResp, then try to parse it.
// Returns 0 = keep waiting, 1 = finished (status/body valid), 2 = connection closed without a complete response.
static int httpPoll(WiFiClient &c, int *status, const char **body, bool *got) {
  int avail = c.available();
  bool full = httpRespLen >= sizeof(httpResp) - 1;
  if (avail > 0) {
    *got = true;
    if (!full) {
      int n = c.read((uint8_t *)httpResp + httpRespLen, minz((size_t)avail, sizeof(httpResp) - 1 - httpRespLen));
      if (n > 0) httpRespLen += (size_t)n;
    } else {
      uint8_t tmp[64];
      c.read(tmp, minz((size_t)avail, sizeof tmp));      // reply bigger than our buffer: drop the rest
    }
  }
  bool closed = (avail <= 0) && !c.connected();
  if (avail <= 0 && !closed && !full) return 0;
  return httpParse(closed || full, status, body);
}

static void sttHandleResponse(int status, const char *body) {
  if (status != 200) { Serial.printf("[STT] HTTP %d: %.200s\n", status, body); return; }
  String text;
  if (!jsonGetString(body, "text", text)) { Serial.printf("[STT] unexpected reply: %.200s\n", body); return; }
  text.trim();
  onTranscript(text, millis() - sttT0);                  // -> LLM
}

static void addField(String &s, const char *name, const char *value) {
  s += "--" STT_BOUNDARY "\r\nContent-Disposition: form-data; name=\"";
  s += name;
  s += "\"\r\n\r\n";
  s += value;
  s += "\r\n";
}

// Start uploading uttBuf[0..samples) as a WAV file in a multipart/form-data POST. Returns false if it could not start.
static bool sttSubmit(size_t samples) {
  if (sttState != STT_IDLE) return false;
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[STT] WiFi not connected"); return false; }

  sttClient.stop();
  if (!sttClient.connect(STT_HOST, STT_PORT, CONNECT_TIMEOUT_MS)) { Serial.println("[STT] connect failed"); return false; }
  sttClient.setNoDelay(true);

  sttPcmBytes = samples * 2;

  // 44-byte WAV header (16 kHz, mono, 16-bit)
  memcpy(sttWavHdr, "RIFF", 4);  put32(sttWavHdr + 4, 36 + (uint32_t)sttPcmBytes);
  memcpy(sttWavHdr + 8, "WAVE", 4);
  memcpy(sttWavHdr + 12, "fmt ", 4);  put32(sttWavHdr + 16, 16);  put16(sttWavHdr + 20, 1);  put16(sttWavHdr + 22, 1);
  put32(sttWavHdr + 24, MIC_SAMPLE_RATE);  put32(sttWavHdr + 28, MIC_SAMPLE_RATE * 2);
  put16(sttWavHdr + 32, 2);  put16(sttWavHdr + 34, 16);
  memcpy(sttWavHdr + 36, "data", 4);  put32(sttWavHdr + 40, (uint32_t)sttPcmBytes);

  // multipart preamble (form fields + the start of the file part) and the closing boundary
  String pre;
  pre.reserve(512);
  addField(pre, "model", STT_MODEL);
  addField(pre, "response_format", "json");
  if (STT_LANGUAGE[0]) addField(pre, "language", STT_LANGUAGE);
  pre += "--" STT_BOUNDARY "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"speech.wav\"\r\n"
         "Content-Type: audio/wav\r\n\r\n";
  sttTail = "\r\n--" STT_BOUNDARY "--\r\n";

  size_t contentLen = pre.length() + sizeof sttWavHdr + sttPcmBytes + sttTail.length();
  sttHead  = "POST " STT_PATH " HTTP/1.1\r\n";
  sttHead += "Host: " STT_HOST ":";  sttHead += String(STT_PORT);  sttHead += "\r\n";
  sttHead += "Content-Type: multipart/form-data; boundary=" STT_BOUNDARY "\r\n";
  sttHead += "Accept: */*\r\n";
  addAuthHeader(sttHead);
  sttHead += "Connection: close\r\n";
  sttHead += "Content-Length: ";  sttHead += String((unsigned long)contentLen);  sttHead += "\r\n\r\n";
  sttHead += pre;

  sttOff = 0;
  httpRespLen = 0;
  sttT0 = sttLastProgress = millis();
  sttState = STT_SEND_HEAD;
  Serial.printf("[STT] uploading %u bytes to Whisper...\n", (unsigned)contentLen);
  return true;
}

// Sends one slice of buf[0..total). Returns true once everything has been handed to the TCP stack.
static bool sttSendSlice(const uint8_t *buf, size_t total) {
  if (sttOff < total) {
    size_t w = sttClient.write(buf + sttOff, minz(total - sttOff, STT_SLICE));
    if (w) { sttOff += w; sttLastProgress = millis(); }
  }
  return sttOff >= total;
}

// Call from loop(). Uploads one slice per call, then polls for the response. Returns true if it did work.
static bool sttService() {
  if (sttState == STT_IDLE) return false;
  bool worked = false;

  // server hung up while we were still uploading -> go and read whatever it said
  if (sttState < STT_WAIT && !sttClient.connected()) sttState = STT_WAIT;

  switch (sttState) {
    case STT_SEND_HEAD:
      worked = true;
      if (sttSendSlice((const uint8_t *)sttHead.c_str(), sttHead.length())) { sttState = STT_SEND_WAVHDR; sttOff = 0; }
      break;

    case STT_SEND_WAVHDR:
      worked = true;
      if (sttSendSlice(sttWavHdr, sizeof sttWavHdr)) { sttState = STT_SEND_PCM; sttOff = 0; }
      break;

    case STT_SEND_PCM:                                   // straight from the utterance buffer, no extra copy
      worked = true;
      if (sttSendSlice((const uint8_t *)uttBuf, sttPcmBytes)) { sttState = STT_SEND_TAIL; sttOff = 0; }
      break;

    case STT_SEND_TAIL:
      worked = true;
      if (sttSendSlice((const uint8_t *)sttTail.c_str(), sttTail.length())) {
        sttState = STT_WAIT;
        Serial.printf("[STT] upload done in %lu ms, waiting for transcript...\n", (unsigned long)(millis() - sttT0));
      }
      break;

    case STT_WAIT: {
      int status = 0;
      const char *body = nullptr;
      bool got = false;
      int r = httpPoll(sttClient, &status, &body, &got);
      worked = got;
      if (r == 1) { sttFinish(); sttHandleResponse(status, body); return true; }
      if (r == 2) { Serial.println("[STT] error: no complete HTTP response from server"); sttFinish(); return true; }
      if ((int32_t)(millis() - sttT0) > (int32_t)STT_RESPONSE_TIMEOUT_MS) { Serial.println("[STT] timeout waiting for transcript"); sttFinish(); return true; }
      break;
    }

    default: break;
  }

  if (sttState != STT_WAIT && sttState != STT_IDLE && (int32_t)(millis() - sttLastProgress) > (int32_t)STT_SEND_TIMEOUT_MS) {
    Serial.println("[STT] upload stalled - aborting");
    sttFinish();
    return true;
  }
  return worked;
}

// ============================== LLM: OLLAMA CLIENT ==========================
static void histPush(const String &u, const String &b) {
  if (histCount == LLM_HISTORY_TURNS) {                  // full: drop the oldest pair
    for (int i = 1; i < LLM_HISTORY_TURNS; i++) { histUser[i - 1] = histUser[i]; histBot[i - 1] = histBot[i]; }
    histCount--;
  }
  histUser[histCount] = u;
  histBot[histCount]  = b;
  histCount++;
}

// Strip what a TTS engine stumbles over: markdown symbols, emoji (4-byte UTF-8), runs of whitespace.
static String speechClean(const String &in) {
  String o;
  o.reserve(in.length());
  bool lastSpace = true;
  for (size_t i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    if (c >= 0xF0) { i += 3; continue; }
    if (c == '*' || c == '#' || c == '`') continue;
    if (c <= ' ') { if (!lastSpace) { o += ' '; lastSpace = true; } continue; }
    o += (char)c;
    lastSpace = false;
  }
  o.trim();
  return o;
}

static void llmApologise() { ttsSpeak("Sorry, I could not get an answer."); }

static void llmFail(const char *why) {
  Serial.printf("[LLM] error: %s\n", why);
  llmApologise();
}

// Sends `user` (plus the short history) to Ollama. Non-blocking after the connect; llmService() collects the reply.
static bool llmAsk(const String &user) {
  if (llmState != LLM_IDLE) return false;
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[LLM] WiFi not connected"); return false; }

  llmClient.stop();
  if (!llmClient.connect(LLM_HOST, LLM_PORT, CONNECT_TIMEOUT_MS)) { llmFail("could not connect to Ollama"); return false; }
  llmClient.setNoDelay(true);

  String body;
  body.reserve(1024);
  body  = "{\"model\":\"" LLM_MODEL "\",\"stream\":false,\"keep_alive\":\"" LLM_KEEP_ALIVE "\",";
  body += "\"options\":{\"num_predict\":";  body += String(LLM_MAX_TOKENS);  body += ",\"temperature\":0.7},";
  body += "\"messages\":[{\"role\":\"system\",\"content\":\"" LLM_SYSTEM_PROMPT "\"}";
  for (int i = 0; i < histCount; i++) {
    body += ",{\"role\":\"user\",\"content\":\"";       body += jsonEscape(histUser[i]);  body += "\"}";
    body += ",{\"role\":\"assistant\",\"content\":\"";  body += jsonEscape(histBot[i]);   body += "\"}";
  }
  body += ",{\"role\":\"user\",\"content\":\"";  body += jsonEscape(user);  body += "\"}]}";

  String req;
  req.reserve(body.length() + 192);
  req  = "POST " LLM_PATH " HTTP/1.1\r\n";
  req += "Host: " LLM_HOST ":";  req += String(LLM_PORT);  req += "\r\n";
  req += "Content-Type: application/json\r\n";
  req += "Accept: application/json\r\n";
#if LLM_USE_API_KEY
  addAuthHeader(req);
#endif
  req += "Connection: close\r\n";
  req += "Content-Length: ";  req += String(body.length());  req += "\r\n\r\n";
  req += body;

  if (llmClient.write((const uint8_t *)req.c_str(), req.length()) != req.length()) {
    llmClient.stop();
    llmFail("request send failed");
    return false;
  }

  llmPending = user;
  httpRespLen = 0;
  llmT0 = millis();
  llmState = LLM_WAIT;
  Serial.println("[LLM] thinking...");
  return true;
}

static void llmHandleResponse(int status, const char *body) {
  uint32_t ms = millis() - llmT0;
  if (status != 200) {                                   // e.g. 404 {"error":"model 'llama3.2:1b' not found"}
    Serial.printf("[LLM] HTTP %d: %.200s\n", status, body);
    llmApologise();
    return;
  }
  String reply;
  if (!jsonGetString(body, "content", reply)) {          // Ollama: {"message":{"role":"assistant","content":"..."},...}
    Serial.printf("[LLM] unexpected reply: %.200s\n", body);
    llmApologise();
    return;
  }
  reply.trim();
  if (!reply.length()) { Serial.println("[LLM] (empty reply)"); return; }

  Serial.printf("[LLM] %s  (%lu ms)\n", reply.c_str(), (unsigned long)ms);
  histPush(llmPending, reply);

  String spoken = speechClean(reply);
  if (spoken.length() && !ttsSpeak(spoken)) Serial.println("[TTS] could not speak the reply");
}

// Call from loop(). Non-blocking: polls for Ollama's reply.
static bool llmService() {
  if (llmState == LLM_IDLE) return false;
  int status = 0;
  const char *body = nullptr;
  bool got = false;
  int r = httpPoll(llmClient, &status, &body, &got);
  if (r == 1) { llmClient.stop(); llmState = LLM_IDLE; llmHandleResponse(status, body); return true; }
  if (r == 2) { llmClient.stop(); llmState = LLM_IDLE; llmFail("Ollama closed the connection without a reply"); return true; }
  if ((int32_t)(millis() - llmT0) > (int32_t)LLM_RESPONSE_TIMEOUT_MS) {
    llmClient.stop(); llmState = LLM_IDLE; llmFail("timeout waiting for Ollama");
    return true;
  }
  return got;
}

// ============================ MIC: CAPTURE + VAD ============================
static bool micIsListening() {
  return micListen && sttState == STT_IDLE && llmState == LLM_IDLE && ttsState == TTS_IDLE && (int32_t)(millis() - micMuteUntil) >= 0;
}

static float frameRms(const int16_t *f) {
  uint64_t sum = 0;
  for (size_t i = 0; i < MIC_FRAME_SAMPLES; i++) sum += (uint64_t)((int32_t)f[i] * (int32_t)f[i]);
  return sqrtf((float)sum / (float)MIC_FRAME_SAMPLES);
}

static void ringPush(const int16_t *f) {
  memcpy(ring[ringHead], f, sizeof ring[0]);
  ringHead = (ringHead + 1) % VAD_PREROLL_FRAMES;
  if (ringCount < VAD_PREROLL_FRAMES) ringCount++;
}

static void captureBegin(bool forced) {
  uttLen = 0;
  size_t start = (ringHead + VAD_PREROLL_FRAMES - ringCount) % VAD_PREROLL_FRAMES;   // oldest frame first
  for (size_t i = 0; i < ringCount; i++) {
    if (uttLen + MIC_FRAME_SAMPLES > uttCap) break;
    memcpy(uttBuf + uttLen, ring[(start + i) % VAD_PREROLL_FRAMES], sizeof ring[0]);
    uttLen += MIC_FRAME_SAMPLES;
  }
  lastVoiceEnd = uttLen;
  silenceRun = 0;
  voicedFrames = forced ? 0 : startRun;
  forcedRec = forced;
  vadState = VAD_CAPTURE;
  Serial.println(forced ? "[MIC] recording..." : "[MIC] speech detected...");
}

static void captureEnd(const char *why) {
  vadState = VAD_IDLE;
  startRun = 0;
  ringCount = ringHead = 0;                              // anything heard while transcribing is dropped

  if (!forcedRec) {
    size_t keep = lastVoiceEnd + (size_t)MIC_SAMPLE_RATE * VAD_TRAIL_KEEP_MS / 1000;   // trim trailing silence
    if (keep < uttLen) uttLen = keep;
    uint32_t speechMs = voicedFrames * FRAME_MS;
    if (speechMs < VAD_MIN_SPEECH_MS) { Serial.printf("[MIC] ignored (%lu ms of speech)\n", (unsigned long)speechMs); return; }
  }
  Serial.printf("[MIC] captured %.2f s (%s)\n", (float)uttLen / MIC_SAMPLE_RATE, why);
  sttSubmit(uttLen);                                     // on failure it already printed why; we simply keep listening
}

static void micForceRecord(uint32_t ms) {
  if (!micIsListening() || vadState != VAD_IDLE) { Serial.println("[MIC] busy (speaking / transcribing / listening off)"); return; }
  forceUntil = millis() + ms;
  startRun = 0;
  captureBegin(true);
}

// One 20 ms frame of conditioned 16-bit audio.
static void micFrame(const int16_t *f) {
  float rms = frameRms(f);
  if (rms > lvRmsMax) lvRmsMax = rms;
  if (noiseFloor < 0) noiseFloor = rms;                  // first frame seeds the floor

  bool listening = micIsListening();
  static bool wasListening = false;
  if (listening != wasListening) {
    wasListening = listening;
    if (listening) Serial.println("[MIC] listening - go ahead and speak");
  }
  if (!listening) {                                      // speaking, thinking, transcribing, warming up or switched off
    vadState = VAD_IDLE;
    startRun = 0;
    ringCount = ringHead = 0;
    return;
  }

  float startThr = fmaxf((float)VAD_MIN_RMS, noiseFloor * VAD_START_RATIO);
  lvStartThr = startThr;

  if (vadState == VAD_IDLE) {
    ringPush(f);                                         // keep the last 400 ms for pre-roll
    if (rms > startThr) {
      if (++startRun >= VAD_START_FRAMES) captureBegin(false);
    } else {
      startRun = 0;
      noiseFloor += (rms - noiseFloor) * (rms < noiseFloor ? 0.2f : 0.01f);   // fast down, slow up
    }
    return;
  }

  // ---- VAD_CAPTURE ----
  if (uttLen + MIC_FRAME_SAMPLES > uttCap) { captureEnd("buffer full"); return; }
  memcpy(uttBuf + uttLen, f, sizeof ring[0]);
  uttLen += MIC_FRAME_SAMPLES;

  float endThr = fmaxf(VAD_MIN_RMS * 0.6f, noiseFloor * VAD_END_RATIO);
  if (rms > endThr) { silenceRun = 0; voicedFrames++; lastVoiceEnd = uttLen; }
  else silenceRun++;

  if (forcedRec) {
    if ((int32_t)(millis() - forceUntil) >= 0) captureEnd("timer");
  } else if (silenceRun * FRAME_MS >= VAD_END_SILENCE_MS) {
    captureEnd("end of speech");
  }
}

// Call from loop(). Non-blocking: drains whatever the mic DMA has and feeds the VAD.
static void micService() {
  static int32_t  buf[256];
  static uint32_t lastPrint = 0;

  for (int rd = 0; rd < 4; rd++) {
    size_t br = 0;
    i2s_channel_read(rxChan, buf, sizeof buf, &br, 0);   // timeout 0 -> non-blocking
    size_t n = br / sizeof(int32_t);
    if (!n) break;

    for (size_t i = 0; i < n; i++) {
      int32_t s = buf[i] >> 8;                           // INMP441: 24-bit signed, left-justified in 32 bits
      dcEst += (s - dcEst) >> 10;                        // one-pole DC blocker (~2.5 Hz corner)
      int32_t x = ((s - dcEst) * micGain) >> 8;          // 24 -> 16 bit scale, times software gain
      if (x > 32767) x = 32767; else if (x < -32768) x = -32768;
      frameBuf[frameFill++] = (int16_t)x;
      if (frameFill == MIC_FRAME_SAMPLES) { frameFill = 0; micFrame(frameBuf); }
    }
  }

  uint32_t now = millis();
  if (micLevelPrint && now - lastPrint >= 1000) {
    const char *st = vadState == VAD_CAPTURE ? "capturing" : (micIsListening() ? "listening" : "deaf (speaking/transcribing/off)");
    Serial.printf("[MIC] loudest 20ms rms=%d  floor=%d  start-threshold=%d  gain=%ld  %s\n",
                  (int)lvRmsMax, (int)noiseFloor, (int)lvStartThr, (long)micGain, st);
    lvRmsMax = 0;
    lastPrint = now;
  }
}

// Utterance buffer: PSRAM if available (N16R8 -> pick "OPI PSRAM" in the Tools menu), else a short internal buffer.
static bool uttAlloc() {
  size_t secs = 0;
  if (psramFound()) {
    uttBuf = (int16_t *)ps_malloc((size_t)MIC_SAMPLE_RATE * UTT_MAX_SECONDS * 2);
    if (uttBuf) secs = UTT_MAX_SECONDS;
  }
  if (!uttBuf) {
    for (size_t s = 6; s >= 2; s--) {
      uttBuf = (int16_t *)malloc((size_t)MIC_SAMPLE_RATE * s * 2);
      if (uttBuf) { secs = s; break; }
    }
  }
  if (!uttBuf) return false;
  uttCap = (size_t)MIC_SAMPLE_RATE * secs;
  Serial.printf("[BOOT] utterance buffer: %u s in %s\n", (unsigned)secs, secs == UTT_MAX_SECONDS && psramFound() ? "PSRAM" : "internal RAM");
  return true;
}

// ================================== SERIAL ==================================
static void printHelp() {
  Serial.println("Speak, or type a message to chat. Commands: /say <text>  /stop  /reset  /vol 0-100  /listen on|off  /rec [sec]  /gain 1-64  /level");
}

static bool pipelineBusy() { return ttsState != TTS_IDLE || sttState != STT_IDLE || llmState != LLM_IDLE; }

static void pipelineCancel() {
  if (llmState != LLM_IDLE) { llmClient.stop(); llmState = LLM_IDLE; }
  if (sttState != STT_IDLE) sttFinish();
  ttsCancel();
}

static void handleCommand(String s) {
  s.trim();
  if (!s.length()) return;

  if (s[0] == '/') {
    String cmd = s, arg;
    int sp = s.indexOf(' ');
    if (sp > 0) { cmd = s.substring(0, sp); arg = s.substring(sp + 1); arg.trim(); }
    cmd.toLowerCase();

    if (cmd == "/stop") {
      pipelineCancel();
    } else if (cmd == "/say") {
      if (!arg.length()) printHelp();
      else if (pipelineBusy()) Serial.println("[TTS] busy - send /stop first");
      else ttsSpeak(arg);
    } else if (cmd == "/reset") {
      histCount = 0;
      Serial.println("[LLM] conversation history cleared");
    } else if (cmd == "/vol") {
      int v = arg.toInt();
      if (v < 0) v = 0;
      if (v > 100) v = 100;
      gainQ8 = (uint32_t)v * 256 / 100;
      Serial.printf("[VOL] %d%%\n", v);
    } else if (cmd == "/gain") {
      int g = arg.toInt();
      if (g < 1) g = 1;
      if (g > 64) g = 64;
      micGain = g;
      Serial.printf("[MIC] gain %d\n", g);
    } else if (cmd == "/listen") {
      if (arg.equalsIgnoreCase("off")) micListen = false;
      else if (arg.equalsIgnoreCase("on")) micListen = true;
      Serial.printf("[MIC] listening %s\n", micListen ? "ON" : "OFF");
    } else if (cmd == "/level") {
      micLevelPrint = !micLevelPrint;
      Serial.printf("[MIC] level print %s\n", micLevelPrint ? "ON" : "OFF");
    } else if (cmd == "/rec") {
      int sec = arg.length() ? arg.toInt() : 5;
      int maxSec = (int)(uttCap / MIC_SAMPLE_RATE);
      if (sec < 1) sec = 1;
      if (sec > maxSec) sec = maxSec;
      micForceRecord((uint32_t)sec * 1000);
    } else {
      printHelp();
    }
    return;
  }

  // plain text: same path as a transcribed voice question (handy for testing without the mic)
  if (pipelineBusy()) { Serial.println("[LLM] busy - wait for the current exchange to finish (or /stop)"); return; }
  Serial.printf("[YOU] %s\n", s.c_str());
  llmAsk(s);
}

static void serialService() {
  static String cmd;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { handleCommand(cmd); cmd = ""; }
    else if (cmd.length() < 1000) cmd += c;
  }
}

// ============================== SETUP / LOOP ================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32-S3 voice assistant (Whisper STT -> Ollama LLM -> streamed Kokoro TTS)");

  if (!uttAlloc()) Serial.println("[BOOT] could not allocate the utterance buffer - STT disabled");
  if (!audioBegin()) Serial.println("[BOOT] I2S init failed - check pins / core version");
  micMuteUntil = millis() + 1500;                        // let the mic's DC offset settle before the VAD starts judging

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                                  // modem sleep adds latency spikes that hurt streaming
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t > 30000) { Serial.println("\n[WIFI] timeout - restarting"); ESP.restart(); }
    delay(250);
    Serial.print('.');
  }
  Serial.printf("\n[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str());
  printHelp();

  Serial.printf("[ASSISTANT] %s\n", GREETING_TEXT);   // step 1: greet - then the mic opens by itself once the greeting has finished
  ttsSpeak(GREETING_TEXT);
}

void loop() {
  bool busy = ttsService();      // speaker: network -> DMA state machine (non-blocking)
  busy |= sttService();          // Whisper upload / response state machine (non-blocking slices)
  busy |= llmService();          // Ollama reply poller (non-blocking)
  if (uttBuf) micService();      // mic -> VAD -> utterance capture
  serialService();
  if (!busy) delay(1);           // yield; 1 ms polling is plenty against the ~340 ms TX and ~128 ms RX DMA rings
}

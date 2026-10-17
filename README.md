# StreamVoice-S3
### A fully local, streaming voice assistant for the ESP32-S3

![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)
![Framework](https://img.shields.io/badge/framework-Arduino--ESP32%203.x-00979D)
![Status](https://img.shields.io/badge/status-experimental-orange)

Talk to your own AI server from a $10 microcontroller. StreamVoice-S3 listens through an I2S microphone, transcribes your speech with **Whisper**, thinks with a local LLM running in **Ollama**, and speaks the answer with **Kokoro TTS**, with the audio **streamed byte-for-byte from the network straight into the I2S DMA buffers**. Nothing ever has to be downloaded into RAM before playback starts.


## Features

- **True streaming playback.** The HTTP response is parsed on the fly and piped into the I2S DMA ring as it arrives. No response buffer, so the first audio plays about a tenth of a second after the request is sent (on a typical LAN).
- **Back-pressure, not buffering.** When the DMA ring is full the firmware simply stops reading the socket and TCP flow control slows the server down.
- **Robust HTTP handling.** A small hand-written HTTP/1.1 client handles `Transfer-Encoding: chunked`, `Content-Length` and read-until-close, so chunk-size lines never end up in your audio as clicks.
- **Dynamic WAV parsing.** Walks the RIFF chunks instead of assuming a 44-byte header, and takes the sample rate and channel count from the stream (the placeholder data size that streaming servers write is ignored).
- **Hands-free listening.** Energy-based voice activity detection with an adaptive noise floor, 400 ms pre-roll so first syllables aren't clipped, and automatic end-of-speech detection.
- **No self-talk.** The mic is deaf while the assistant is thinking and speaking, plus a short tail afterwards.
- **Non-blocking by design.** Everything runs from `loop()` as small state machines. The only bounded-blocking calls are the TCP connects and the 4 KB upload slices.
- **Conversation memory.** The last few exchanges are sent to the LLM with each question.
- **No external libraries.** Only what ships with the Arduino-ESP32 core.

## How a conversation works

1. **Greet.** The assistant prints and speaks a greeting at boot.
2. **Listen.** The VAD waits for speech. Serial shows `[MIC] listening - go ahead and speak`.
3. **Transcribe.** The utterance is wrapped in a WAV header and uploaded to Whisper as `multipart/form-data`. The transcript is printed.
4. **Think.** The transcript (plus recent history) goes to Ollama's `/api/chat`. The reply is printed.
5. **Speak.** The reply is sent to the TTS server and streamed straight to the speaker. Then the mic opens again.

Example Serial output:

```
[ASSISTANT] Hello! I am ready. What would you like to talk about?
[TTS] HTTP 200, body framing: chunked
[TTS] WAV 24000 Hz, 1 ch, 16-bit  (first audio after 82 ms)
[MIC] listening - go ahead and speak
[MIC] speech detected...
[MIC] captured 2.10 s (end of speech)
[STT] upload done in 41 ms, waiting for transcript...
[STT] you said: What's the tallest mountain in the world?  (640 ms)
[LLM] thinking...
[LLM] Mount Everest, at about 8,849 metres.  (1214 ms)
[TTS] done: 3.02 s of audio (145000 bytes PCM) in 3120 ms
```

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 dev board, **N16R8** (16 MB flash, 8 MB octal PSRAM) | PSRAM holds the utterance buffer (up to 20 s). Without PSRAM the sketch falls back to a short internal buffer. |
| MAX98357A I2S amplifier | Drives the speaker directly. |
| INMP441 I2S MEMS microphone | 24-bit, left channel (L/R pin to GND). |
| 4-8 ohm speaker | 3 W is plenty. |

### Default wiring

| Signal | ESP32-S3 GPIO |
|---|---|
| MAX98357A **BCLK** | 15 |
| MAX98357A **LRC** | 16 |
| MAX98357A **DIN** | 7 |
| INMP441 **SCK** | 4 |
| INMP441 **WS** | 5 |
| INMP441 **SD** | 6 |

- Power the INMP441 from **3.3 V** and tie its **L/R** pin to **GND**.
- Power the MAX98357A from 3.3 V or 5 V (5 V is louder). Leave SD and GAIN at their defaults to start.
- Share a common **GND** between all boards.
- All pins are `#define`s at the top of the sketch. On an N16R8 **avoid GPIO 26-37** (flash and octal PSRAM), **19/20** (USB) and the strapping pins **0, 3, 45, 46**.

## Server requirements

The ESP32 talks to three services on your network. The defaults assume they all live on `192.168.1.4`.

| Service | Default endpoint | Notes |
|---|---|---|
| Text-to-speech | `POST :8880/v1/audio/speech` | OpenAI-style speech API returning WAV (for example [Kokoro-FastAPI](https://github.com/remsky/Kokoro-FastAPI)). Body: `{"model","input","voice","response_format":"wav","speed"}`. |
| Speech-to-text | `POST :3000/api/v1/audio/transcriptions` | `multipart/form-data` with a `file` field; must reply with JSON containing a `"text"` field. |
| LLM | `POST :11434/api/chat` | [Ollama](https://ollama.com) native API, model `llama3.2:1b` (`ollama pull llama3.2:1b`). |

The TTS and STT requests send `Authorization: Bearer <API_KEY>`. Set `API_KEY` to `""` if your servers don't need one.

## Getting started

1. **Install the Arduino-ESP32 core 3.0 or newer** (Boards Manager, "esp32 by Espressif Systems"). The sketch uses the `driver/i2s_std.h` API, which does not exist in 2.x.
2. **Select the board and options** under *Tools*:
   - Board: **ESP32S3 Dev Module**
   - Flash Size: **16MB**
   - PSRAM: **OPI PSRAM** (never *QSPI PSRAM* on an N16R8, or the board will not boot)
   - USB CDC On Boot: **Enabled** if you use the native USB port for Serial
3. **Clone the repo** and open `esp32s3_voice_stream/esp32s3_voice_stream.ino`.
4. **Edit the config block** at the top: Wi-Fi credentials, `API_KEY`, and the server addresses if they differ.
5. **Upload**, open the Serial Monitor at **115200 baud**, and say something after the greeting.

> **Before you commit:** the sketch keeps `WIFI_SSID`, `WIFI_PASS` and `API_KEY` in the source. Don't push real credentials to a public repo.

## Configuration

All settings are `#define`s or constants at the top of the sketch.

| Setting | Default | Purpose |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASS` | placeholders | Wi-Fi credentials |
| `API_KEY` | `"YOUR_API_KEY"` | Bearer token for TTS and STT (`""` = send no header) |
| `TTS_HOST` / `TTS_PORT` / `TTS_PATH` | `192.168.1.4` / `8880` / `/v1/audio/speech` | TTS endpoint |
| `TTS_MODEL` / `TTS_VOICE` / `TTS_SPEED` | `kokoro` / `af_bella` / `1.0` | TTS request parameters |
| `STT_HOST` / `STT_PORT` / `STT_PATH` | `192.168.1.4` / `3000` / `/api/v1/audio/transcriptions` | Whisper endpoint |
| `STT_MODEL` / `STT_LANGUAGE` | `whisper-1` / `en` | Sent as form fields (`""` language = auto-detect). Some servers ignore the model field. |
| `LLM_HOST` / `LLM_PORT` / `LLM_PATH` | `192.168.1.4` / `11434` / `/api/chat` | Ollama endpoint |
| `LLM_MODEL` | `llama3.2:1b` | Ollama model name |
| `LLM_MAX_TOKENS` | `160` | Caps reply length |
| `LLM_HISTORY_TURNS` | `3` | Remembered question/answer pairs (minimum 1) |
| `LLM_SYSTEM_PROMPT` | short, plain-speech prompt | Keeps answers brief and TTS-friendly |
| `LLM_USE_API_KEY` | `0` | Send the Bearer key to the LLM too |
| `GREETING_TEXT` | "Hello! I am ready..." | Spoken at boot |
| `MIC_GAIN_DEFAULT` | `8` | Software gain applied to the INMP441 signal |
| `VAD_MIN_RMS` | `250` | Absolute floor for the speech-start threshold |
| `VAD_END_SILENCE_MS` | `800` | Silence that ends an utterance |
| `VAD_MIN_SPEECH_MS` | `300` | Shorter bursts are discarded |
| `UTT_MAX_SECONDS` | `20` | Longest utterance (PSRAM buffer size) |
| `MIC_MUTE_TAIL_MS` | `600` | Deaf time after the speaker stops |
| `VOLUME_PCT_DEFAULT` | `80` | Speaker digital volume |
| `PREROLL_MS` | `120` | Leading silence queued before TTS audio (jitter headroom) |

## Serial commands

Type a plain message to chat with the LLM without speaking. Commands:

| Command | Effect |
|---|---|
| `/say <text>` | Speak text directly (skips the LLM) |
| `/stop` | Abort whatever is in progress |
| `/reset` | Forget the conversation history |
| `/vol 0-100` | Speaker volume |
| `/listen on\|off` | Enable or disable voice activity detection |
| `/rec [seconds]` | Record immediately for N seconds (default 5) and transcribe, bypassing the VAD |
| `/gain 1-64` | Microphone gain |
| `/level` | Toggle a once-per-second print of mic level and VAD thresholds |

### Tuning the microphone

The INMP441 is quiet, so you will probably want to adjust two numbers. Run `/level` and speak:

```
[MIC] loudest 20ms rms=2400  floor=40  start-threshold=250  gain=8  listening
```

- Speech should peak **well above** `start-threshold`, and silence should sit **well below** it.
- Too quiet? Raise the gain live with `/gain 12`, then bake it into `MIC_GAIN_DEFAULT`.
- Triggering on noise? Raise `VAD_MIN_RMS` or lower the gain.

## Design notes

**Streaming TTS path.** `HTTPClient` is deliberately not used. Servers such as Kokoro-FastAPI stream with chunked transfer encoding, and a raw stream pipe would feed the chunk-size lines into the DAC. Instead a small state machine reads the socket, strips the HTTP framing, parses the WAV header, then applies volume and mono-to-stereo expansion on slices of about 1 KB. The I2S transmit channel runs with `auto_clear` so an underrun produces silence rather than repeated audio. The DMA ring (16 x 512 frames, about 340 ms at 24 kHz) is the only buffer.

**Why mono is duplicated to both channels.** The MAX98357A's output channel depends on how its SD_MODE pin is wired. Sending identical samples on left and right makes the result independent of that.

**Why STT is buffered (and TTS isn't).** Whisper endpoints take a complete file and need a `Content-Length`, so each utterance is held in PSRAM and uploaded straight from that buffer in 4 KB slices. Playback has no such constraint, so it stays unbuffered.

**Voice activity detection.** 20 ms frames, RMS energy against an adaptive noise floor (falls fast, rises slowly), three consecutive loud frames to start, 800 ms of quiet to stop, and a pre-roll ring so the first syllable is kept.

**Memory footprint (approx.).** 32 KB TX DMA, 8 KB RX DMA, 13 KB pre-roll ring, 8 KB HTTP response buffers, about 3 KB streaming slices, and up to 640 KB of PSRAM for the utterance.

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Board boot-loops on an N16R8 | PSRAM is set to *QSPI*. Choose **OPI PSRAM**. |
| Compile errors mentioning `i2s_*` | Arduino-ESP32 core older than 3.0. |
| `[TTS] connect failed` / `[STT] connect failed` | Wrong IP or port, or a firewall on the server. |
| `[TTS] HTTP 401` or `[STT] HTTP 401` | Missing or wrong `API_KEY`. |
| `[STT] HTTP 4xx` | Check the endpoint path, the model name and the key. The server's error text is printed. |
| `[STT] unexpected reply` | The server didn't return JSON with a `"text"` field. |
| `[STT] (nothing recognised)` | Audio too quiet or wrong language. Raise `/gain`, check `STT_LANGUAGE`. |
| `[LLM] HTTP 404 ... model not found` | Run `ollama pull llama3.2:1b` on the server. |
| First answer takes a long time | Ollama is loading the model. Later answers are much faster. |
| Never triggers on speech | Run `/level`. Speech must exceed `start-threshold`: raise gain or lower `VAD_MIN_RMS`. |
| Triggers constantly | Background noise. Raise `VAD_MIN_RMS` or lower the gain. |
| The assistant answers itself | Raise `MIC_MUTE_TAIL_MS`, lower `/vol`, or move the mic away from the speaker. |
| No sound but the logs look fine | Check speaker wiring, the amplifier's SD pin, `/vol`, and that BCLK and LRC aren't swapped. |

Built on [Arduino-ESP32](https://github.com/espressif/arduino-esp32), [Whisper](https://github.com/openai/whisper), [Ollama](https://ollama.com), and the [Kokoro](https://huggingface.co/hexgrad/Kokoro-82M) TTS model.


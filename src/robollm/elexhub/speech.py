"""Offline speech adapter. Audio remains local; recognized text still needs preview."""

from __future__ import annotations

import io
import json
import wave


class Speech:
    def __init__(self, model_path):
        from vosk import Model, SetLogLevel

        SetLogLevel(-1)
        self.model = Model(str(model_path))

    def transcribe(self, wav_bytes):
        from vosk import KaldiRecognizer

        if len(wav_bytes) > 2_000_000:
            raise ValueError("audio clip is too long")
        with wave.open(io.BytesIO(wav_bytes), "rb") as audio:
            if (
                audio.getnchannels() != 1
                or audio.getsampwidth() != 2
                or audio.getframerate() != 16000
                or audio.getnframes() > 16000 * 15
            ):
                raise ValueError(
                    "use mono 16-bit PCM WAV at 16 kHz, at most 15 seconds"
                )
            recognizer = KaldiRecognizer(
                self.model,
                16000,
                json.dumps(
                    [
                        "reach here",
                        "move this red box here",
                        "move the red box here",
                        "move this here",
                        "put this here",
                        "[unk]",
                    ]
                ),
            )
            recognizer.AcceptWaveform(audio.readframes(audio.getnframes()))
            result = json.loads(recognizer.FinalResult())
        return result.get("text", "")

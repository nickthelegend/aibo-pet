"""build_voice.py — pre-render every line AIBO speaks.

Rendered with Kokoro, an open weights 82M parameter model, not with a
system voice.

Lisa (lisa.locomotive.ca) does not run text to speech in the browser. Its
network log is a bank of static files:

    /assets/lisa/en/lisa.intro.3.mp3
    /assets/lisa/en/lisa.greeting.9.mp3

Numbered variants of each line, rendered ahead of time with a real voice and
served as audio. That is why it sounds like a person and not like a browser.
speechSynthesis, which the first version of this used, is robotic, differs on
every machine, and on some has no usable English voice at all.

So this does the same thing: renders each line once with the system voice,
converts to mp3, and writes a manifest the page loads. The browser only ever
plays audio.

    .venv/bin/python cad/build_voice.py          # render changed lines
    .venv/bin/python cad/build_voice.py --force  # re-render everything
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.normpath(os.path.join(HERE, "..", "web"))
OUT = os.path.join(WEB, "voice")

# Kokoro (hexgrad/Kokoro-82M) via onnxruntime. The first version shelled out
# to macOS `say`, which is the same class of robot voice as speechSynthesis --
# fine as a placeholder, not fine as the thing a visitor hears. Kokoro is an
# open weights model that actually sounds like a person, and it renders here
# at roughly 2.5x realtime on CPU, so the whole bank is a few seconds.
MODEL = os.path.join(HERE, "..", ".models", "kokoro-v1.0.onnx")
VOICES = os.path.join(HERE, "..", ".models", "voices-v1.0.bin")
VOICE = "af_heart"     # warm, mid pace; af_nicole and am_echo are the alts
SPEED = 1.0

# id -> [variants]. Multiple variants are picked between at random, which is
# what stops a repeated line sounding like a recording on the second hearing.
LINES = {
    "intro": [
        "Hello. I am a desk lamp with four servos and opinions about where you are sitting.",
        "Oh good, someone. I am a desk lamp, and I have been waiting to be useful.",
    ],
    "menu": [
        "What do you want to know?",
        "Ask me something.",
    ],
    "move": [
        "Four joints. Base, shoulder, elbow, and a tilt at the head. "
        "Every move runs through a spring that overshoots a little, which is "
        "the difference between a servo turning and something looking alive.",
    ],
    "need": [
        "Three big servos, one small one, a microphone, a speaker, and a board "
        "to run them. Twenty two printed parts, and every one fits the smallest "
        "Bambu bed.",
    ],
    "print": [
        "Nine plates. Start with the test plate, it takes fifteen minutes and "
        "tells you whether the printed spline fits your servo before you commit "
        "to an eight hour tub.",
    ],
    "why": [
        "Because a lamp that only switches on is furniture. This one notices "
        "you walked in.",
    ],
    "ask_name": [
        "Before I send anything. What should I call you?",
    ],
    "ask_printer": [
        "What are you printing on?",
    ],
    "ask_email": [
        "And where do I send the files?",
    ],
    "bad_email": [
        "That does not look like an email address. Try again.",
    ],
    "done": [
        "You are on the list. I will be here, leaning slightly to the left.",
    ],
    "again": [
        "Again? Fine. I have nowhere else to be.",
    ],
}


def have(cmd):
    return shutil.which(cmd) is not None


_kokoro = None


def kokoro():
    global _kokoro
    if _kokoro is None:
        from kokoro_onnx import Kokoro
        if not os.path.exists(MODEL):
            raise SystemExit(
                "missing .models/kokoro-v1.0.onnx and voices-v1.0.bin -- fetch "
                "them from the kokoro-onnx releases page")
        _kokoro = Kokoro(MODEL, VOICES)
    return _kokoro


def render(text, path):
    """Kokoro -> wav -> mp3."""
    import soundfile as sf
    samples, sr = kokoro().create(text, voice=VOICE, speed=SPEED, lang="en-us")
    wav = path + ".wav"
    sf.write(wav, samples, sr)
    if have("lame"):
        subprocess.run(["lame", "--quiet", "-V", "5", wav, path], check=True)
    elif have("ffmpeg"):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", wav,
                        "-codec:a", "libmp3lame", "-qscale:a", "5", path],
                       check=True)
    else:
        raise SystemExit("need lame or ffmpeg to make mp3")
    os.remove(wav)


def main():
    force = "--force" in sys.argv

    os.makedirs(OUT, exist_ok=True)

    manifest, made, kept = {}, 0, 0
    for key, variants in LINES.items():
        manifest[key] = []
        for i, text in enumerate(variants):
            name = f"{key}.{i}.mp3"
            path = os.path.join(OUT, name)
            # hash the text so an edited line re-renders and an untouched one
            # does not; the whole bank takes a while otherwise
            sig = hashlib.sha1(f"kokoro|{VOICE}|{SPEED}|{text}".encode()).hexdigest()[:10]
            sig_path = path + ".sig"
            cur = open(sig_path).read().strip() if os.path.exists(sig_path) else ""
            if force or cur != sig or not os.path.exists(path):
                render(text, path)
                open(sig_path, "w").write(sig)
                made += 1
            else:
                kept += 1
            manifest[key].append({"src": f"./voice/{name}", "text": text})

    with open(os.path.join(OUT, "lines.json"), "w") as f:
        json.dump({"engine": "kokoro-82M", "voice": VOICE, "lines": manifest},
                  f, indent=1)

    total = sum(os.path.getsize(os.path.join(OUT, f))
                for f in os.listdir(OUT) if f.endswith(".mp3"))
    print(f"voice: {made} rendered, {kept} unchanged, "
          f"{sum(len(v) for v in manifest.values())} clips, {total/1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())

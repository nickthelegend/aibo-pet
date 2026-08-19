"""build_voice.py — pre-render every line AIBO speaks.

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

VOICE = "Samantha"     # clearest of the built-in en_US voices
RATE = 178             # words per minute; the default 175 drags slightly

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


def render(text, path):
    """say -> aiff -> mp3. afconvert ships with macOS; lame and ffmpeg are
    both common. Whichever exists wins."""
    aiff = path + ".aiff"
    subprocess.run(["say", "-v", VOICE, "-r", str(RATE), "-o", aiff, text],
                   check=True)
    if have("lame"):
        subprocess.run(["lame", "--quiet", "-V", "5", aiff, path], check=True)
    elif have("ffmpeg"):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", aiff,
                        "-codec:a", "libmp3lame", "-qscale:a", "5", path],
                       check=True)
    else:
        raise SystemExit("need lame or ffmpeg to make mp3")
    os.remove(aiff)


def main():
    force = "--force" in sys.argv
    if not have("say"):
        raise SystemExit("`say` not found: this build step needs macOS")
    os.makedirs(OUT, exist_ok=True)

    manifest, made, kept = {}, 0, 0
    for key, variants in LINES.items():
        manifest[key] = []
        for i, text in enumerate(variants):
            name = f"{key}.{i}.mp3"
            path = os.path.join(OUT, name)
            # hash the text so an edited line re-renders and an untouched one
            # does not; the whole bank takes a while otherwise
            sig = hashlib.sha1(f"{VOICE}|{RATE}|{text}".encode()).hexdigest()[:10]
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
        json.dump({"voice": VOICE, "rate": RATE, "lines": manifest}, f, indent=1)

    total = sum(os.path.getsize(os.path.join(OUT, f))
                for f in os.listdir(OUT) if f.endswith(".mp3"))
    print(f"voice: {made} rendered, {kept} unchanged, "
          f"{sum(len(v) for v in manifest.values())} clips, {total/1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())

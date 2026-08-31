"""stages.py — the build order the video shows.

Grounded in cad/v2_insert.py's verified order: every part that audit walks
appears here in the same sequence. The audit only walks parts whose
APPROACH could be blocked, so the ones it takes for granted -- the tub
itself, the inner plates that are placed before their servo, the spacers --
are filled in around it. Nothing is invented: the relative order of any two
parts the audit does check is the audit's.

A token ending in "-" is a FAMILY prefix; anything else is an exact part
name. Without that rule "v2-disc" swallows both disc keys and "v2-screw"
swallows the elbow screw, and three parts get built twice.
"""


def match(part, token):
    return part.startswith(token) if token.endswith("-") else part == token


STAGES = [
    ("The shell",            ["v2-tub"]),
    ("Brains",               ["esp32-"]),
    ("Amp and speaker",      ["amp-", "spk-"]),
    ("Microphone",           ["mic-"]),
    ("The button",           ["mx-", "v2-keycap"]),
    ("Pan servo",            ["pan-"]),
    ("Its horn",             ["horn-pan-"]),
    ("The turntable",        ["v2-disc"]),
    ("Two keys lock it",     ["v2-disckey-"]),
    ("The tower",            ["v2-tower"]),
    ("Shoulder servo",       ["sh-"]),
    ("Shoulder horn",        ["horn-shoulder-"]),
    ("Link 1, inner",        ["v2-link1-in", "v2-link1-spacers",
                              "v2-link1-ledges", "v2-screw"]),
    ("Link 1, outer",        ["v2-link1-out"]),
    ("Elbow servo",          ["el-"]),
    ("Elbow horn",           ["horn-elbow-"]),
    ("Link 2, inner",        ["v2-link2-in", "v2-link2-spacers",
                              "v2-screw-elbow"]),
    ("Link 2, outer",        ["v2-link2-out"]),
    ("The head",             ["v2-head"]),
    ("Head servo and horn",  ["hd-", "horn-head-"]),
    ("The cone",             ["shade"]),
    ("LED ring",             ["ring-"]),
    ("Cap it",               ["v2-conecap"]),
]

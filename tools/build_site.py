"""build_site.py — one source of truth for the Thenar site chrome.

Six pages sharing a hand-copied header is how a site ends up with a nav that
says one thing on the landing page and another everywhere else. It already
happened once on the Hotaru site. So the header, the footer and the <head>
live here, every page is emitted from this file, and index.html gets its
chrome patched between markers rather than maintained by hand.

    .venv/bin/python tools/build_site.py
"""
from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "thenar"))

SITE = "https://thenar.io"
TITLE_SUFFIX = "THENAR"

# slug, nav label, page title, meta description
NAV = [
    ("products", "Products", "Products",
     "The Thenar Band, Quest 3S capture, and Hotaru. What each one records and what state it is in."),
    ("protocol", "Protocol", "Protocol",
     "How contact data is committed, licensed and settled on Avalanche, and what is deliberately kept off chain."),
    ("market", "Market", "Go to market",
     "Who buys contact data for manipulation, how we reach them, and what we charge."),
    ("faq", "FAQ", "FAQ",
     "The questions we get asked, including the hostile ones."),
    ("company", "Company", "Company",
     "What Thenar has shipped, what is still in design, and where the company actually stands."),
]

WORDMARK = ('<svg viewBox="0 0 340 92" aria-hidden="true" style="height:24px">'
            '<text x="0" y="70" font-family="Manrope,sans-serif" font-size="72" '
            'font-weight="700" letter-spacing="-2" fill="currentColor">THENAR</text></svg>')


def header(active: str) -> str:
    """`active` is a slug, or "home", or "" for none."""
    links = []
    for slug, label, _t, _d in NAV:
        cur = ' aria-current="page"' if slug == active else ""
        links.append(f'    <a href="./{slug}.html"{cur}>{label}</a>')
    return (
        '<!-- NAV:START -->\n'
        '<header class="top solid">\n'
        f'  <a class="brand" href="./index.html" aria-label="Thenar home">{WORDMARK}</a>\n'
        '  <nav class="topnav" aria-label="Primary">\n'
        + "\n".join(links) + "\n"
        '  </nav>\n'
        '  <a class="btn primary" href="./index.html#contact">Talk to us</a>\n'
        '</header>\n'
        '<!-- NAV:END -->'
    )


def footer() -> str:
    cols = [
        ("Products", [("./products.html", "Overview"),
                      ("./index.html#band", "Thenar Band"),
                      ("./index.html#capture", "Quest capture"),
                      ("https://aibo.loompad.tech", "Hotaru")]),
        ("Protocol", [("./protocol.html", "Architecture"),
                      ("./index.html#chain", "What goes on chain"),
                      ("./faq.html", "FAQ")]),
        ("Company", [("./company.html", "Status"),
                     ("./market.html", "Go to market"),
                     ("https://github.com/nickthelegend/aibo-pet", "Source")]),
        ("Legal", [("./privacy.html", "Privacy"),
                   ("./terms.html", "Terms")]),
    ]
    out = ['<!-- FOOT:START -->', '<footer><div class="fwrap">',
           '  <div class="fcols">']
    for title, links in cols:
        out.append(f'    <div><h4>{title}</h4>')
        for href, label in links:
            out.append(f'      <a href="{href}">{label}</a>')
        out.append('    </div>')
    out += ['  </div>',
            '  <div class="fbase">',
            '    <span>THENAR — contact data for physical AI.</span>',
            '    <span>Built in the open. Quest and Meta are trade marks of '
            'Meta Platforms, Inc. Thenar is not affiliated with Meta.</span>',
            '  </div>', '</div></footer>', '<!-- FOOT:END -->']
    return "\n".join(out)


def head(title: str, desc: str, slug: str, extra_css: str = "") -> str:
    url = f"{SITE}/{'' if slug == 'index' else slug}"
    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title} — {TITLE_SUFFIX}</title>
<meta name="description" content="{desc}">
<link rel="canonical" href="{url}">
<link rel="icon" href="./mark.svg" type="image/svg+xml">
<meta property="og:title" content="{title} — {TITLE_SUFFIX}">
<meta property="og:description" content="{desc}">
<meta property="og:type" content="website">
<meta property="og:url" content="{url}">
<meta property="og:site_name" content="THENAR">
<meta property="og:image" content="{SITE}/og.png">
<meta property="og:image:width" content="1200">
<meta property="og:image:height" content="630">
<meta property="og:image:alt" content="THENAR, contact data for physical AI">
<meta name="twitter:card" content="summary_large_image">
<meta name="twitter:image" content="{SITE}/og.png">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Manrope:wght@400;500;600;700&display=swap" rel="stylesheet">
<link rel="stylesheet" href="./site.css">
{extra_css}</head>
<body>
<a class="skip" href="#main">Skip to content</a>
'''


def page(slug, title, desc, body, extra_css="", active=None):
    html = (head(title, desc, slug, extra_css)
            + header(active if active is not None else slug) + "\n"
            + '<main id="main">\n' + body + '\n</main>\n'
            + footer() + "\n</body>\n</html>\n")
    with open(os.path.join(OUT, slug + ".html"), "w") as f:
        f.write(html)
    return len(html)


def patch_index():
    """index.html is hand built; only its chrome is owned by this file."""
    p = os.path.join(OUT, "index.html")
    s = open(p).read()
    # Test for the MARKERS, not for a diff: this patch is idempotent, so a
    # second run legitimately produces identical text and a diff check would
    # report that correct outcome as a failure.
    if "<!-- NAV:START -->" not in s or "<!-- FOOT:START -->" not in s:
        print("  index.html: markers missing, chrome NOT patched")
        return False
    s = re.sub(r"<!-- NAV:START -->.*?<!-- NAV:END -->", header("home"), s, flags=re.S)
    s = re.sub(r"<!-- FOOT:START -->.*?<!-- FOOT:END -->", footer(), s, flags=re.S)
    open(p, "w").write(s)
    print("  index.html: chrome patched")
    return True


def faq_body():
    """FAQ moves to its own page; the landing page keeps a short version."""
    import re as _re
    src = open(os.path.join(OUT, "index.html")).read()
    m = _re.search(r'<div class="qalist">.*?</div>', src, _re.S)
    qa = m.group(0) if m else "<p>No questions yet.</p>"
    return ('  <section class="fwrap">\n'
            '    <p class="feyebrow">Objections</p>\n'
            '    <h2 class="fh2">The questions<br>we get asked</h2>\n'
            '    <p class="flede" style="margin-top:var(--s-300)">Including the '
            'hostile ones. If an answer here is weaker than you expected, that is '
            'deliberate.</p>\n' + qa + '\n  </section>\n')


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    import pages

    print("build_site: chrome is defined once here")
    patch_index()
    bodies = {
        "products": pages.PRODUCTS,
        "protocol": pages.PROTOCOL,
        "market": pages.MARKET,
        "company": pages.COMPANY,
        "faq": faq_body(),
    }
    for slug, label, title, desc in NAV:
        n = page(slug, title, desc, bodies[slug])
        print(f"  {slug}.html  {n/1024:.1f} kB")

#!/usr/bin/env python3
"""Build an offline HTML reader with embedded figures and native MathML."""
from __future__ import annotations

import argparse
import base64
import html
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def render(source):
    result = subprocess.run(["pandoc", str(source), "--from=markdown", "--to=html5", "--mathml"],
                            check=True, capture_output=True, text=True)
    body = result.stdout
    prefix = (source.parent.name if source.name == "manual.md" else source.stem.lower()) + "-"
    body = re.sub(r'id="([^"]+)"', lambda m: 'id="' + prefix + m.group(1) + '"', body)
    body = re.sub(r'href="#([^"]+)"', lambda m: 'href="#' + prefix + m.group(1) + '"', body)

    def embed(match):
        image_path = (source.parent / match.group(1)).resolve()
        data = base64.b64encode(image_path.read_bytes()).decode("ascii")
        return 'src="data:image/png;base64,' + data + '"'

    body = re.sub(r'src="([^"\n]+\.png)"', embed, body)
    body = re.sub(r'href="(?:\.\./)?README\.md"', 'href="#start"', body)
    body = re.sub(r'href="lab(\d\d)/manual\.md"', r'href="#lab\1"', body)
    body = re.sub(r'href="(?:\.\./)?ROS_TOOLKIT\.md"', 'href="#commands"', body)
    return body


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "build/ARC_Laboratories.html")
    args = parser.parse_args()
    nav = '<a href="#start">Start here</a> <a href="#commands">ROS command guide</a> ' + " ".join(
        f'<a href="#lab{n:02d}">Lab {n}</a>' for n in range(1,10))
    sections = [('start', ROOT / "docs/labs/README.md")]
    sections += [('commands', ROOT / "docs/labs/ROS_TOOLKIT.md")]
    sections += [(f"lab{n:02d}", ROOT / f"docs/labs/lab{n:02d}/manual.md") for n in range(1,10)]
    def section_body(name, source):
        if name in ('lab01', 'commands'):
            original = (ROOT / 'docs/reader' / (name + '.html')).read_text()
            if name == 'lab01':
                note = '<aside style="background:#edf7f8;padding:16px;border-left:4px solid #087e94"><strong>Course numbering:</strong> For this nine-lab sequence, the original forward references to driving/sensors in Lab 3 now refer to Lab 2. Nav2 navigation previously called Lab 6 is now Lab 5.</aside>'
                return note + original
            return original
        return render(source)
    body = "\n".join(f'<section id="{name}">{section_body(name, source)}</section>' for name, source in sections)
    document = '''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARC Laboratories: Build and understand robot software</title>
<style>
:root { color-scheme: light; --ink:#213247; --muted:#53657a; --accent:#087e94; }
* { box-sizing:border-box; } html { scroll-behavior:smooth; }
body { margin:0; color:var(--ink); background:#f1f5f7; font:17px/1.65 system-ui,sans-serif; }
header { background:#143246; color:white; padding:40px max(24px,calc((100vw - 940px)/2)); }
header p { margin:0; color:#c4e2e8; } header h1 { font-size:36px; line-height:1.15; margin:12px 0; }
nav { position:sticky; top:0; background:#fff; border-bottom:1px solid #d6e1e5; padding:12px 20px; display:flex; flex-wrap:wrap; gap:8px 20px; justify-content:center; z-index:5; font-size:14px; }
a { color:#006f85; text-underline-offset:3px; } nav a { text-decoration:none; font-weight:650; }
main { max-width:1020px; margin:auto; padding:24px; }
section { background:white; padding:40px 48px; margin-bottom:32px; border-top:5px solid var(--accent); scroll-margin-top:100px; }
h1 { font-size:32px; line-height:1.2; letter-spacing:-.5px; } h2 { margin-top:38px; font-size:23px; line-height:1.3; }
p, li { max-width:80ch; } pre { background:#f0f4f6; border-left:3px solid #71a8b3; padding:18px; overflow:auto; font-size:13px; line-height:1.55; }
code { font-family:ui-monospace,Consolas,monospace; font-size:.87em; } pre code { font-size:inherit; }
figure { margin:28px 0; } img { max-width:100%; height:auto; } figcaption { color:var(--muted); font-size:14px; line-height:1.5; margin-top:10px; }
table { border-collapse:collapse; width:100%; font-size:14px; margin:24px 0; } th,td { padding:10px 12px; text-align:left; border-bottom:1px solid #d6e1e5; vertical-align:top; } th { background:#eef6f7; }
math[display="block"] { overflow:auto; padding:12px; }
@media(max-width:700px) { main { padding:10px; } section { padding:22px 18px; } h1 { font-size:27px; } header h1 { font-size:30px; } nav { position:static; } }
@media print { body { background:white; font-size:11pt; } nav { display:none; } main { padding:0; max-width:none; } section { padding:10mm 0; break-before:page; } pre { white-space:pre-wrap; } figure,table { break-inside:avoid; } header { padding:15mm; } }
</style></head><body><header><p>ROS 2 Jazzy · Gazebo Harmonic · ARC VM 2026.1</p>
<h1>Build and understand robot software</h1><p>Nine connected laboratories. Coursework in week 6 covers Labs 1–4.</p></header>
<nav aria-label="Laboratories">''' + nav + '</nav><main>' + body + '</main></body></html>'
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(document)
    print(args.out)


if __name__ == "__main__":
    main()

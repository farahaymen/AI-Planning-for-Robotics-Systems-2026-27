#!/usr/bin/env python3
"""Refresh explicitly marked full-source listings from the workshop reference."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PATTERN = re.compile(r'(<!-- reference: ([^\n]+) -->\n```\w+\n)(.*?)(\n```)', re.S)


def main():
    count = 0
    for manual in sorted((ROOT / 'docs/labs').glob('lab*/manual.md')):
        def replace(match):
            nonlocal count
            path = (ROOT / match[2]).resolve()
            path.relative_to(ROOT / 'teaching/building')
            count += 1
            return match[1] + path.read_text().rstrip() + match[4]
        manual.write_text(PATTERN.sub(replace, manual.read_text()))
    print(f'Refreshed {count} workshop source listings')


if __name__ == '__main__':
    main()

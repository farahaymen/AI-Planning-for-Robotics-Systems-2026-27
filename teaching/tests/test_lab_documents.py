from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def test_all_nine_labs_have_resolved_diagrams_and_plots():
    for n in range(1, 10):
        source = ROOT / f"docs/labs/lab{n:02d}/manual.md"
        text = source.read_text()
        images = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", text)
        assert len(images) >= 2
        assert all((source.parent / image).is_file() for image in images)


def test_all_lab_shell_blocks_parse_without_execution():
    sources = list((ROOT / "docs/labs").glob("lab*/manual.md"))
    sources.append(ROOT / "docs/labs/README.md")
    sources.append(ROOT / "docs/labs/ROS_TOOLKIT.md")
    for source in sources:
        for block in re.findall(r"```bash\n(.*?)```", source.read_text(), re.S):
            result = subprocess.run(["bash", "-n"], input=block, capture_output=True, text=True)
            assert result.returncode == 0, (source, result.stderr)

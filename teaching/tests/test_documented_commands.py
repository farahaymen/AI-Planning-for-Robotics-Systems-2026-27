"""Keep student commands tied to source interfaces without importing ROS.

This is a source contract check, not a replacement for launch tests in the VM.
The executable numerical snippets are run separately from shell/ROS commands.
"""
import ast
from pathlib import Path
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SOURCES = [ROOT / "docs/labs/README.md", ROOT / "docs/labs/ROS_TOOLKIT.md",
           *sorted((ROOT / "docs/labs").glob("lab*/manual.md"))]


def package_root(package):
    if package == 'robot_workshop':
        return ROOT / 'teaching/building/robot_workshop'
    return ROOT / package


def calls(path, name):
    tree = ast.parse(path.read_text())
    return [node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and ((isinstance(node.func, ast.Name) and node.func.id == name)
                 or (isinstance(node.func, ast.Attribute) and node.func.attr == name))]


def literal(node):
    try:
        return ast.literal_eval(node)
    except (ValueError, TypeError):
        return None


def commands(pattern):
    for source in SOURCES:
        text = source.read_text().replace("\\\n", " ")
        for match in re.finditer(pattern, text):
            yield source, shlex.split(match.group())


def test_documented_ros_launch_files_and_arguments():
    count = 0
    for source, words in commands(r"ros2 launch (?:arc_\w+|robot_workshop) [\w.]+[^`\n]*"):
        package, filename = words[2:4]
        launch = package_root(package) / "launch" / filename
        assert launch.is_file(), (source, launch)
        declared = {literal(call.args[0]): call for call in calls(launch, "DeclareLaunchArgument")}
        supplied = {word.split(":=", 1)[0] for word in words[4:] if ":=" in word}
        required = {name for name, call in declared.items()
                    if not any(kw.arg == "default_value" for kw in call.keywords)}
        assert supplied <= declared.keys(), (source, supplied - declared.keys())
        assert required <= supplied, (source, required - supplied)
        count += 1
    assert count >= 4, "No longer scanning the expected lab launch commands"


def test_documented_ros_executables_and_explicit_parameters():
    count = 0
    for source, words in commands(r"ros2 run (?:arc_\w+|robot_workshop) \w+[^`\n]*"):
        package, executable = words[2:4]
        setup = package_root(package) / "setup.py"
        setup_call, = calls(setup, "setup")
        entry_points = literal(next(kw.value for kw in setup_call.keywords if kw.arg == "entry_points"))
        entries = dict(entry.split("=", 1) for entry in entry_points["console_scripts"])
        entries = {key.strip(): value.strip() for key, value in entries.items()}
        assert executable in entries, (source, executable)
        module, function = entries[executable].split(":")
        path = package_root(package) / (module.replace(".", "/") + ".py")
        tree = ast.parse(path.read_text())
        assert any(isinstance(node, ast.FunctionDef) and node.name == function for node in tree.body)
        declared = {literal(call.args[0]) for call in calls(path, "declare_parameter")}
        declared.add("use_sim_time")  # Built into ROS nodes.
        supplied = {words[i + 1].split(":=", 1)[0] for i, word in enumerate(words) if word == "-p"}
        assert supplied <= declared, (source, supplied - declared)
        count += 1
    assert count >= 4


def test_documented_python_cli_options_and_choices():
    count = 0
    pattern = r"python3 (?:-m teaching\.[\w]+|starters/[\w/]+\.py)[^`\n]*"
    for source, words in commands(pattern):
        module_command = words[1] == "-m"
        path = ROOT / (words[2].replace(".", "/") + ".py" if module_command else words[1])
        args = words[3:] if module_command else words[2:]
        assert path.is_file(), (source, path)
        options = {}
        required = set()
        for call in calls(path, "add_argument"):
            names = [literal(arg) for arg in call.args]
            kwargs = {kw.arg: literal(kw.value) for kw in call.keywords}
            for name in names:
                if isinstance(name, str) and name.startswith("--"):
                    options[name] = kwargs
                    if kwargs.get("required"):
                        required.add(name)
        supplied = {arg.split("=", 1)[0] for arg in args if arg.startswith("--")}
        assert supplied <= options.keys(), (source, supplied - options.keys())
        assert required <= supplied, (source, required - supplied)
        for i, arg in enumerate(args):
            choices = options.get(arg, {}).get("choices")
            if choices is not None:
                assert args[i + 1] in [str(choice) for choice in choices], (source, arg)
        count += 1
    assert count >= 15


def test_documented_numerical_entrypoints_import_and_show_help():
    modules = sorted(set(words[2] for _, words in commands(
        r"python3 -m teaching\.[\w]+[^`\n]*")))
    assert modules, "No executable teaching experiments found"
    for module in modules:
        result = subprocess.run([sys.executable, '-m', module, '--help'],
                                cwd=ROOT, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, (module, result.stderr)
        assert 'usage:' in result.stdout


def test_documented_source_paths_exist():
    for source in SOURCES:
        for path in re.findall(r"`((?:arc_\w+|starters|teaching|scripts)/[^`\s]+)`", source.read_text()):
            assert (ROOT / path).exists(), (source, path)

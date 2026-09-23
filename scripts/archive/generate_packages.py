#!/usr/bin/env python3
"""
Generate the ROS 2 package scaffolding for the course workspace.

The course content (Python modules, configs, URDF, manuals) already exists. What
this adds is the packaging around it: manifests, build files, entry points,
launch files and worlds, so that `colcon build` produces a workspace students can
actually run.

It is a generator rather than forty hand-written files because the manifests are
highly repetitive and drift apart when maintained by hand. Run it again after
changing a dependency and every package stays consistent.

    python3 scripts/generate_packages.py [--root .]
"""

from __future__ import annotations

import argparse
import stat
from pathlib import Path

MAINTAINER = "Farah Aymen"
EMAIL = "farah.aymen@bue.edu.eg"
LICENSE = "Apache-2.0"
VERSION = "2026.1.0"

written: list[str] = []


def write(root: Path, rel: str, content: str, executable: bool = False) -> None:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.lstrip("\n"))
    if executable:
        path.chmod(path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
    written.append(rel)


# ---------------------------------------------------------------------------
# Manifests
# ---------------------------------------------------------------------------

def package_xml(name: str, description: str, build_type: str, deps: list[str]) -> str:
    dep_lines = "\n".join(f"  <exec_depend>{d}</exec_depend>" for d in deps)
    test_deps = ("  <test_depend>ament_copyright</test_depend>\n"
                 "  <test_depend>ament_flake8</test_depend>\n"
                 "  <test_depend>ament_pep257</test_depend>\n"
                 "  <test_depend>python3-pytest</test_depend>")
    if build_type == "ament_cmake":
        test_deps = "  <test_depend>ament_lint_auto</test_depend>"
    return f"""
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>{VERSION}</version>
  <description>{description}</description>
  <maintainer email="{EMAIL}">{MAINTAINER}</maintainer>
  <license>{LICENSE}</license>

  <buildtool_depend>{'ament_cmake' if build_type == 'ament_cmake' else 'ament_python'}</buildtool_depend>
{dep_lines}

{test_deps}

  <export>
    <build_type>{build_type}</build_type>
  </export>
</package>
"""


def cmake_lists(name: str, install_dirs: list[str], cpp_nodes: dict[str, list[str]] | None = None) -> str:
    installs = "\n".join(
        f"install(DIRECTORY {d} DESTINATION share/${{PROJECT_NAME}})" for d in install_dirs)
    cpp = ""
    if cpp_nodes:
        finds = "\n".join(f"find_package({p} REQUIRED)" for p in
                          sorted({d for deps in cpp_nodes.values() for d in deps}))
        targets = []
        for exe, deps in cpp_nodes.items():
            targets.append(
                f"add_executable({exe} src/{exe}.cpp)\n"
                f"ament_target_dependencies({exe} {' '.join(deps)})\n"
                f"install(TARGETS {exe} DESTINATION lib/${{PROJECT_NAME}})")
        cpp = f"\nfind_package(ament_cmake REQUIRED)\n{finds}\n\n" + "\n\n".join(targets) + "\n"
    return f"""
cmake_minimum_required(VERSION 3.8)
project({name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
{cpp}
{installs}

ament_package()
"""


def setup_py(name: str, description: str, entry_points: dict[str, str],
             data_dirs: list[str] | None = None) -> str:
    eps = ",\n".join(f"            '{k} = {v}'" for k, v in entry_points.items())
    extra = ""
    for d in (data_dirs or []):
        extra += (f"        (os.path.join('share', package_name, '{d}'),\n"
                  f"         glob('{d}/*')),\n")
    return f"""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = '{name}'

setup(
    name=package_name,
    version='{VERSION}',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
{extra}    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{MAINTAINER}',
    maintainer_email='{EMAIL}',
    description='{description}',
    license='{LICENSE}',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
{eps},
        ],
    }},
)
"""


def setup_cfg(name: str) -> str:
    return f"""
[develop]
script_dir=$base/lib/{name}
[install]
install_scripts=$base/lib/{name}
"""


def python_package(root: Path, name: str, description: str, deps: list[str],
                   entry_points: dict[str, str], data_dirs: list[str] | None = None) -> None:
    write(root, f"{name}/package.xml", package_xml(name, description, "ament_python", deps))
    write(root, f"{name}/setup.py", setup_py(name, description, entry_points, data_dirs))
    write(root, f"{name}/setup.cfg", setup_cfg(name))
    write(root, f"{name}/resource/{name}", "")
    init = root / name / name / "__init__.py"
    if not init.exists():
        write(root, f"{name}/{name}/__init__.py", "")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    args = parser.parse_args()
    root = Path(args.root).resolve()
    import package_content
    package_content.build(root, write, package_xml, cmake_lists, python_package)
    print(f"generated {len(written)} files under {root}")
    for w in sorted(written):
        print(f"  {w}")

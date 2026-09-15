#!/usr/bin/env bash
# Build every lab manual and checklist from Markdown.
#
# Two outputs per document, deliberately:
#
#   DOCX  the student-facing deliverable. Equations become native Word OMML,
#         which Word renders and edits correctly.
#   PDF   built through LaTeX, which renders equations correctly everywhere.
#
# The PDF exists because LibreOffice silently DROPS OMML equations when it
# exports a DOCX to PDF. A student opening the Word file in LibreOffice on a lab
# machine may see blank space where the kinematics should be. Until that is
# confirmed fixed on the target machines, distribute the PDF for any manual that
# contains mathematics.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# pandoc resolves image paths against the working directory, not the source
# file, and every manual refers to figures as docs/figures/<name>.png.
cd "$ROOT"

# Regenerate the figures first so a manual can never ship a stale diagram.
if [[ "${SKIP_FIGURES:-0}" != "1" ]]; then
  PYTHONPATH="$ROOT" python3 scripts/make_figures.py
  PYTHONPATH="$ROOT" python3 scripts/make_diagrams.py
fi
REF="$ROOT/docs/templates/arc-reference.docx"
OUT="$ROOT/build"
mkdir -p "$OUT"

build () {
  local src="$1" stem="$2" math="${3:-no}"
  pandoc "$src" --reference-doc="$REF" --toc --toc-depth=3 \
         --highlight-style=tango -o "$OUT/$stem.docx"
  if [[ "$math" == "math" ]]; then
    pandoc "$src" --toc --toc-depth=3 --highlight-style=tango \
           --pdf-engine=xelatex -V geometry:margin=2.5cm -V fontsize=11pt \
           -o "$OUT/$stem.pdf"
  fi
  echo "built $stem"
}

build "$ROOT/docs/proposal.md"                     "ARC_Curriculum_Proposal"
build "$ROOT/docs/golden_vm_build_runbook.md"      "ARC_Golden_VM_Build_Runbook"
build "$ROOT/docs/00_design_baseline.md"           "ARC_Design_Baseline_v0.1"
build "$ROOT/docs/labs/lab01/manual.md"            "Lab01_ROS2_as_a_Robotics_Software_System"
build "$ROOT/docs/labs/lab02/manual.md"            "Lab02_Kinematics_TF2_URDF_and_ros2_control" math
build "$ROOT/docs/labs/lab03/manual.md"            "Lab03_Gazebo_Simulation_Sensors_and_Robot_Data"
build "$ROOT/docs/labs/lab04/manual.md"            "Lab04_Occupancy_Grid_Mapping" math
build "$ROOT/docs/labs/lab05/manual.md"            "Lab05_Localisation_and_SLAM" math
build "$ROOT/docs/projects/project1_specification.md" "Project1_Autonomous_Mapping_and_Navigation"
build "$ROOT/docs/projects/project2_specification.md" "Project2_Autonomous_Robotics_Grand_Challenge"
build "$ROOT/docs/labs/lab06/manual.md"            "Lab06_Autonomous_Navigation_with_Nav2" math
build "$ROOT/docs/labs/lab07/manual.md"            "Lab07_Robust_Autonomy_Behaviour_Trees_and_Safety"
build "$ROOT/docs/labs/lab08/manual.md"            "Lab08_Building_a_Reinforcement_Learning_Environment"
build "$ROOT/docs/labs/lab09/manual.md"            "Lab09_Deep_Reinforcement_Learning_for_Navigation"
build "$ROOT/docs/labs/lab10/manual.md"            "Lab10_Hybrid_Autonomy_Generalisation_and_Sim_to_Real"
for n in 01 02 03 04 05 06 07 08 09 10; do
  build "$ROOT/docs/labs/lab$n/validation_checklist.md" "Lab${n}_Validation_Checklist"
done

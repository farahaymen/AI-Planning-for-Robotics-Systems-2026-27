# Teaching the nine-lab sequence

Keep Lab 1 unchanged. Lab 2 combines model/frame inspection with a complete movement-and-feedback experiment, resolving the previous static-display versus simulation confusion. Lab 3 explicitly separates supplied-pose occupancy mapping from SLAM and saves the navigation map. Lab 4's two-hour core is search and tracking; other algorithms are guided extensions. Coursework is submitted in week 6 using only Labs 1–4.

Nav2 is Lab 5 because it assembles the conventional stack and supplies a baseline for learned methods. Labs 6–9 follow the conceptual order tabular RL, deep value learning, policy gradients/PPO, and ROS deployment/evaluation. The supplied older notebooks informed this sequence: old Lab 5 maps to Lab 6, old Lab 6 maps to Lab 7, and old Lab 7's policy methods map to Lab 8. PSO and multi-agent learning remain optional later projects.

The shared navigation observation is partially observable. Teach reward design, terminal/truncation distinctions, training/validation/test separation, model metadata and multiple seeds alongside algorithm updates. A short training run validates execution only. No minimum success rate is promised for the provided default budgets.

Lab 1 is a message experiment. Labs 2, 3, 5 and 9 use Gazebo; Labs 3, 4 and 6–9 also provide numerical robot visualisations. These visualisations are labelled so students do not mistake surrogate output for Gazebo evidence.

Before classroom release, run the VM acceptance checklist in verification/README.md. ROS/Gazebo could not be executed in the authoring environment. Do not present the offline checks as proof of physical simulation integration.

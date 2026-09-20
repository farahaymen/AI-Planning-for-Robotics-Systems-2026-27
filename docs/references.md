# Further reading

Every reference here was checked by opening it in September 2026. Each one has a
sentence saying what you get from it and which part of the lab it connects to.

Three kinds of source appear under each lab:

- **Papers.** Recent work, mostly 2023 onward. arXiv links are free to read.
  Anything paywalled is marked.
- **Industry.** How the topic is used in a product, written by the people who
  built it.
- **Documentation.** The reference to keep open while you work.

You are not expected to read all of this. Pick the paper for the part of the lab
you found hardest, and keep the documentation open during the session.

---

## Lab 1. ROS 2 middleware

### Papers

**Robot Operating System 2: Design, Architecture, and Uses In The Wild**
Macenski, Foote, Gerkey, Lalancette, Woodall. *Science Robotics* 7(66), 2022.
<https://arxiv.org/abs/2211.07752>
Written by the ROS 2 core maintainers, it explains why the node and topic layer
was rebuilt on DDS and what that buys you. Paywalled at Science Robotics, free
on arXiv.

**Dependency Chain Analysis of ROS 2 DDS QoS Policies**
Lee, Kang, Park. arXiv, 2025. <https://arxiv.org/abs/2509.03381>
Maps how sixteen DDS QoS policies depend on each other and builds a tool that
flags conflicting profiles before deployment, which is the theory behind the QoS
incompatibility exercise.

**Performance Evaluation of ROS2-DDS middleware implementations**
Paul, Le Phuoc, Hauswirth. arXiv, 2024. <https://arxiv.org/abs/2412.07485>
Benchmarks different DDS vendors under the same ROS 2 workload, which shows that
the middleware underneath rclpy changes measured latency and throughput.

### Industry

**How to Use ROS 2 Lifecycle Nodes**
Millán, Foxglove, 2023.
<https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes>
Walks through the four lifecycle states with a working Python node and shows how
Nav2's lifecycle manager brings a real stack up in a deterministic order.

**Zenoh Experimental Support Lands in ROS 2**
ZettaScale and Open Source Robotics Foundation, 2024.
<https://www.zettascale.tech/news/zenoh-experimental-support-lands-in-ros-2/>
Explains why rmw_zenoh was added alongside DDS, which makes concrete the idea
that the middleware is a swappable layer rather than a fixed part of ROS 2.

### Documentation

- **QoS policies.** <https://design.ros2.org/articles/qos.html>
  History, depth, reliability and durability, plus the predefined sensor data
  and parameter profiles.
- **Managed nodes.** <https://design.ros2.org/articles/node_lifecycle.html>
  The four primary states and seven transitions, so you can predict what
  `on_configure` and `on_activate` are allowed to do.
- **Parameter API design.** <https://design.ros2.org/articles/ros_parameters.html>
  The nine parameter types and the get, set and list services, including the
  atomic set rule that makes runtime reconfiguration safe.

---

## Lab 2. Robot description and differential drive kinematics

### Papers

**Understanding URDF: A Dataset and Analysis**
Tola, Corke. 2023. <https://arxiv.org/pdf/2308.00514>
Analyses 322 real URDF files to show which conventions and which mistakes
actually occur, worth reading before you write your own description.

**Differential drive kinematics and odometry for a mobile robot using TwinCAT**
Ferreira, Moreira, Lopes. *Electronic Research Archive* 31(4), 2023. Open access.
<https://www.aimspress.com/article/doi/10.3934/era.2023092?viewType=HTML>
Derives the forward and inverse kinematics and encoder odometry end to end, then
reports the trajectory errors that result on real hardware.

**Tightly-Coupled LiDAR-IMU-Wheel Odometry with Online Calibration**
Okawara, Koide, Oishi, Yokozuka, Banno, Uno, Yoshida. 2024.
<https://arxiv.org/html/2404.02515v1>
Names the specific error sources that break wheel odometry, longitudinal and
lateral slip, wrong wheel separation, changing terrain, and estimates them online
rather than trusting a fixed calibration.

### Industry

**ros2_control Concepts and Simulation**
Newans, Articulated Robotics.
<https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts/>
Explains hardware interfaces, controllers and resource claiming in plain terms,
then configures diff_drive_controller from YAML.

**ROS2 Control with the JetBot: Building a ros2_control System**
Hart, Mike Likes Robots, 2024.
<https://mikelikesrobots.github.io/blog/jetbot-motors-pt2/>
Implements a real hardware interface for a physical robot and shows why an open
loop setup with no encoders gives poor odometry.

### Documentation

- **Wheeled mobile robot kinematics.**
  <https://control.ros.org/jazzy/doc/ros2_controllers/doc/mobile_robot_kinematics.html>
  The differential drive equations exactly as diff_drive_controller implements
  them, including the no-slip assumption this lab then breaks.
- **diff_drive_controller.**
  <https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>
  Every parameter you have to set, and what appears on `~/odom` and TF.
- **REP 105, coordinate frames for mobile platforms.**
  <https://reps.openrobotics.org/rep-0105/>
  Defines `base_link`, `odom` and `map`, and states that `odom` is continuous but
  drifts without bound.

---

## Lab 3. Sensors and data validation

### Papers

**How Accurate Can 2D LiDAR Be?**
Biernacki, Ziębiński. *Sensors* 25(4), 2025. Open access.
<https://www.mdpi.com/1424-8220/25/4/1211>
Measures four 2D LiDARs against a certified reference and separates precision
from accuracy, which is the distinction you need when judging whether a scan is
wrong.

**Runtime Verification and Field-based Testing for ROS-based Robotic Systems**
Caldas, Piñera García, Schiopu, Pelliccione, Rodrigues, Berger. 2024.
<https://arxiv.org/html/2404.11498v3>
Twenty guidelines drawn from 1,088 repositories and 55 practitioners on
instrumenting and monitoring a ROS system so faults are visible in the field.

**ROSMonitoring 2.0**
Ghaffari Saadat, Ferrando, Dennis, Fisher. FMAS 2024.
<https://arxiv.org/abs/2411.14367>
Attaches automatic monitors to topics that check message content and ordering,
which is the systematic version of this lab's "publishing but wrong" diagnosis.

### Industry

**MCAP as the ROS 2 Default Bag Format**
Smith, Foxglove, 2022.
<https://foxglove.dev/blog/mcap-as-the-ros2-default-bag-format>
Why rosbag2 moved from SQLite to MCAP, and what write throughput and compression
mean when recording sensor streams.

**How to Replay ros2 bags with changed Quality of Service**
Aderinola, The Construct, 2022.
<https://www.theconstruct.ai/how-to-replay-ros2-bags-with-changed-quality-of-service/>
The override file mechanism for replaying a bag under a different reliability
setting, which is the fix when a bag will not play back into a subscriber.

### Documentation

- **REP 145, conventions for IMU sensor drivers.**
  <https://reps.openrobotics.org/rep-0145/>
  The units, frames and covariance conventions that decide whether an IMU
  message is usable downstream.
- **rosbag2.** <https://github.com/ros2/rosbag2/blob/rolling/README.md>
  Record and play options, including splitting, compression and QoS overrides.
- **Gazebo sensors.** <https://gazebosim.org/docs/harmonic/sensors/>
  The SDF for adding IMU and lidar sensors, and how update rate sets the
  publication rate you then measure.

---

## Lab 4. Occupancy grid mapping and classical path planning

### Papers

**Open-Source, Cost-Aware Kinematically Feasible Planning**
Macenski, Booker, Wallace. 2024. <https://arxiv.org/abs/2401.13078>
Describes the A*, Hybrid-A* and State Lattice planners actually shipped in Nav2,
including a cost-aware A* that uses every cost value in the grid rather than only
avoiding collisions.

**A Comprehensive Study of Recent Path-Planning Techniques in Dynamic Environments**
AbuJabal, Baziyad, Fareh, Brahmi, Rabie, Bettayeb. *Sensors* 24(24), 2024. Open access.
<https://www.mdpi.com/1424-8220/24/24/8089>
Classifies grid based, sampling based and potential field planners in one table,
which shows where BFS, Dijkstra and A* sit in the wider field.

**Grid-Centric Traffic Scenario Perception for Autonomous Driving**
Shi, Jiang, Li, Qian, Wen, Yang, Wang, Yang. 2023.
<https://arxiv.org/abs/2303.01212>
Its appendix sets out the binary Bayes filter and a worked inverse sensor model
for a single LiDAR return, which is the log-odds update you implement here. Free
on arXiv, paywalled at IEEE.

### Industry

**Nav2 Smac Planner Paper Released**
Macenski, Open Navigation, 2024. <https://opennav.org/news/smac-planner-paper/>
The maintainer's own account of why Nav2 needed three A* variants and what cost
awareness changes about path quality near keep-out zones.

**Autonomous Robot Navigation and Nav2**
Millán, Foxglove, 2024.
<https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2>
Explains the 0 to 255 costmap cell values and the global and local costmap split,
connecting your occupancy grid to a production planner's input.

### Documentation

- **Costmap 2D.**
  <https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/>
  The static, obstacle, inflation and voxel layers, and how a raw occupancy grid
  becomes a planning cost field.
- **nav2_smac_planner.**
  <https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md>
  A concrete example of heuristic design and its effect on node expansions.

---

## Lab 5. SLAM and localisation

### Papers

**KISS-ICP: In Defense of Point-to-Point ICP**
Vizzo, Guadagnino, Mersch, Wiesmann, Behley, Stachniss. RA-L 2023.
<https://arxiv.org/abs/2209.15397>
Shows how far plain point-to-point ICP gets you with adaptive correspondence
thresholds and motion compensation, which is scan matching stripped of
unnecessary machinery.

**Efficiently Closing Loops in LiDAR-Based SLAM Using Point Cloud Density Maps**
Gupta, Guadagnino, Mersch, Trekel, Malladi, Stachniss. IJRR 2026.
<https://arxiv.org/html/2501.07399v2>
A complete modern loop closure pipeline, which explains why loop closure is what
stops pose graph drift accumulating.

**A Survey on Global LiDAR Localization**
Yin, Xu, Lu, Chen, Xiong, Shen, Stachniss, Wang. 2024.
<https://arxiv.org/abs/2302.07433>
Surveys how a robot works out where it is on a prior map, giving the wider
context for what AMCL does and where particle filters run out of road. Free on
arXiv, paywalled at Springer.

### Industry

**Autonomous Navigation with LeKiwi and Nav2**
Kamath, Foxglove, 2026.
<https://foxglove.dev/blog/autonomous-navigation-with-lekiwi-and-nav2>
Maps a real robot with slam_toolbox in async mode, then localises it two ways,
slam_toolbox localization on a saved pose graph against AMCL on a static map,
which is exactly the comparison Exercise 5.4 asks for.

**The Robotics Behind Dexory's Automated Inventory Management System**
Dexory, 2024.
<https://www.dexory.com/insights/the-robotics-behind-dexorys-automated-inventory-management-system>
A deployed warehouse robot running mapping, localisation, relocalisation,
planning and navigation as five concurrent processes without floor markers, which
shows what map stability over time is worth commercially.

### Documentation

- **slam_toolbox.** <https://github.com/SteveMacenski/slam_toolbox>
  Pose graph, scan matching, Ceres solver plugins, loop closure, serialisation,
  and the mapping, lifelong and localization modes.
- **AMCL configuration.**
  <https://docs.nav2.org/configuration/packages/configuring-amcl.html>
  Every parameter you will tune, including the laser model type, the odometry
  motion model, particle counts and the alpha noise terms.
- **Navigating while mapping.**
  <https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html>
  Running slam_toolbox alongside Nav2 and saving the result.

---

## Lab 6. The Nav2 navigation stack

### Papers

**From the Desks of ROS Maintainers: A Survey of Modern and Capable Mobile Robotics Algorithms in ROS 2**
Macenski, Moore, Lu, Merzlyakov, Ferguson. *Robotics and Autonomous Systems*, 2023.
<https://arxiv.org/abs/2307.15236>
The best single overview of the whole stack, covering NavFn, the Smac variants,
Theta*, DWB, TEB, MPPI, Regulated Pure Pursuit, the costmap layers and the BT
navigator, written by the people who maintain them. Free on arXiv.

**Open-Source, Cost-Aware Kinematically Feasible Planning**
Macenski, Booker, Wallace. 2024. <https://arxiv.org/html/2401.13078v1>
Why a planner has to respect turning radius and costmap gradient rather than just
find the shortest grid path.

**Lessons Learned from The 2nd BARN Challenge at ICRA 2023**
Xiao, Xu, Warnell, Stone and others. ICRA 2023.
<https://arxiv.org/abs/2308.03205>
A standardised head to head of navigation systems on the same robot and the same
courses, and an honest account of how much on-site tuning the results still
needed. Read it before Experiment 6.4.

### Industry

**Nav2 Jazzy Release**
Macenski, Open Navigation, 2024.
<https://discourse.openrobotics.org/t/nav2-jazzy-release/38321>
What changed in the exact distribution this course uses, including BT.CPP v4,
MPPI optimisations and velocity-scheduled collision monitor polygons.

**Autonomous robot navigation and Nav2: the first steps**
Millán, Foxglove, 2024.
<https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps>
Maps the server architecture onto a running simulation and explains why gradual
costmap gradients beat sharp cost frontiers.

### Documentation

- **Setting up navigation plugins.**
  <https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html>
  Tables matching each planner and controller to differential, omnidirectional,
  Ackermann and legged platforms. This is the reference for Exercise 6.2.
- **Tuning guide.** <https://docs.nav2.org/tuning/index.html>
  Inflation as a potential field rather than a safety buffer, and the behavioural
  differences between the planners and between the controllers.
- **Navigation concepts.** <https://docs.nav2.org/concepts/index.html>
  The vocabulary sheet: behaviour trees, planners, controllers, costmaps,
  smoothers, recoveries, lifecycle nodes.

---

## Lab 7. Robustness and recovery

### Papers

**A Survey of Behavior Trees in Robotics and AI**
Iovino, Scukins, Styrud, Ögren, Smith. *Robotics and Autonomous Systems* 154, 2022.
Open access. <https://www.sciencedirect.com/science/article/pii/S0921889022000513>
Classifies 166 papers and explains why behaviour trees scale where finite state
machines do not, which is the theory behind Nav2's recovery subtree.

**Runtime Verification and Field-Based Testing for ROS-Based Robotic Systems**
Caldas and others. *IEEE TSE* 50(10), 2024. <https://arxiv.org/abs/2404.11498>
Twenty guidelines for architecting a ROS system so faults can be diagnosed while
it is running, not only in simulation. Free on arXiv, paywalled at IEEE.

**Safe-ROS: An Architecture for Autonomous Robots in Safety-Critical Domains**
Benjumea, Farrell, Dennis. FMAS 2025. <https://arxiv.org/abs/2511.14433>
Separates the control system from an independent safety function that can stop
the robot but never drive it, which is exactly the veto-not-command property of
the emergency brake you wrote in Lab 3.

### Industry

**State Machines vs Behavior Trees**
Ahmad, Polymath Robotics, 2023.
<https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees>
A first-hand account of a company outgrowing a state machine once e-stop and
pause states multiplied.

**SOVD diagnostics and OTA updates in Foxglove**
Burda, Foxglove, 2026.
<https://foxglove.dev/blog/sovd-diagnostics-and-ota-updates-in-foxglove-via-ros2-medkit>
Follows one warehouse robot from an aborted navigation action through automated
watchers, a captured rosbag, diagnosis of a faulty LiDAR sector and an over-the-air
fix.

### Documentation

- **Collision Monitor.**
  <https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html>
  The safety node that works on polygons and raw sensor data while bypassing the
  costmap and the trajectory planner.
- **Detailed behavior tree walkthrough.**
  <https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html>
  Steps through the replanning and recovery tree, including the round robin of
  clearing costmaps, spinning, waiting and backing up.
- **Behavior tree XML nodes.**
  <https://docs.nav2.org/configuration/packages/configuring-bt-xml.html>
  The full catalogue, including `IsStuck` and the cancel actions.

---

## Lab 8. Building a reinforcement learning environment

### Papers

**Gymnasium: A Standard Interface for Reinforcement Learning Environments**
Towers, Kwiatkowski, Terry, Balis and others, Farama Foundation. NeurIPS Datasets
and Benchmarks 2025. <https://arxiv.org/abs/2407.17032>
Why `reset`, `step`, `observation_space` and `action_space` are shaped the way
they are, which is the contract you implement in this lab.

**Arena 4.0: A ROS2 Development and Benchmarking Platform for Human-centric Navigation**
Shcherbyna, Kästner and others. 2024. <https://arxiv.org/abs/2409.12471>
A working ROS 2 platform where the training environment and the real navigation
stack share one interface, which is the observation contract point of this lab.

**Transformable Gaussian Reward Function for Socially-Aware Navigation**
Kim, Kang, Yang, Kim, Jargalbaatar, Kim. 2024. <https://arxiv.org/abs/2402.14569>
A worked example of navigation reward design that replaces a pile of hand-tuned
terms with three parameters.

### Industry

**Fast-Track Robot Learning in Simulation Using NVIDIA Isaac Lab**
Vishwanath, Shaltiel, NVIDIA, 2024.
<https://developer.nvidia.com/blog/fast-track-robot-learning-in-simulation-using-nvidia-isaac-lab/>
Why training happens in simulation at all, and how throughput rather than
fidelity drives the design of a training environment.

**Revolutionizing warehouse automation with scientific simulation**
Akyildiz, Liu, Amazon Science, 2025.
<https://www.amazon.science/blog/revolutionizing-warehouse-automation-with-scientific-simulation>
The speed argument in a product setting: hundreds of configurations explored in
simulation in the time a few physical setups can be tested.

### Documentation

- **Gymnasium Env API.** <https://gymnasium.farama.org/api/env/>
- **Gymnasium spaces.** <https://gymnasium.farama.org/api/spaces/>
  What you need when deciding how to encode a laser scan plus a goal vector.
- **Stable-Baselines3 custom environments.**
  <https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html>
  `check_env()` catches contract mistakes before training wastes an afternoon.

---

## Lab 9. Training a navigation policy

### Papers

**Revisiting Sparse Rewards for Goal-Reaching Reinforcement Learning**
Vasan, Wang, Shahriar, Bergstra, Jagersand, Mahmood. 2024.
<https://arxiv.org/html/2407.00324v2>
Evidence that a minimum-time sparse reward can beat a shaped one on final
performance, which is the honest counterweight to reward shaping.

**Bridging RL Theory and Practice with the Effective Horizon**
Laidlaw, Russell, Dragan. NeurIPS 2023. <https://arxiv.org/abs/2304.09853>
A usable notion of how far ahead a policy actually has to look, connecting the
discount factor to whether learning will succeed at all.

**Principles and Guidelines for Evaluating Social Robot Navigation Algorithms**
Francis, Pérez-D'Arpino, Li, Xia, Alahi and others. ACM THRI, 2023.
<https://arxiv.org/abs/2306.16740>
A metrics and scenario checklist for evaluating a navigation policy fairly, which
is what the held-out evaluation needs. Free on arXiv, paywalled at ACM.

### Industry

**The 37 Implementation Details of Proximal Policy Optimization**
Huang, Dossa, Raffin, Kanervisto, Wang. ICLR Blog Track, 2022.
<https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/>
Written by the maintainers of CleanRL and Stable-Baselines3, it lists the
practical choices that decide whether a PPO run works.

**Rliable: Better Evaluation for Reinforcement Learning**
Raffin, 2021. <https://araffin.github.io/post/rliable/>
Interquartile mean, bootstrap confidence intervals and performance profiles, so
you stop reporting one seed's best score.

### Documentation

- **PPO.** <https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html>
  The parameters you will actually set.
- **RL tips and tricks.**
  <https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html>
  Use a separate test environment, average 5 to 20 episodes, normalise
  observations. These are the three things people get wrong.
- **Callbacks.**
  <https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html>
  `EvalCallback` and `CheckpointCallback` are the monitoring machinery.

---

## Lab 10. The simulation to reality gap

### Papers

**A Survey of Sim-to-Real Methods in RL**
Da, Turnau, Kutralingam, Velasquez, Shakarian, Wei. 2025.
<https://arxiv.org/abs/2502.13187v1>
Organises domain randomisation by which part of the problem is randomised,
observation, transition or reward, which is the structure this lab needs.

**NavRL++: Improving Sim-to-Real Transfer in RL-Based Robot Navigation**
Xu, Jin, Shimada, Carnegie Mellon. 2026. <https://arxiv.org/abs/2605.15559>
Measures separately how sensor noise, perception dropouts, latency and control
response each break a trained navigation policy, and reports transfer between
Isaac Sim and Gazebo.

**Hybrid Motion Planning with Deep Reinforcement Learning for Mobile Robot Navigation**
Kolomeytsev, Golembiovsky. 2025. <https://arxiv.org/html/2512.24651v1>
A clean example of the hybrid pattern: A* produces global checkpoints, a learned
local policy handles dynamic obstacles between them. This is Exercise 10.3.

### Industry

**Starting on the Right Foot with Reinforcement Learning**
Boston Dynamics, 2024.
<https://bostondynamics.com/blog/starting-on-the-right-foot-with-reinforcement-learning/>
Over a million randomised simulations, a 2,000 hours per week robustness fleet,
and reproduced real failures fed back into training. This is what deploying a
learned policy actually costs.

**Closing the Sim-to-Real Gap: Training Spot Quadruped Locomotion**
Omotuyi, Hoeller, Burnham, NVIDIA, 2024.
<https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab/>
Names the specific quantities randomised, mass, friction and applied
disturbances, for a transfer that reached a real robot.

### Documentation

- **Visual domain randomization.**
  <https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/latest/transferring-robot-learning-policies-from-simulation-to-reality/03-bridging-the-gap-simulation-enhancement/01-visual-domain-randomization.html>
  The point people usually miss: the goal is variation, not photorealism.
- **Vectorized environments.**
  <https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html>
  `VecNormalize.save()` and `load()`, the practical reason a policy that trained
  fine silently fails once it is moved to a different stack.

---

## Applications. How planning differs by product

Read this before Project 2. Not all robots have the same planning problem, and
the differences are larger than they look.

### Papers

**End-to-End Framework for Robot Lawnmower Coverage Path Planning**
Shah, Dey, Nishimiya, Honda R&D. 2025. <https://arxiv.org/html/2506.06028v1>
Why coverage is a different problem from point to point navigation, with
boustrophedon decomposition as the baseline and non-mowing travel as the cost
being minimised.

**Where Paths Collide: A Survey of Classic and Learning-Based Multi-Agent Pathfinding**
Wang, Xu, Zhang, Lin, Lu, Wang, Li. 2025. <https://arxiv.org/abs/2505.19219>
Over 200 papers on the warehouse problem, where the hard part is coordinating
hundreds of robots rather than planning one good path.

**Robust Route Planning for Sidewalk Delivery Robots**
Tong, Simoni. 2025. <https://arxiv.org/pdf/2507.12067>
Frames last-mile routing as a robust shortest path problem under pedestrian
induced travel-time uncertainty, which is not how warehouse or coverage planning
is posed.

**Path-Planning Technologies in Autonomous Agricultural Machinery**
Wane, Shamshiri, Guo, Chen, Auat Cheein. *Computation* 14(8), 2026. Open access.
<https://www.mdpi.com/2079-3197/14/8/194>
Grades agricultural planning by readiness level, with GNSS point to point at TRL
9 and over a million units deployed while whole-farm autonomy sits at TRL 3 to 5.

### Industry

**Nav2 complete coverage planning capabilities**
Macenski, Open Navigation with Bonsai Robotics, 2023.
<https://discourse.openrobotics.org/t/nav2-new-complete-coverage-planning-capabilities/34625>
Why a coverage task server is structured differently from a point to point
planner server, and why swaths and turns are separate operations.

**How Amazon robots navigate congestion**
Brown, Amazon Science, 2022.
<https://www.amazon.science/latest-news/how-amazon-robots-navigate-congestion>
The fulfilment floor as a city grid lane graph with cloud-side routing and
continuous replanning.

**Amazon builds first foundation model for multirobot coordination**
Durham, Amazon Science, 2025.
<https://www.amazon.science/blog/amazon-builds-first-foundation-model-for-multirobot-coordination>
The current warehouse direction: predict congestion instead of simulating it.

**Scaling Physical AI for Sidewalk Autonomy**
Zhou, Coco Robotics, 2025.
<https://www.cocodelivery.com/blog/scaling-physical-ai-for-sidewalk-autonomy>
A delivery company combining simulation-trained policies with teleoperator
interventions as a training signal.

**John Deere harvests the seeds of large vehicle autonomy**
Oitzman, The Robot Report, 2025.
<https://www.therobotreport.com/john-deere-harvests-the-seeds-of-large-vehicle-autonomy/>
Agricultural constraints: predefined paths inside user-defined boundaries, turn
automation, and perception failing under canopy where GPS is blocked.

### Documentation

- **Nav2 coverage server.**
  <https://docs.nav2.org/configuration/packages/configuring-coverage-server.html>
  Swath angle, headland width and the boustrophedon, snake and spiral orderings.
  This is the production version of the coverage planner you wrote.
- **Nav2 route server tools.**
  <https://docs.nav2.org/tutorials/docs/route_server_tools.html>
  How lane graphs of nodes and edges are authored for warehouse navigation.
- **RMF core overview.** <https://osrf.github.io/ros2multirobotbook/rmf-core.html>
  The traffic schedule fleets publish itineraries to, and the negotiation that
  resolves conflicts. Reservation-based traffic management in practice.

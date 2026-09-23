# Source and factual review

Checked against ROS 2 Jazzy documentation, ros2_control Jazzy controller interfaces, Nav2 documentation, SLAM Toolbox documentation, Gymnasium time-limit handling, Sutton and Barto (second edition), the DQN/Double DQN and PPO papers, Coulter's Pure Pursuit report, Theta* and frontier exploration papers, and Salimpour et al. (2025) on ROS 2 policy transfer. Each chapter links the relevant sources.

Corrections include: a static RViz display does not drive Gazebo; joint positions and fixed transforms do not prove base motion; supplied-pose mapping differs from SLAM; map images differ from serialised pose graphs; planning differs from tracking; Theta* is not guaranteed continuously optimal; Double DQN uses argmax for action selection; terminal masks differ from external time limits; PPO clipping is not a safety guarantee; a trained checkpoint is not proof of useful navigation; scan proximity is not a physical contact count.

Lab 1 is preserved from the supplied HTML. Archived documents describe the earlier sequence and are clearly excluded from current instruction.

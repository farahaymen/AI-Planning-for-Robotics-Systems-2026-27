# Lab 7: Replace the table with a neural action-value function

The grid table works because there are few states. A robot's laser distances and velocity are continuous. Listing a separate table row for every possible observation is impractical. **DQN**, deep Q-network, uses a neural network to approximate action values from an observation.

**Your result:** implement DQN and Double DQN targets, train a checkpoint, and watch its robot trajectory on layouts excluded from training. A checkpoint that loads correctly is not automatically a useful navigation policy.

## Keep the learning question recognisable

The observation has 29 numbers: 24 laser ranges, distance to the goal, sine and cosine of the relative goal angle, forward velocity and angular velocity. Ranges and distance are scaled to known limits. Sine and cosine avoid a jump between angles just below positive pi and just above negative pi. The sensor origin is 0.10 m ahead of the body, matching the supplied model.

The observation is still incomplete information about the world. It does not contain a full map or the intentions of moving obstacles. A feed-forward policy may fail when memory is needed. That limitation should inform interpretation of the experiment.

DQN selects from a **finite action set**. Our six choices are forward, forward-left, forward-right, turn-left, turn-right and slow reverse. `teaching/deep_q.py` stores their normalised velocity pairs in `ACTIONS`. `scale_action` converts a normalised forward value of 0.6 to 0.30 m/s, and a turn value of 0.5 to 0.90 rad/s. DQN does not directly optimise an unrestricted continuous action in this lab.

## What the network predicts

The network receives one 29-number observation and returns six Q estimates, one per action. The largest output determines the greedy choice. Hidden layers combine input features through learned weights and nonlinear functions. These outputs estimate return; they are not probabilities and need not sum to one.

Training changes weights to reduce a prediction error. A **gradient** describes how changing a weight changes the loss. The optimiser adjusts weights in a direction intended to reduce it. Neural approximation allows generalisation between observations but introduces errors and instability that a small exact table did not have.

![DQN learns from replayed transitions and uses a separate target network. Double DQN separates action selection from evaluation.](../../figures/nine/lab07_flow.png)

## DQN visually: the step-by-step training cycle

The following diagrams are by **Ketan Doshi**, from [Reinforcement Learning Explained Visually (Part 5): Deep Q Networks, step-by-step](https://medium.com/data-science/reinforcement-learning-explained-visually-part-5-deep-q-networks-step-by-step-5a5317197f4b). The images are linked from [the author's own illustrated edition](https://ketanhdoshi.github.io/Reinforcement-Learning-Deep-Q-Network/); they require an internet connection in the otherwise offline HTML reader. The explanation alongside each image refers to **our Lab 7 implementation**, not necessarily the article's exact hyperparameters.

### First, connect the Q-table to the DQN architecture

![A small Q-table maps each state-action pair to an estimated return.](https://ketanhdoshi.github.io/assets/images/RLDQN/Overview-1.png)

Lab 6 used a table indexed by the exact grid cell and an action. It cannot enumerate the continuous observations used here.

![A neural network replaces a table lookup by predicting action values.](https://ketanhdoshi.github.io/assets/images/RLDQN/Overview-3.png)

In Lab 7, one 29-number observation enters the online network and six Q-values come out, one for each discrete robot action.

![The DQN architecture includes replay memory, an online Q-network and a target network.](https://ketanhdoshi.github.io/assets/images/RLDQN/Arch-1.png)

Only the online network receives gradient updates. The target network supplies temporarily stable future-value estimates.

![The high-level DQN workflow combines experience collection, prediction, target computation and learning.](https://ketanhdoshi.github.io/assets/images/RLDQN/Workflow-1.png)

The same phases repeat over environment steps; one gradient update is not the same as an entire episode.

### Part A: initialize the learner

**Step 1: Bootstrap replay memory.** Collect early transitions before the first parameter update. `teaching/deep_q.py` starts training once the buffer has at least 256 samples.

![DQN step 1: bootstrap replay memory.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-1.png)

**Step 2: Initialize and synchronize both networks.** Create online and target networks with matching architecture, then copy the online weights to the target. Only the online network will be optimized.

![DQN step 2: initialize and synchronize both networks.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-2.png)

### Part B: gather training data

**Step 3: Choose an epsilon-greedy action.** Obtain six current Q estimates from the online network; with probability epsilon take a random action, otherwise choose the action index with the largest Q estimate.

![DQN step 3: choose an epsilon-greedy action.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-3.png)

**Step 4: Execute the action.** Apply the chosen velocity pair in the fast navigation environment and observe the next observation, reward, and termination or truncation.

![DQN step 4: execute the action.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-4.png)

**Step 5: Store the transition in replay memory.** Append `(observation, action, reward, next_observation, terminated)` to the bounded replay buffer. In this implementation, a time-limit truncation does not set the terminal-learning mask.

![DQN step 5: store the transition in replay memory.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-5.png)

### Part C: train from a replay minibatch

**Step 6: Sample a shuffled minibatch.** Randomly select 64 stored transitions, mixing older and newer experience. The code updates the network every four environment steps once memory contains at least 256 samples.

![DQN step 6: sample a shuffled minibatch.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-6.png)

**Step 7: Predict Q-values for sampled current observations.** Pass each batch observation through the online network; each row contains six predicted action values.

![DQN step 7: predict q-values for sampled current observations.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-7.png)

**Step 8: Select the prediction for the action actually taken.** Gather the online Q-value at the stored action index. The loss trains this selected prediction, not all six outputs against one target.

![DQN step 8: select the prediction for the action actually taken.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-8.png)

**Step 9: Evaluate each next observation with the target network.** Pass next observations through the target network and obtain candidate future values without tracking gradients.

![DQN step 9: evaluate each next observation with the target network.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-9.png)

**Step 10: Build the bootstrap target.** For ordinary DQN, use the target network's maximum next-action value: `reward + gamma * (1 - terminated) * max(target_next)`. For terminal samples, the future term is zero.

![DQN step 10: build the bootstrap target.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-10.png)

**Step 11: Compare predictions with targets.** Calculate the difference between the selected online prediction and the detached target, one value per sampled transition.

![DQN step 11: compare predictions with targets.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-11.png)

**Step 12: Calculate the batch loss.** The article illustrates mean-squared error. This lab uses PyTorch `smooth_l1_loss` (Huber-style), which is less sensitive to large errors.

![DQN step 12: calculate the batch loss.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-12.png)

### Part D: optimize, repeat and refresh the target network

**Step 13: Backpropagate through the online network.** Clear old gradients, backpropagate the loss, clip gradient norm, and step the Adam optimizer. Do not optimize the target network.

![DQN step 13: backpropagate through the online network.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-13.png)

**Step 14: Repeat the interaction and learning cycle.** Continue collecting transitions and performing later minibatch updates. The target network remains fixed while the online network changes.

![DQN step 14: repeat the interaction and learning cycle.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-14.png)

**Step 15: Periodically synchronize the target weights.** Every 1,000 environment steps in this lab, copy online weights into the target network. The cycle then resumes with refreshed target predictions.

![DQN step 15: periodically synchronize the target weights.](https://ketanhdoshi.github.io/assets/images/RLDQN/DQN-15.png)

### Connect the pictures to the actual Lab 7 loss

The picture sequence illustrates ordinary DQN. In this course, `td_targets` also implements **Double DQN**: select the best next action using the online network, then **evaluate that selected action** using the target network. The action choice and action value must not be confused. The lesson's terminal mask removes the bootstrap term on true termination; an external time limit does not imply a terminal Q target.

In `teaching/deep_q.py`, the replay capacity is **20,000**, minibatches contain **64** transitions, optimization begins at **256** transitions and occurs every **4** environment steps, and weights are copied to the target every **1,000** steps. This is a numerical navigation simulator; **Gazebo deployment belongs to Lab 9**.

## Why there are two networks and a replay buffer

A **replay buffer** stores transitions: observation, action, reward, next observation and terminal flag. Sampling a batch mixes experiences from different moments and reuses data. This reduces immediate temporal correlation in updates, although it does not make all samples independent.

The **online network** is updated by gradient descent. The **target network** is copied from it less frequently and supplies temporarily more stable targets. Our implementation copies every 1,000 environment steps and begins updates after 256 transitions. Read these constants before changing them.

The DQN target is

$$y=r+\gamma(1-d)\max_a Q_{target}(o',a).$$

`o'` is the next observation and `d` is true task termination. The network prediction being trained is $Q_{online}(o,a)$ for the action actually taken. A Huber loss penalises their difference, with less sensitivity to very large errors than squared loss. Target calculation is performed without gradient tracking; otherwise the optimiser can change both sides of the target unintentionally.

## Double DQN: selection and evaluation are different operations

Taking a maximum over noisy value estimates can create overestimation. **Double DQN** selects the action using the online network and evaluates that selected action using the target network:

$$a^*=\operatorname{argmax}_a Q_{online}(o',a),\qquad
y=r+\gamma(1-d)Q_{target}(o',a^*).$$

`argmax` returns an action index. `max` returns a value. If online values are `[5,4]` and target values are `[2,6]`, Double DQN selects action 0 and uses target value 2. Ordinary DQN uses target maximum 6. This is Double DQN, a neural-network adaptation of the double-estimation idea; it is not the same implementation as tabular Double Q-learning.

**You implement** `td_targets` in `starters/lab07/targets_skeleton.py`. Return one target for each row in the batch. Apply the terminal mask after obtaining the chosen next value. The provided driver keeps replay, optimisation and environment interaction visible in one module.

```bash
ARC_DQN_TARGET=starters.lab07.targets_skeleton python3 -m teaching.deep_q --method double --steps 5000 --out results/lab07_student
```

The first 5,000 steps are a development run. They exercise the update but are not a promised training budget for reliable navigation. Compare reference runs under equal budgets:

```bash
python3 -m teaching.deep_q --method dqn --steps 50000 --seed 0 --out results/lab07_dqn
python3 -m teaching.deep_q --method double --steps 50000 --seed 0 --out results/lab07_double
```

## Watch the policy and measure what it does

```bash
python3 -m teaching.rollout --policy dqn --model results/lab07_double/policy.pt --seeds 500 501 502 --out results/lab07_validation
```

Open the generated `replay_500.html` and trajectory plots. `episodes.csv` records outcomes and `summary.json` counts collision-free successes, collisions and timeouts. Use the same layouts to evaluate the gap-following baseline from Lab 4. `policy.pt` is a PyTorch checkpoint; `loss.csv` is optimisation history. A decreasing training loss is not proof that the robot reaches goals.

Training uses geometry seeds 0–99. Validation uses 500–509 to help choose settings. Final test layouts 1000–1009 stay unused until those choices are frozen. A random seed controls a pseudorandom sequence; changing only sensor noise in the same layout would not test generalisation to new geometry.

![A DQN target can differ from a Double DQN target even when both networks see the same next observation.](../../figures/nine/lab07_plot.png)

## Diagnose a learned failure

If the robot spins, reverses indefinitely or stops near obstacles, trace observation, action and reward together. Check action scaling before changing the network. Compare epsilon during collection with greedy evaluation. Confirm that terminal transitions do not bootstrap and external time limits do. Then compare several independent training seeds.

The environment is a fast kinematic simulator. It omits wheel acceleration and contact physics, so success here does not establish Gazebo performance. Lab 9 tests the ROS interface and this transfer gap explicitly. Save model metadata with weights so action order and observation layout remain recoverable.

## Files and reading

| Student code | Reference and integration | Generated files |
|---|---|---|
| `starters/lab07/targets_skeleton.py` | `teaching/deep_q.py`, `arc_rl/nav_core.py` | `policy.pt`, losses, episode returns, evaluation replays |

- Mnih et al., *Human-level control through deep reinforcement learning*, 2015. [Paper](https://www.nature.com/articles/nature14236).
- van Hasselt, Guez and Silver, *Deep Reinforcement Learning with Double Q-learning*, 2016. [Author manuscript](https://arxiv.org/abs/1509.06461).
- [PyTorch DQN tutorial](https://docs.pytorch.org/tutorials/intermediate/reinforcement_q_learning.html) provides another view of replay, targets and gradient updates.

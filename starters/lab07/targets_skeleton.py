def td_targets(rewards, terminated, online_next, target_next, gamma=.99, double=True):
    # Tensor inputs, one output target per batch row.
    # Double DQN selects with online_next and evaluates with target_next.
    raise NotImplementedError("Implement DQN and Double DQN targets")

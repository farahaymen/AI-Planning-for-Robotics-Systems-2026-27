"""Lab 8: REINFORCE on the grid robot before continuous PPO navigation."""
import argparse
from pathlib import Path
import json
import numpy as np
import torch
from teaching.tabular import GridRobot


def discounted_returns(rewards, gamma=.95):
    out=[];value=0.
    for reward in reversed(rewards):
        value=reward+gamma*value;out.append(value)
    return list(reversed(out))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episodes',type=int,default=3000)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists() or a.episodes<1:p.error('Use a new directory and positive episodes')
    a.out.mkdir(parents=True);torch.manual_seed(a.seed);torch.set_num_threads(1);rng=np.random.default_rng(a.seed)
    env=GridRobot();logits=torch.nn.Parameter(torch.zeros(env.n_states,4));optimiser=torch.optim.Adam([logits],lr=.03)
    returns=[]
    # Here the task really ends after 150 decisions. This finite horizon is part
    # of the REINFORCE objective, unlike the external DQN collection cutoff.
    for _ in range(a.episodes):
        state=env.state(env.start);log_probs=[];rewards=[]
        for step in range(150):
            distribution=torch.distributions.Categorical(logits=logits[state])
            action=distribution.sample();log_probs.append(distribution.log_prob(action))
            state,reward,done=env.sample(state,int(action),rng);rewards.append(reward)
            if done:break
        targets=torch.tensor(discounted_returns(rewards),dtype=torch.float32)
        # gamma**t is included for the discounted start-state objective.
        weights=.95**torch.arange(len(rewards))
        loss=-(torch.stack(log_probs)*targets*weights).sum()
        optimiser.zero_grad();loss.backward();optimiser.step();returns.append(sum(rewards))
    np.save(a.out/'policy.npy',logits.detach().argmax(dim=1).numpy())
    np.savetxt(a.out/'returns.csv',returns,delimiter=',',header='episode_return',comments='')
    (a.out/'report.json').write_text(json.dumps(dict(algorithm='REINFORCE',seed=a.seed,episodes=a.episodes,horizon=150),indent=2))
    print('Saved policy and episode returns. Compare independent evaluation, not training return alone.')


# Optional student implementation; the environment value is a module name.
import os as _os
import importlib as _importlib
if _os.environ.get('ARC_RETURN'):
    discounted_returns = _importlib.import_module(_os.environ['ARC_RETURN']).discounted_returns

if __name__=='__main__':main()

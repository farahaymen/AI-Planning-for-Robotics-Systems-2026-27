"""Lab 7: transparent DQN and Double DQN on the same continuous robot world.

Actions are discretised; observations still contain continuous laser ranges.
The saved checkpoint includes its action and observation contracts.
"""
from __future__ import annotations
import argparse
from collections import deque
import json
from pathlib import Path
import random
import numpy as np
import torch
from torch import nn
from arc_rl.nav_core import FastNavEnv, OBS
from teaching.train_policy import training_bank

ACTIONS=np.asarray([[.6,0],[.3,.5],[.3,-.5],[0,.7],[0,-.7],[-.4,0]],dtype=np.float32)


def network():return nn.Sequential(nn.Linear(OBS.size,64),nn.ReLU(),nn.Linear(64,64),nn.ReLU(),nn.Linear(64,len(ACTIONS)))


def bootstrap_values(online_next, target_next, double=True):
    if double:
        selected=online_next.argmax(dim=1,keepdim=True)
        return target_next.gather(1,selected).squeeze(1)
    return target_next.max(dim=1).values


def td_targets(rewards, terminated, online_next, target_next, gamma=.99, double=True):
    return rewards+gamma*(1-terminated)*bootstrap_values(online_next,target_next,double)


def load_policy(path):
    checkpoint=torch.load(path,map_location='cpu',weights_only=True)
    if checkpoint['observation_size'] != OBS.size or not np.array_equal(np.asarray(checkpoint['actions']),ACTIONS):
        raise ValueError('checkpoint contract differs from this code')
    model=network();model.load_state_dict(checkpoint['state_dict']);model.eval()
    def policy(obs):
        with torch.no_grad():return ACTIONS[int(model(torch.as_tensor(obs)).argmax())].copy()
    return policy


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--steps',type=int,default=50000)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--method',choices=['dqn','double'],default='double')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists() or a.steps<1:p.error('Use a new output directory and positive steps')
    a.out.mkdir(parents=True);torch.set_num_threads(1);torch.manual_seed(a.seed);random.seed(a.seed);rng=np.random.default_rng(a.seed)
    online=network();target=network();target.load_state_dict(online.state_dict());target.eval()
    optimiser=torch.optim.Adam(online.parameters(),lr=3e-4);memory=deque(maxlen=20000)
    env=FastNavEnv(scenario_factory=training_bank,randomise_start=False)
    obs,_=env.reset(seed=a.seed);episode_return=0.;records=[];losses=[]
    for step in range(a.steps):
        epsilon=max(.05,1-.95*step/max(1,a.steps*.7))
        with torch.no_grad():action=int(rng.integers(len(ACTIONS))) if rng.random()<epsilon else int(online(torch.as_tensor(obs)).argmax())
        next_obs,reward,terminated,truncated,info=env.step(ACTIONS[action])
        memory.append((obs.copy(),action,reward,next_obs.copy(),float(terminated)))
        obs=next_obs;episode_return+=reward
        if terminated or truncated:
            records.append(dict(step=step+1,episode_return=episode_return,success=info['success'],reason=info['termination_reason']))
            obs,_=env.reset();episode_return=0.
        if len(memory)>=256 and step%4==0:
            s,ac,r,s2,done=map(np.asarray,zip(*random.sample(memory,64)))
            s=torch.as_tensor(s,dtype=torch.float32);s2=torch.as_tensor(s2,dtype=torch.float32)
            with torch.no_grad():
                targets=td_targets(torch.as_tensor(r,dtype=torch.float32),torch.as_tensor(done,dtype=torch.float32),online(s2),target(s2),double=a.method=='double')
            chosen=online(s).gather(1,torch.as_tensor(ac,dtype=torch.int64).unsqueeze(1)).squeeze(1)
            loss=nn.functional.smooth_l1_loss(chosen,targets)
            optimiser.zero_grad();loss.backward();nn.utils.clip_grad_norm_(online.parameters(),10);optimiser.step()
            losses.append((step+1,float(loss.detach())))
        if (step+1)%1000==0:target.load_state_dict(online.state_dict())
    env.close()
    torch.save(dict(state_dict=online.state_dict(),actions=ACTIONS.tolist(),observation_size=OBS.size),a.out/'policy.pt')
    np.savetxt(a.out/'loss.csv',np.asarray(losses).reshape(-1,2),delimiter=',',header='environment_step,huber_loss',comments='')
    (a.out/'episodes.json').write_text(json.dumps(records,indent=2))
    report=dict(method=a.method,seed=a.seed,environment_steps=a.steps,gradient_updates=len(losses),training_layouts='0..99',validation_layouts='500..509',test_layouts='1000..1009',warning='A saved checkpoint is not evidence of successful navigation.')
    (a.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


# Optional student implementation; the environment value is a module name.
import os as _os
import importlib as _importlib
if _os.environ.get('ARC_DQN_TARGET'):
    td_targets = _importlib.import_module(_os.environ['ARC_DQN_TARGET']).td_targets

if __name__=='__main__':main()

"""Lab 6: a small robot learns to navigate a grid, with visible saved rollouts."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from teaching.animation import save_replay

MOVES = ((0,1),(1,0),(0,-1),(-1,0))  # east, north, west, south in (row,column)


class GridRobot:
    def __init__(self, slip=.1):
        if not 0 <= slip <= 1: raise ValueError('slip must be in [0,1]')
        self.grid = np.zeros((6,6),bool)
        self.grid[1:5,3] = True
        self.start, self.goal, self.slip = (0,0), (5,5), slip
        self.n_states = self.grid.size

    def state(self, cell):return int(np.ravel_multi_index(cell,self.grid.shape))
    def cell(self, state):return tuple(map(int,np.unravel_index(state,self.grid.shape)))

    def transitions(self, state, action):
        cell=self.cell(state)
        if cell == self.goal:return [(1.,state,0.,True)]
        outcomes=[]
        for a,prob in ((action,1-self.slip),((action-1)%4,self.slip/2),((action+1)%4,self.slip/2)):
            dr,dc=MOVES[a];nr,nc=cell[0]+dr,cell[1]+dc
            collision=not(0<=nr<6 and 0<=nc<6) or self.grid[nr,nc]
            next_cell=cell if collision else (nr,nc)
            done=next_cell == self.goal
            reward=10. if done else -.2 if collision else -.05
            outcomes.append((prob,self.state(next_cell),reward,done))
        return outcomes

    def sample(self, state, action, rng):
        outcomes=self.transitions(state,action)
        _,next_state,reward,done=outcomes[rng.choice(len(outcomes),p=[o[0] for o in outcomes])]
        return next_state,reward,done


def action_values(env, values, gamma=.95):
    return np.asarray([[sum(p*(r+gamma*(not done)*values[s2]) for p,s2,r,done in env.transitions(s,a))
                         for a in range(4)] for s in range(env.n_states)])


def value_iteration(env, gamma=.95, tolerance=1e-9):
    values=np.zeros(env.n_states)
    for iteration in range(10000):
        q=action_values(env,values,gamma);new=q.max(axis=1)
        if np.max(np.abs(new-values)) < tolerance:
            return new,action_values(env,new,gamma).argmax(axis=1),iteration+1
        values=new
    raise RuntimeError('value iteration did not converge')


def policy_iteration(env, gamma=.95, tolerance=1e-9):
    policy=np.zeros(env.n_states,dtype=int);values=np.zeros(env.n_states)
    for iteration in range(100):
        for _ in range(10000):
            q=action_values(env,values,gamma);new=q[np.arange(env.n_states),policy]
            if np.max(np.abs(new-values)) < tolerance:values=new;break
            values=new
        improved=action_values(env,values,gamma).argmax(axis=1)
        if np.array_equal(improved,policy):return values,policy,iteration+1
        policy=improved
    raise RuntimeError('policy iteration did not converge')


def q_update(q, state, action, reward, next_state, terminated, alpha=.2, gamma=.95):
    target=reward+gamma*(not terminated)*np.max(q[next_state])
    q[state,action]+=alpha*(target-q[state,action])
    return float(target)


def learn(env, episodes=3000, seed=0):
    rng=np.random.default_rng(seed);q=np.zeros((env.n_states,4));returns=[]
    for episode in range(episodes):
        state=env.state(env.start);total=0.
        epsilon=max(.05,1.-episode/(episodes*.8))
        for _ in range(150):
            action=int(rng.integers(4)) if rng.random()<epsilon else int(q[state].argmax())
            s2,reward,terminated=env.sample(state,action,rng)
            q_update(q,state,action,reward,s2,terminated)
            total+=reward;state=s2
            if terminated:break
        # The 150-step collection cutoff is external, not a terminal state.
        returns.append(total)
    return q,np.asarray(returns)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--method',choices=['value','policy','q'],default='q')
    p.add_argument('--episodes',type=int,default=3000)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--slip',type=float,default=.1)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists() or a.episodes<1:p.error('Use a new output directory and positive episode count')
    env=GridRobot(a.slip);a.out.mkdir(parents=True)
    if a.method=='q':
        q,returns=learn(env,a.episodes,a.seed);policy=q.argmax(axis=1);values=q.max(axis=1);np.save(a.out/'q.npy',q)
        np.savetxt(a.out/'returns.csv',returns,delimiter=',',header='episode_return',comments='')
    else:
        values,policy,_=(value_iteration if a.method=='value' else policy_iteration)(env)
        returns=np.array([])
    np.save(a.out/'policy.npy',policy)
    rng=np.random.default_rng(10000+a.seed);successes=0;rows=[]
    for episode in range(100):
        state=env.state(env.start);r,c=env.start;points=[(0.,c+.5,r+.5,0.)]
        for step in range(150):
            action=int(policy[state]);state,_,done=env.sample(state,action,rng)
            r,c=env.cell(state);points.append(((step+1)*.2,c+.5,r+.5,action*np.pi/2))
            if done:successes+=1;break
        if episode==0:rows=points
    walls=[(0,0,6,0),(6,0,6,6),(6,6,0,6),(0,6,0,0)]
    circles=[(c+.5,r+.5,.5) for r,c in np.argwhere(env.grid)]
    save_replay(a.out/'replay.html',rows,walls,circles,[(5.5,5.5)],(6,6),'Lab 6: '+a.method)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6,5));masked=values.reshape(6,6).copy();masked[env.grid]=np.nan
    im=ax.imshow(masked,origin='lower',cmap='viridis');fig.colorbar(im,ax=ax,label='estimated discounted return')
    for r,c in np.argwhere(~env.grid):
        if (r,c)!=env.goal:
            dr,dc=MOVES[policy[env.state((r,c))]];ax.arrow(c,r,dc*.25,dr*.25,head_width=.13,color='white')
    ax.set(title='Value and greedy policy',xlabel='column',ylabel='row');fig.savefig(a.out/'policy.png',dpi=140);plt.close(fig)
    if len(returns):
        fig,ax=plt.subplots();window=min(100,len(returns));ax.plot(np.arange(window-1,len(returns)),np.convolve(returns,np.ones(window)/window,'valid'));ax.set(xlabel='training episode',ylabel='mean return over last '+str(window));fig.savefig(a.out/'learning.png',dpi=140);plt.close(fig)
    report=dict(method=a.method,seed=a.seed,slip=a.slip,evaluation_episodes=100,successes=successes,environment='discrete grid robot',evaluation_rng_seed=10000+a.seed)
    (a.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


# Optional student implementation; the environment value is a module name.
import os as _os
import importlib as _importlib
if _os.environ.get('ARC_TABULAR'):
    q_update = _importlib.import_module(_os.environ['ARC_TABULAR']).q_update

if __name__=='__main__':main()

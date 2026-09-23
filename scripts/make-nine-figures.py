#!/usr/bin/env python3
"""Build precise diagrams and analytical plots for the nine-lab reader.

The mapping, navigation and tabular plots are copied from executed experiments.
Other plots illustrate equations or interface rules, not measured performance.
"""
from pathlib import Path
import argparse
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/figures/nine'
plt.rcParams.update({'font.size':12,'axes.spines.top':False,'axes.spines.right':False,
                     'axes.labelcolor':'#213247','text.color':'#213247','axes.titleweight':'bold'})


def flow(number,title,labels,edges):
    fig,ax=plt.subplots(figsize=(10,5));ax.set(xlim=(0,10),ylim=(0,5));ax.axis('off')
    positions=[(2.4,3.45),(7.5,3.45),(2.4,1.25),(7.5,1.25)]
    for i,(label,(x,y)) in enumerate(zip(labels,positions)):
        box=FancyBboxPatch((x-1.85,y-.63),3.7,1.26,boxstyle='round,pad=.08',facecolor='#edf7f8' if i%2==0 else '#f4f0e9',edgecolor='#35889a',lw=1.5)
        ax.add_patch(box);ax.text(x,y,label,ha='center',va='center',fontsize=13)
    for a,b,label in edges:
        x,y=positions[a];u,v=positions[b]
        dx,dy=u-x,v-y
        if abs(dx)>abs(dy):start=(x+np.sign(dx)*1.98,y);end=(u-np.sign(dx)*1.98,v)
        else:start=(x,y+np.sign(dy)*.77);end=(u,v-np.sign(dy)*.77)
        ax.add_patch(FancyArrowPatch(start,end,arrowstyle='-|>',mutation_scale=16,color='#537087',lw=1.8))
        if label:ax.text((start[0]+end[0])/2+.1,(start[1]+end[1])/2+.13,label,fontsize=10,ha='center',backgroundcolor='white')
    ax.set_title(title,pad=15,fontsize=17);fig.tight_layout();fig.savefig(OUT/f'lab{number:02d}_flow.png',dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--map-run',type=Path,required=True);p.add_argument('--navigation-run',type=Path,required=True);p.add_argument('--tabular-run',type=Path,required=True);a=p.parse_args();OUT.mkdir(parents=True,exist_ok=True)
    diagrams={
        2:('A coordinate frame belongs to a part or reference', ['odom\nLocal movement reference','base_footprint / base_link\nRobot body reference','wheel frames\nJoints change their angles','laser_link\nFixed mounting offset'],[(0,1,'motion'),(1,3,'mount'),(1,2,'')]),
        3:('Mapping with supplied poses and SLAM solve different problems',['Scan + supplied pose\nKnown-pose mapper','Free / occupied evidence\nUpdate grid cells','Scan + odometry\nSLAM pose constraints','Estimated trajectory\nand occupancy map'],[(0,1,''),(2,3,'')]),
        4:('A complete route needs both planning and control',['Map + robot footprint\nInflate blocked space','Planner\nChoose a route','Measured robot pose\nClose the feedback loop','Controller\nChoose body velocity'],[(0,1,''),(1,3,'path'),(3,2,'motion'),(2,0,'pose')]),
        5:('Nav2 coordinates a continuing navigation task',['Goal action\nRequest and final result','Planner server\nGlobal route','Localisation + sensors\nPose and obstacles','Controller server\nTrack and avoid'],[(0,1,''),(1,3,'path'),(2,3,'feedback'),(3,0,'')]),
        6:('Learning uses the consequences of a decision',['Agent policy\nChoose an action','Environment\nApply motion and slip','Value update\nRevise an estimate','Reward + next state\nObserved transition'],[(0,1,'action'),(1,3,''),(3,2,'sample'),(2,0,'learn')]),
        7:('DQN separates data collection from value updates',['Robot interaction\nCollect transitions','Replay buffer\nSample a mixed batch','Online Q network\nTrain selected action value','Target Q network\nSupply bootstrap value'],[(0,1,''),(1,3,'next obs'),(3,2,'target'),(2,0,'policy')]),
        8:('Actor and critic learn different quantities',['Actor\nAction distribution','Environment rollout\nActions and rewards','Policy update\nUse estimated advantages','Critic\nExpected return estimate'],[(0,1,''),(1,3,''),(3,2,'baseline'),(2,0,'update')]),
        9:('Preserve the contract at the ROS boundary',['Scan + odometry\nFrames and timestamps','Observation adapter\nValidate and normalise','Robot controller\nAct, then measure again','Policy + outer checks\nAction and stop decision'],[(0,1,''),(1,3,'29 values'),(3,2,'Twist'),(2,0,'feedback')])}
    for n,(title,labels,edges) in diagrams.items():flow(n,title,labels,edges)
    # Differential-drive equations with fixed left wheel speed.
    right=np.linspace(-3,9,100);left=3.;v=.05*(right+left)/2;w=.05*(right-left)/.35
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for ax,values,label in zip(axes,[v,w],['body forward speed (m/s)','body turning speed (rad/s)']):
        ax.plot(right,values,color='#087e94');ax.axvline(3,color='#dd921a',ls='--');ax.axhline(0,color='#b7c3ca',lw=.8);ax.set(xlabel='right wheel speed (rad/s)',ylabel=label)
    fig.suptitle('Ideal differential drive: left wheel fixed at 3 rad/s');fig.savefig(OUT/'lab02_plot.png',dpi=150);plt.close(fig)
    for n,source in [(3,a.map_run/'map.png'),(4,a.navigation_run/'trajectory.png'),(6,a.tabular_run/'policy.png')]:shutil.copy2(source,OUT/f'lab{n:02d}_plot.png')
    # Geometry illustration, deliberately not a measured Nav2 trace.
    fig,ax=plt.subplots(figsize=(7,5),layout='constrained')
    ax.add_patch(Circle((2,2),.65,color='#f6dca8',label='illustrative inflated region'))
    ax.add_patch(Circle((2,2),.35,color='#475c70',label='obstacle'))
    ax.add_patch(Circle((3.1,2),.22,color='#087e94',label='robot footprint'))
    ax.plot([.3,1.1,2,3,3.8],[1,1,1.15,1.15,2.5],'--',color='#178a67',label='example global path')
    ax.set(xlim=(0,4),ylim=(0,3.6),aspect='equal',xlabel='x (m)',ylabel='y (m)',title='Illustration: plan for a body, not only a point');ax.legend(loc='upper left',fontsize=9);fig.savefig(OUT/'lab05_plot.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained');x=np.arange(2)
    ax.bar(x-.17,[5,4],.34,label='online Q');ax.bar(x+.17,[2,6],.34,label='target Q',color='#dd921a');ax.set(xticks=x,xticklabels=['action 0','action 1'],ylabel='example Q estimate',title='Double DQN selects action 0; evaluates it as 2');ax.legend();fig.savefig(OUT/'lab07_plot.png',dpi=150);plt.close(fig)
    ratios=np.linspace(.3,1.7,200);fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for ax,adv in zip(axes,[1,-1]):
        ax.plot(ratios,ratios*adv,ls='--',label='unclipped');ax.plot(ratios,np.minimum(ratios*adv,np.clip(ratios,.8,1.2)*adv),label='clipped objective');ax.axvspan(.8,1.2,alpha=.15,color='#087e94');ax.set(xlabel='new / old probability ratio',ylabel='objective to maximise',title=f'Example advantage = {adv}');ax.legend(fontsize=9)
    fig.savefig(OUT/'lab08_plot.png',dpi=150);plt.close(fig)
    ages=np.linspace(0,1,101);fig,ax=plt.subplots(figsize=(7,4),layout='constrained');ax.step(ages,np.where(ages<.5,.15,0),where='post',color='#087e94');ax.axvline(.5,ls='--',color='#dd921a');ax.set(xlabel='sensor arrival age (wall seconds)',ylabel='permitted example command (m/s)',title='Illustration: stale data suppresses motion',ylim=(-.01,.18));fig.savefig(OUT/'lab09_plot.png',dpi=150);plt.close(fig)
    print(OUT)


if __name__=='__main__':main()

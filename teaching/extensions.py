"""Lab 4 extension experiments: reachable frontiers and local ICP alignment."""
import argparse
import json
from pathlib import Path
import numpy as np
from teaching.navigation import reachable_frontiers,icp


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Choose a new output directory')
    a.out.mkdir(parents=True)
    grid=np.full((20,20),-1,dtype=np.int8);grid[2:17,2:12]=0;grid[7:14,7]=100
    frontiers=reachable_frontiers(grid,(3,3))
    rng=np.random.default_rng(7);target=rng.uniform(-1,1,(80,2));angle=.05
    exact=np.asarray([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);shift=np.array([.04,-.03])
    source=(target-shift)@exact
    rotation,translation,residuals=icp(source,target)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    axes[0].imshow(grid,origin='lower',cmap='Greys');r,c=np.asarray(frontiers).T;axes[0].scatter(c,r,s=10,color='#dd921a');axes[0].set(title='Reachable free frontier cells',xlabel='column',ylabel='row')
    axes[1].scatter(*target.T,s=15,label='target');axes[1].scatter(*source.T,s=15,label='before');axes[1].scatter(*(source@rotation.T+translation).T,s=8,label='aligned');axes[1].legend();axes[1].set(title='Local point-to-point ICP',xlabel='x (m)',ylabel='y (m)',aspect='equal')
    fig.savefig(a.out/'extensions.png',dpi=140);plt.close(fig)
    report=dict(frontier_cells=len(frontiers),icp_translation=translation.tolist(),icp_residuals_m=residuals,scope='Demonstrations of frontier detection and local alignment, not complete autonomous exploration or SLAM.')
    (a.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


if __name__=='__main__':main()

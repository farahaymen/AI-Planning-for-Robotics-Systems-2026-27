"""Known-pose occupancy mapping demonstration; this is not SLAM."""
import argparse
import json
from pathlib import Path
import numpy as np
from arc_rl.nav_core import FastNavEnv, Scenario
from starters.lab03.occupancy import OccupancyMap, MappingParams
from teaching.animation import save_replay


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Choose a new output directory')
    a.out.mkdir(parents=True)
    scenario=Scenario(name='mapping-room',bounds=(6.,6.),walls=[(0,0,6,0),(6,0,6,6),(6,6,0,6),(0,6,0,0)],static_circles=[(3.,3.,.4)],dynamic_circles=[],start_pose=(1.,1.,0.),goals=[(5.,5.)],max_episode_steps=1000)
    env=FastNavEnv(scenario=scenario,randomise_start=False,lidar_noise_std=0.)
    env.reset(seed=0);mapper=OccupancyMap(6.2,6.2,origin=(-.1,-.1),params=MappingParams(resolution=.1))
    route=np.asarray([(1,1),(5,1),(5,5),(1,5),(1,1)],float);rows=[]
    for u,v in zip(route[:-1],route[1:]):
        for f in np.linspace(0,1,30,endpoint=False):
            env.x,env.y=u+(v-u)*f;env.theta=float(np.arctan2(*(v-u)[::-1]))
            beams=env._raycast();laser_x=env.x+.1*np.cos(env.theta);laser_y=env.y+.1*np.sin(env.theta)
            mapper.integrate_scan((laser_x,laser_y,env.theta),beams,env._angles)
            rows.append((len(rows)*.1,env.x,env.y,env.theta))
    probability=1/(1+np.exp(-mapper.log_odds))
    # Conventional trinary export: black occupied, white free, gray unknown.
    pixels=np.full(probability.shape,205,dtype=np.uint8);pixels[probability>.65]=0;pixels[probability<.196]=254
    from PIL import Image
    Image.fromarray(np.flipud(pixels)).save(a.out/'map.pgm')
    import yaml
    (a.out/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',mode='trinary',resolution=.1,origin=[-.1,-.1,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6,5));im=ax.imshow(probability,origin='lower',extent=[-.1,6.1,-.1,6.1],cmap='Greys',vmin=0,vmax=1)
    rows=np.asarray(rows);ax.plot(rows[:,1],rows[:,2],color='#087e94');ax.set(xlabel='x (m)',ylabel='y (m)',title='Known-pose occupancy mapping');fig.colorbar(im,ax=ax,label='occupancy probability');fig.savefig(a.out/'map.png',dpi=140);plt.close(fig)
    save_replay(a.out/'replay.html',rows,scenario.walls,scenario.static_circles,[],scenario.bounds,'Lab 3: collecting scans with known poses')
    (a.out/'report.json').write_text(json.dumps(dict(scans=len(rows),pose_source='exact prescribed trajectory',slam=False),indent=2));print(a.out/'map.yaml')


if __name__=='__main__':main()

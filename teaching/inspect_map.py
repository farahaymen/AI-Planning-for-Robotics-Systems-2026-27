"""Show row/column indices and inflated free space before choosing planning cells."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from teaching.navigation import load_map,inflate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Choose a new output directory')
    grid,res,origin=load_map(a.map);blocked=inflate(grid,math.ceil(.30/res));a.out.mkdir(parents=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
    axes[0].imshow(grid,origin='lower',cmap='gray_r',vmin=-1,vmax=100);axes[0].set(title='Occupancy (-1 unknown, 0 free, 100 occupied)')
    axes[1].imshow(blocked,origin='lower',cmap='Greys');axes[1].set(title='Planning grid: white cells are available')
    for ax in axes:ax.set(xlabel='column',ylabel='row')
    fig.savefig(a.out/'grid.png',dpi=160);plt.close(fig)
    (a.out/'metadata.json').write_text(json.dumps(dict(shape=list(grid.shape),resolution=res,origin=origin.tolist(),inflated_free_cells=int(np.sum(~blocked))),indent=2))
    print(a.out/'grid.png')


if __name__=='__main__':main()

"""Lab 4: search, reachable frontiers, local scan alignment and robot tracking.

The grid convention is grid[row, column], row increases with world y.
These small reference algorithms favour clarity over large-map performance.
"""
from __future__ import annotations
import argparse
from collections import deque
import heapq
import json
import math
from pathlib import Path
import numpy as np
from teaching.animation import save_replay


def line_of_sight(blocked, a, b):
    """Conservative segment versus CLOSED occupied cell squares.

    A line that touches a blocked corner is rejected. This exact slab test
    avoids the missed thin obstacle problem of sampling along a segment.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    h, w = blocked.shape
    if any(p[0] < 0 or p[0] >= h or p[1] < 0 or p[1] >= w for p in (a, b)):
        return False
    low = np.maximum(0, np.floor(np.minimum(a, b)-.5).astype(int))
    high = np.minimum([h-1, w-1], np.ceil(np.maximum(a, b)+.5).astype(int))
    for r, c in np.argwhere(blocked[low[0]:high[0]+1, low[1]:high[1]+1]) + low:
        enter, leave = 0., 1.
        for axis, centre in enumerate((r, c)):
            delta = b[axis]-a[axis]
            if abs(delta) < 1e-12:
                if abs(a[axis]-centre) > .5:
                    enter, leave = 1., 0.
                    break
            else:
                t0, t1 = sorted(((centre-.5-a[axis])/delta, (centre+.5-a[axis])/delta))
                enter, leave = max(enter, t0), min(leave, t1)
        if enter <= leave + 1e-12:
            return False
    return True


def search(blocked, start, goal, algorithm="astar"):
    """Four-connected BFS/Dijkstra/A*, or eight-connected Basic Theta*.

    On the four-connected unit-cost grid BFS and Dijkstra have equal cost.
    Theta* uses Euclidean distance; it need not find the continuous optimum.
    """
    if algorithm not in ("bfs", "dijkstra", "astar", "theta"):
        raise ValueError("unknown search algorithm")
    h, w = blocked.shape
    for p in (start, goal):
        if not (0 <= p[0] < h and 0 <= p[1] < w) or blocked[p]:
            raise ValueError("start and goal must be free cells")
    distance = lambda a,b: math.hypot(a[0]-b[0], a[1]-b[1])
    heuristic = lambda p: (distance(p, goal) if algorithm == "theta" else
                            abs(p[0]-goal[0])+abs(p[1]-goal[1]) if algorithm == "astar" else 0)
    parent, cost, closed = {start:start}, {start:0.}, set()
    frontier = [(heuristic(start), 0, start)]
    serial = 0
    while frontier:
        _, _, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        if node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1], len(closed)
        moves = [(-1,0),(1,0),(0,-1),(0,1)]
        if algorithm == "theta":
            moves += [(-1,-1),(-1,1),(1,-1),(1,1)]
        for dr, dc in moves:
            nb = node[0]+dr, node[1]+dc
            if nb in closed or not line_of_sight(blocked, node, nb):
                continue
            previous = parent[node] if algorithm == "theta" and line_of_sight(blocked, parent[node], nb) else node
            candidate = cost[previous] + distance(previous, nb)
            if candidate < cost.get(nb, float("inf")):
                cost[nb], parent[nb] = candidate, previous
                serial += 1
                heapq.heappush(frontier, (candidate+heuristic(nb), serial, nb))
    return [], len(closed)


def reachable_frontiers(grid, start):
    """Return reachable FREE cells adjacent to unknown (-1), in BFS order.

    Input must already have obstacles inflated. Unknown cells are not entered.
    """
    h,w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w) or grid[start] != 0:
        return []
    queue, seen, result = deque([start]), {start}, []
    while queue:
        r,c = queue.popleft()
        neighbours = [(a,b) for a,b in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)) if 0 <= a < h and 0 <= b < w]
        if any(grid[p] == -1 for p in neighbours):
            result.append((r,c))
        for p in neighbours:
            if grid[p] == 0 and p not in seen:
                seen.add(p); queue.append(p)
    return result


def icp(source, target, iterations=40, max_distance=.5):
    """Point-to-point 2D ICP. Return R,t such that source @ R.T + t aligns target.

    Requires a close initial alignment and sufficient overlap. Nearest-neighbour
    correspondences are recomputed each iteration; reflections are excluded.
    """
    source, target = np.asarray(source, float), np.asarray(target, float)
    if source.ndim != 2 or target.ndim != 2 or source.shape[1] != 2 or target.shape[1] != 2 or min(len(source),len(target)) < 3:
        raise ValueError("two point sets of shape (N,2), with at least 3 points, required")
    moved, R, t = source.copy(), np.eye(2), np.zeros(2)
    residuals = []
    for _ in range(iterations):
        distances = np.linalg.norm(moved[:,None,:]-target[None,:,:], axis=2)
        nearest = distances.argmin(axis=1)
        keep = distances[np.arange(len(moved)),nearest] < max_distance
        if keep.sum() < 3:
            raise ValueError("insufficient overlapping correspondences")
        a,b = moved[keep], target[nearest[keep]]
        ac,bc = a.mean(axis=0), b.mean(axis=0)
        u,_,vt = np.linalg.svd((a-ac).T @ (b-bc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1; rotation = vt.T @ u.T
        translation = bc-rotation @ ac
        moved = moved @ rotation.T + translation
        R,t = rotation @ R, rotation @ t + translation
        residuals.append(float(np.sqrt(np.mean(np.sum((moved[keep]-b)**2,axis=1)))))
        if np.linalg.norm(translation) < 1e-8 and np.linalg.norm(rotation-np.eye(2)) < 1e-8:
            break
    return R,t,residuals


def load_map(path):
    """Read Nav2 trinary YAML/PGM map, preserving resolution and origin yaw."""
    import yaml
    from PIL import Image
    path = Path(path)
    metadata = yaml.safe_load(path.read_text())
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("This teaching loader supports mode: trinary only")
    pixels = np.asarray(Image.open(path.parent / metadata['image']).convert('L'), dtype=float)
    probability = pixels/255 if metadata.get('negate',0) else (255-pixels)/255
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    grid[probability < metadata['free_thresh']] = 0
    grid[probability > metadata['occupied_thresh']] = 100
    return np.flipud(grid), float(metadata['resolution']), np.asarray(metadata['origin'],float)


def inflate(grid, cells):
    blocked = grid != 0  # unknown is blocked for this conservative planner
    output = blocked.copy()
    h,w = blocked.shape
    for r,c in np.argwhere(blocked):
        for dr in range(-cells,cells+1):
            for dc in range(-cells,cells+1):
                if dr*dr+dc*dc <= cells*cells and 0 <= r+dr < h and 0 <= c+dc < w:
                    output[r+dr,c+dc] = True
    if cells:
        output[:cells]=True; output[-cells:]=True; output[:,:cells]=True; output[:,-cells:]=True
    return output


def follow(path, controller="pursuit", dt=.05):
    """Kinematic closed-loop experiment on a fixed metric path (no Gazebo)."""
    from starters.algorithms.control import PID
    points = np.asarray(path,float)
    x,y = points[0]; theta=0.; index=1; rows=[(0.,x,y,theta)]
    pid = PID(kp=2.5, kd=.12, dt=dt)
    for k in range(6000):
        if np.linalg.norm(points[-1]-[x,y]) < .15:
            break
        while index < len(points)-1 and np.linalg.norm(points[index]-[x,y]) < .45:
            index += 1
        dx,dy = points[index]-[x,y]
        error = math.atan2(math.sin(math.atan2(dy,dx)-theta),math.cos(math.atan2(dy,dx)-theta))
        v=.25*max(0.,math.cos(error))
        if controller == "pid":
            w=pid(error)
        else:
            local_y=-math.sin(theta)*dx+math.cos(theta)*dy
            w=2*v*local_y/max(dx*dx+dy*dy,1e-8) if math.cos(error)>0 else math.copysign(1.,error)
        w=float(np.clip(w,-1.8,1.8))
        x+=v*math.cos(theta)*dt; y+=v*math.sin(theta)*dt; theta+=w*dt
        rows.append(((k+1)*dt,x,y,theta))
    return np.asarray(rows)


def trajectory_clearance(grid, resolution, trajectory, radius=.22):
    """Minimum circular-footprint clearance to blocked/unknown cell squares."""
    cells=np.argwhere(grid != 0)
    h,w=grid.shape
    minimum=float('inf')
    centres=(cells[:, ::-1]+.5)*resolution
    for x,y in trajectory[:,1:3]:
        clearance=min(x,y,w*resolution-x,h*resolution-y)-radius
        if len(centres):
            delta=np.maximum(np.abs(centres-[x,y])-resolution/2,0)
            clearance=min(clearance,float(np.linalg.norm(delta,axis=1).min())-radius)
        minimum=min(minimum,clearance)
    return minimum


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', type=Path)
    p.add_argument('--start', nargs=2, type=int, default=[8,8], help='row column')
    p.add_argument('--goal', nargs=2, type=int, default=[48,48], help='row column')
    p.add_argument('--algorithm',choices=['bfs','dijkstra','astar','theta'],default='astar')
    p.add_argument('--controller',choices=['pid','pursuit'],default='pursuit')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Choose a new output directory')
    if a.map:
        grid,resolution,origin=load_map(a.map)
    else:
        grid=np.zeros((60,60),np.int8);grid[0,:]=grid[-1,:]=100;grid[:,0]=grid[:,-1]=100
        grid[15:45,29:32]=100;resolution=.1;origin=np.zeros(3)
    blocked=inflate(grid,math.ceil(.30/resolution))
    path,expanded=search(blocked,tuple(a.start),tuple(a.goal),a.algorithm)
    if not path:p.error('No path. Choose reachable free start/goal cells after inflation.')
    # Subdivide long Theta* segments before choosing a lookahead target.
    route=[]
    for u,v in zip(path[:-1],path[1:]):
        for f in np.linspace(0,1,max(2,int(np.linalg.norm(np.subtract(v,u)))) ,endpoint=False):
            r,c=np.asarray(u)+(np.asarray(v)-u)*f;route.append(((c+.5)*resolution,(r+.5)*resolution))
    r,c=path[-1];route.append(((c+.5)*resolution,(r+.5)*resolution))
    trajectory=follow(route,a.controller)
    # Draw in map-local metric coordinates. World origin/yaw stored in report.
    a.out.mkdir(parents=True)
    circles=[((c+.5)*resolution,(r+.5)*resolution,resolution*.71) for r,c in np.argwhere(grid==100)]
    save_replay(a.out/'replay.html',trajectory,circles=circles,goals=[route[-1]],bounds=(grid.shape[1]*resolution,grid.shape[0]*resolution),title='Lab 4: '+a.algorithm+' + '+a.controller)
    np.savetxt(a.out/'trajectory.csv',trajectory,delimiter=',',header='time_s,x_m,y_m,theta_rad',comments='')
    np.save(a.out/'path_cells.npy',np.asarray(path))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(6,6));ax.imshow(blocked,origin='lower',cmap='Greys',extent=[0,grid.shape[1]*resolution,0,grid.shape[0]*resolution])
    ax.plot(*np.asarray(route).T,'--',label='planned path');ax.plot(trajectory[:,1],trajectory[:,2],label='robot');ax.legend();ax.set(xlabel='map-local x (m)',ylabel='map-local y (m)');fig.savefig(a.out/'trajectory.png',dpi=140);plt.close(fig)
    clearance=trajectory_clearance(grid,resolution,trajectory)
    report=dict(minimum_blocked_space_clearance_m=clearance,trajectory_valid=bool(clearance>0),algorithm=a.algorithm,controller=a.controller,expanded=expanded,goal_error_m=float(np.linalg.norm(trajectory[-1,1:3]-route[-1])),resolution=resolution,origin=origin.tolist(),environment='kinematic teaching simulation')
    (a.out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))


# Optional student implementation; the environment value is a module name.
import os as _os
import importlib as _importlib
if _os.environ.get('ARC_SEARCH'):
    search = _importlib.import_module(_os.environ['ARC_SEARCH']).search

if __name__ == '__main__':main()

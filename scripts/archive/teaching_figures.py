"""Reproduce the laboratory diagrams and controlled numerical illustrations.

All plots are synthetic, explicitly controlled experiments. None is a claimed
Gazebo measurement or trained-policy benchmark. Run from the repository root.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.path import Path as PlotPath
import numpy as np

from arc_rl.nav_core import OBS, RewardTerms, downsample_scan, scale_action
from arc_rl.nav_core import reward_dense_progress, reward_sparse_safe
from starters.lab02.diffdrive import integrate_pose
from starters.lab03.occupancy import OccupancyMap
from starters.lab05.particle_filter import ParticleFilter
from starters.lab04.grid_tools import inflate
from starters.lab05.recovery import StuckDetector
from starters.lab09.hybrid import HybridController

INK, BLUE, ORANGE, GREEN = "#24334a", "#087e94", "#ca6d25", "#368359"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.labelcolor": INK,
                     "text.color": INK, "axes.titlecolor": INK,
                     "axes.prop_cycle": plt.cycler(color=[BLUE, ORANGE, GREEN])})


def save(fig, out, name):
    fig.savefig(out / name, dpi=165, facecolor="white")
    plt.close(fig)


def flow(out, lab, title, labels, edges):
    """Two rows of three nodes; edges use patch-aware endpoints."""
    fig, ax = plt.subplots(figsize=(11, 4.6))
    fig.subplots_adjust(left=0.025, right=0.975, bottom=0.04, top=0.85)
    positions = [(0.16, 0.76), (0.5, 0.76), (0.84, 0.76),
                 (0.16, 0.23), (0.5, 0.23), (0.84, 0.23)]
    boxes = []
    for index, label in enumerate(labels):
        x, y = positions[index]
        box = FancyBboxPatch((x - 0.128, y - 0.135), 0.256, 0.27,
                             boxstyle="round,pad=0.012,rounding_size=0.025",
                             linewidth=1.2, edgecolor=BLUE,
                             facecolor="#eff7f9" if index < 3 else "#f5f5f1")
        ax.add_patch(box)
        ax.text(x, y, label, ha="center", va="center", fontsize=11, linespacing=1.45)
        boxes.append(box)
    for start, end, label, curve in edges:
        a, b = positions[start], positions[end]
        if abs(a[0] - b[0]) > 0.5 and abs(a[1] - b[1]) < 0.01:
            route = PlotPath([(a[0], a[1] - 0.147), (a[0], 0.018),
                              (b[0], 0.018), (b[0], b[1] - 0.147)])
            ax.add_patch(FancyArrowPatch(path=route, arrowstyle="-|>",
                                         mutation_scale=13, color=INK, lw=1.35))
            if label:
                ax.text(0.5, 0.046, label, ha="center", va="center", fontsize=8.5,
                        bbox=dict(facecolor="white", edgecolor="none", pad=1))
            continue
        arrow = FancyArrowPatch(a, b, patchA=boxes[start], patchB=boxes[end],
                                arrowstyle="-|>", mutation_scale=13,
                                color=INK, lw=1.35,
                                connectionstyle=f"arc3,rad={curve}")
        ax.add_patch(arrow)
        if label:
            x = (a[0] + b[0]) / 2 + curve * (b[1] - a[1]) / 2
            y = (a[1] + b[1]) / 2 - curve * (b[0] - a[0]) / 2
            ax.text(x, y + (0.035 if abs(a[1] - b[1]) < 0.01 else 0), label,
                    ha="center", va="center", fontsize=8.5,
                    bbox=dict(facecolor="white", edgecolor="none", pad=2))
    ax.set(xlim=(0, 1), ylim=(0, 1))
    ax.axis("off")
    fig.suptitle(f"Lab {lab:02d}  |  {title}", fontsize=16, fontweight="bold", y=0.97)
    save(fig, out, f"lab{lab:02d}_flow.png")


def diagrams(out):
    specifications = [
        ("One message stream, two subscribers",
         ["Source node\npublisher", "Topic: /range\nFloat32 messages", "Monitor node\nsubscriber",
          "Terminal command\nchanges amplitude", "Terminal observer\ntopic echo", "Terminal output\nrate + close-range alerts"],
         [(0,1,"publish",0),(1,2,"receive",0),(3,0,"parameter",0),(1,4,"receive",0),(2,5,"print",0)]),
        ("Coordinate frames have distinct owners",
         ["odom\ncontinuous motion", "base_footprint\nground reference", "base_link\nrigid body",
          "Wheel feedback\ncontroller odometry", "Wheel links\njoint positions", "laser_link\nfixed sensor offset"],
         [(0,1,"dynamic",0),(1,2,"fixed",0),(3,0,"estimate",0),(2,4,"",0),(2,5,"fixed",0)]),
        ("Two data paths share one simulation clock",
         ["Gazebo sensors\nscan + IMU", "ros_gz_bridge\nmessage conversion", "ROS consumers\nRViz + recorder",
          "Simulated wheels\nros2_control", "Odometry + TF\ncommand relay", "Simulation clock\nuse_sim_time"],
         [(0,1,"",0),(1,2,"",0),(3,4,"",0),(4,2,"",0),(5,2,"time",0),(5,4,"time",0)]),
        ("Geometry comes before occupancy evidence",
         ["Scan header + ranges\nsensor measurement", "Matched odometry\nheader timestamp", "Sensor pose\nbase + extrinsic",
          "Ray cells\nfree / hit / invalid", "Log-odds grid\nadd + clamp", "Map artifacts\nPGM + YAML + report"],
         [(0,1,"time match",0),(1,2,"transform",0),(2,3,"",0),(3,4,"evidence",0),(4,5,"threshold",0)]),
        ("One global correction, two operating modes",
         ["Odometry + scan\ntime + TF", "SLAM Toolbox\nestimate trajectory", "Saved map\nimage + metadata",
          "AMCL\npose hypotheses", "map to odom\nglobal correction", "Robot in map\ncomposed transforms"],
         [(0,1,"mapping",0),(1,2,"save",0),(0,3,"localise",0),(2,3,"",0),(1,4,"or",0),(3,4,"or",0),(4,5,"",0)]),
        ("A route must become executable motion",
         ["Navigation action\ngoal + feedback", "Planner\nroute selection", "Controller\nvelocity selection",
          "Behaviour tree\nretry + recovery", "Costmaps\nmap + scan + inflation", "Robot + localisation\nfeedback + pose"],
         [(0,1,"request",0),(1,2,"path",0),(0,3,"status",0),(4,1,"costs",0),(4,2,"",0),(2,5,"command",0),(5,4,"",0)]),
        ("Recovery is a bounded decision loop",
         ["Goal execution\nplan + follow", "Progress evidence\ndisplacement + time", "Failure decision\nretry budget",
          "Recovered motion\nreset escalation", "Recovery action\nclear / spin / wait / back", "Abort result\nreport cause"],
         [(0,1,"observe",0),(1,2,"failure",0),(2,4,"retry",0),(4,3,"progress",0),(3,0,"resume",0),(2,5,"exhausted",0)]),
        ("An environment defines the learning problem",
         ["Policy observation\n29 encoded values", "Policy action\ntwo values in [-1, 1]", "Motion + geometry\nadvance 0.05 seconds",
          "Measurement encoding\nbeams + relative goal", "Reward + outcome\nsuccess / contact / limit", "Episode state\npose + goal + counters"],
         [(0,1,"policy",0),(1,2,"scale",0),(2,5,"state",0),(5,4,"assess",0),(5,3,"",0.42),(3,0,"observe",0)]),
        ("Separate learning from the comparison",
         ["Training bank\ngeometry seeds 0–99", "PPO collection\n4 x 512 transitions", "Policy update\nadvantages + clipping",
          "Validation bank\nseeds 500–509", "Saved model\nweights + metadata", "Matched outcomes\npaths + episode CSV"],
         [(0,1,"reset",0),(1,2,"batch",0),(2,4,"save",0),(3,5,"episodes",-0.42),(4,5,"policy",0)]),
        ("Handover needs a trigger and an exit",
         ["Classical controller\ndefault authority", "Stall AND tight front\n40-position history", "Learned controller\n60 committed steps",
          "Fresh episode\nclear state + counters", "Return to classical\ncommitment expires", "Episode evidence\nhandovers + outcomes"],
         [(0,1,"check",0),(1,2,"budget allows",0),(2,4,"timer",0),(4,0,"",0),(3,0,"reset",0),(2,5,"record",0)]),
    ]
    for lab, (title, labels, edges) in enumerate(specifications, 1):
        flow(out, lab, title, labels, edges)


def plots(out):
    fig, ax = plt.subplots(figsize=(10, 4), layout="constrained")
    t = np.arange(0, 14, 0.1)
    for amplitude in (2.0, 0.4):
        ax.plot(t, amplitude * (1.5 + np.sin(np.arange(len(t)) * 0.1)),
                label=f"amplitude = {amplitude}")
    ax.axhline(1, ls="--", color=ORANGE, label="strict warning threshold")
    ax.set(xlabel="Elapsed time (s)", ylabel="Synthetic range (m)",
           title="Changing one parameter changes the distance stream (10 callbacks/s)")
    ax.legend(loc="lower right")
    save(fig, out, "lab01_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    for omega in (0.0, 0.25, 0.5):
        points = [(0., 0., 0.)]
        for _ in range(160):
            points.append(integrate_pose(*points[-1], 0.2, omega, 0.05))
        p = np.asarray(points)
        axes[0].plot(p[:,0], p[:,1], label=f"yaw rate {omega} rad/s")
    axes[0].set(xlabel="x (m)", ylabel="y (m)", aspect="equal", title="Exact-arc integration, 8 seconds")
    axes[0].legend(fontsize=8)
    distance = np.linspace(0, 3, 60)
    axes[1].plot(distance, distance, label="correct radius")
    axes[1].plot(distance, 1.1 * distance, label="assumed 0.055 m / true 0.050 m")
    axes[1].set(xlabel="True straight travel (m)", ylabel="Reported travel (m)", title="Ideal wheel-radius scale error")
    axes[1].legend(fontsize=8)
    save(fig, out, "lab02_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), layout="constrained")
    n = np.arange(20)
    axes[0].plot(n / 10, n, "o-", label="RTF = 1.0")
    axes[0].plot(n / 8, n, "s-", label="RTF = 0.8")
    axes[0].set(xlabel="Wall elapsed time (s)", ylabel="Message index", title="Same 10 Hz sensor, different wall rate")
    axes[0].legend()
    axes[1].bar(["Healthy\n10 Hz", "Rate fault\n3 Hz"], [0.1, 1/3], color=[BLUE, ORANGE])
    axes[1].set(ylabel="Header interval (simulation s)", title="Header timing identifies a rate change")
    save(fig, out, "lab03_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
    grid = OccupancyMap(4, 4)
    for _ in range(5):
        grid.integrate_scan((1, 1, 0), np.array([2., 2., 2.]), np.array([0., .45, .9]))
    im = axes[0].imshow(grid.probability(), origin="lower", extent=(0,4,0,4),
                         cmap="RdBu_r", vmin=0, vmax=1)
    axes[0].scatter(1, 1, color="black", s=20)
    axes[0].set(xlabel="x (m)", ylabel="y (m)", title="Three rays, each repeated five times")
    fig.colorbar(im, ax=axes[0], label="Occupancy probability", shrink=.8)
    cell = OccupancyMap(1, 1)
    values = [.5]
    for delta in [0.85] * 8 + [-0.4] * 24:
        cell._update_cell(0, 0, delta)
        values.append(cell.probability()[0,0])
    axes[1].plot(values, "o-", markersize=3)
    axes[1].axvline(8, color=ORANGE, ls="--", label="switch hits to free rays")
    axes[1].axhline(.65, color="gray", ls=":")
    axes[1].axhline(.25, color="gray", ls=":")
    axes[1].set(xlabel="Observation count", ylabel="Occupancy probability", title="Clamped confidence can be revised", ylim=(0,1))
    axes[1].legend(fontsize=8)
    save(fig, out, "lab04_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), layout="constrained")
    pf = ParticleFilter(n_particles=1000, seed=11)
    pf.seed_uniform((0, 0, 6, 6))
    prior = pf.particles.copy()
    landmarks = np.array([[0.5,0.5], [5.5,1.], [4.5,5.5]])
    truth = np.array([2.4,3.2])
    measured = np.linalg.norm(landmarks - truth, axis=1)
    predicted = np.linalg.norm(pf.particles[:,None,:2] - landmarks[None,:,:], axis=2)
    errors = np.sqrt(np.mean((predicted - measured) ** 2, axis=1))
    pf.update_weights(errors, sigma=.25)
    ess = pf.effective_sample_size()
    weights = pf.weights.copy()
    pf.resample()
    axes[0].scatter(prior[:,0], prior[:,1], s=8, c=weights, cmap="viridis")
    axes[1].scatter(pf.particles[:,0], pf.particles[:,1], s=9, alpha=.3, color=BLUE)
    for ax in axes:
        ax.scatter(*landmarks.T, marker="^", s=70, color=ORANGE, label="known landmarks")
        ax.scatter(*truth, marker="x", s=90, color="black", label="synthetic true position")
        ax.set(xlim=(0,6), ylim=(0,6), xlabel="x (m)", ylabel="y (m)", aspect="equal")
    axes[0].set_title(f"Range-based weighting; effective samples {ess:.1f}")
    axes[1].set_title("Systematic resampling (heading not observed)")
    axes[1].legend(fontsize=8, loc="lower left")
    save(fig, out, "lab05_plot.png")

    fig, axes = plt.subplots(1, 3, figsize=(11, 4), layout="constrained")
    obstacles = np.zeros((40,40), dtype=bool)
    obstacles[[0,-1],:] = True
    obstacles[:,[0,-1]] = True
    obstacles[:16,20] = True
    obstacles[24:,20] = True
    for ax, radius in zip(axes, [0,2,5]):
        ax.imshow(inflate(obstacles, radius), origin="lower", cmap="Greys", extent=(0,2,0,2), vmin=0, vmax=1)
        ax.set(title=f"Binary inflation: {radius} cells", xlabel="x (m)", ylabel="y (m)")
    fig.suptitle("0.05 m cells; doorway has eight free rows before inflation")
    save(fig, out, "lab06_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), layout="constrained")
    t = np.arange(240) * .05
    signals = {"forward 0.1 m/s": .1*t, "slow 0.02 m/s": .02*t,
               "oscillation": .06*np.sin(2*np.pi*t)}
    for name, x in signals.items():
        detector = StuckDetector()
        displacement, fired = [], []
        for value in x:
            fired.append(detector.update(float(value), 0))
            displacement.append(detector.displacement)
        axes[0].plot(t, x, label=name)
        axes[1].plot(t, displacement, label=name)
    axes[0].set(xlabel="Time (s)", ylabel="x position (m)", title="Controlled motion histories")
    axes[0].legend(fontsize=8)
    axes[1].axhline(.15, color="black", ls="--", label="displacement threshold")
    axes[1].axvspan(0, 4.95, color="gray", alpha=.1, label="history filling")
    axes[1].set(xlabel="Time (s)", ylabel="Window displacement (m)", title="Stuck only after the history is full")
    axes[1].legend(fontsize=8)
    save(fig, out, "lab07_plot.png")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), layout="constrained")
    raw = np.full(360, 4., dtype=np.float32)
    raw[181] = .35
    minimum = downsample_scan(raw)
    mean = np.array([s.mean() for s in np.array_split(raw,24)])
    axes[0].plot(np.arange(24), minimum, "o-", label="sector minimum")
    axes[0].plot(np.arange(24), mean, "x--", label="sector mean")
    axes[0].set(xlabel="Sector index", ylabel="Range (m)", title="One thin obstacle among 360 raw beams")
    axes[0].legend()
    actions = np.linspace(-1,1,101)
    v = [scale_action(np.array([a,0]))[0] for a in actions]
    axes[1].plot(actions,v)
    axes[1].axhline(0,color="gray",lw=.7)
    axes[1].axvline(0,color="gray",lw=.7)
    axes[1].set(xlabel="Normalised linear action", ylabel="Applied forward speed (m/s)", title="Reverse and forward use different scales")
    save(fig, out, "lab08_plot.png")

    fig, ax = plt.subplots(figsize=(10,4.2), layout="constrained")
    clearance = np.linspace(.12,1,150)
    dense, sparse = [], []
    for beam in clearance:
        terms = RewardTerms(2,1.98,float(beam),.2,.4,False,False,1)
        dense.append(reward_dense_progress(terms))
        sparse.append(reward_sparse_safe(terms))
    ax.plot(clearance,dense,label="dense progress")
    ax.plot(clearance,sparse,label="sparse safe")
    ax.axhline(0,color="gray",lw=.8)
    ax.set(xlabel="Minimum beam distance (m), not body clearance", ylabel="Reward for one transition",
           title="Same 0.02 m goal progress and 0.4 rad/s turn; no terminal bonus")
    ax.legend()
    save(fig, out, "lab09_plot.png")

    fig, axes = plt.subplots(2,1,figsize=(10,5.5), sharex=True, layout="constrained")
    hybrid = HybridController(lambda obs: np.array([0.,0.]), lambda obs: np.array([1.,0.]))
    chosen, front = [], []
    t = np.arange(180)*.05
    for time in t:
        distance = .4 if time < 3.0 else 2.0
        obs = np.zeros(OBS.size, dtype=np.float32)
        obs[:OBS.n_beams] = (distance - OBS.lidar_min_range)/(OBS.lidar_max_range - OBS.lidar_min_range)
        chosen.append(hybrid(obs, position=(0.,0.))[0])
        front.append(distance)
    axes[0].plot(t,front)
    axes[0].axhline(.55,color=ORANGE,ls="--",label="tight threshold")
    axes[0].set(ylabel="Front range (m)",title="Stationary synthetic robot; obstruction clears at 3 s")
    axes[0].legend()
    axes[1].step(t,chosen,where="post")
    axes[1].set(yticks=[0,1],yticklabels=["classical","learned"],xlabel="Time (s)",ylabel="Command owner",ylim=(-.15,1.15))
    save(fig, out, "lab10_plot.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("docs/figures/teaching"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    diagrams(args.out)
    plots(args.out)
    print(f"Wrote 10 diagrams and 10 computed plots to {args.out}")


if __name__ == "__main__":
    main()

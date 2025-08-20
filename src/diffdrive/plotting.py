from __future__ import annotations
import matplotlib.pyplot as plt

def plot_path_and_traj(path, states, save: str | None = None, show: bool = True):
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    tx = [s.x for s in states]
    ty = [s.y for s in states]

    plt.figure()
    plt.plot(xs, ys, '--', label='path')
    plt.plot(tx, ty, label='trajectory')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.legend()
    if save:
        import os
        os.makedirs(os.path.dirname(save), exist_ok=True)
        plt.savefig(save, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
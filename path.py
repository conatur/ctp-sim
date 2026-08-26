import numpy as np
import matplotlib.pyplot as plt  
from scipy.interpolate import splprep, splev
from track import make_track
from centerline import midpoints, order_pts, offsets

def densify(truth, n=4000):
    d = np.linalg.norm(np.diff(truth, axis=0, append=truth[:1]), axis=1)
    s = np.concatenate([[0], d.cumsum()[:-1]])
    q = np.linspace(0, d.sum(), n, endpoint=False)
    return np.stack([np.interp(q, s, truth[:, 0]),
                     np.interp(q, s, truth[:, 1])], axis=1)

def plot_path(t, mids, ordered, path, ax=None):
    if ax is None:
        _, ax = plt.subplots(figsize=(9, 9))
    ax.scatter(*t.cones[t.color == 0].T, c='tab:blue',   s=25, zorder=3)
    ax.scatter(*t.cones[t.color == 1].T, c='tab:orange', s=25, zorder=3)
    ax.scatter(*mids.T, c='tab:red', s=10, zorder=4, label='midpoints')
    ax.plot(*ordered.T, '-', c='0.6', lw=1.0, zorder=5, label='ordered')
    ax.plot(*t.truth.T, '-', c='tab:green', lw=1.2, alpha=0.7, zorder=2, label='truth')
    ax.plot(*path.T, '-', c='tab:purple', lw=2.0, zorder=6, label='spline')
    ax.set_aspect('equal')
    ax.legend()
    return ax


def report(path, truth, label=""):
    e = offsets(path, truth)
    print(f"{label:>8}  mean {e.mean():.4f}  p95 {np.percentile(e, 95):.4f}  "
          f"max {e.max():.4f}")
    return e

def spline():
    t = make_track(1); mids = midpoints(t); ordered = order_pts(t,mids)
    closed = np.vstack([ordered, ordered[:1]])
    SMOOTH = 1.0
    tck, u = splprep([closed[:,0], closed[:,1]], s = SMOOTH, per = 1)
    xs, ys = splev(np.linspace(0, 1, 3000, endpoint=None), tck)
    path = densify(np.stack([xs, ys], axis=1), n=3000)
    """---Graph---"""
    # plot_path(t, mids, ordered, path)
    # plt.show()
    
    """---Smoothness comparison---"""
    # for smooth in [0, 1, 5, 20, 50, 100, 200]:
    #     tck, _ = splprep([closed[:, 0], closed[:, 1]], s=smooth, per=1)
    #     xs, ys = splev(np.linspace(0, 1, 3000, endpoint=False), tck)
    #     p = densify(np.stack([xs, ys], axis=1), 3000)
    #     report(p, t.truth, label=str(smooth))

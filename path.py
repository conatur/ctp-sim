import numpy as np
import matplotlib.pyplot as plt  
from scipy.interpolate import splprep, splev
from track import make_track
from centerline import midpoints, order_pts, offsets
from dataclasses import dataclass

@dataclass
class Path:
    path: np.ndarray
    s: np.ndarray
    kappa: np.ndarray
    v: np.ndarray
    length: float
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

def spline(t):
    mids = midpoints(t); ordered = order_pts(t,mids)
    closed = np.vstack([ordered, ordered[:1]])
    SMOOTH = 1.0
    u_fine = np.linspace(0, 1, 3000, endpoint=False)
    tck, _ = splprep([closed[:, 0], closed[:, 1]], s=SMOOTH, per=1)

    # evaluate on the raw u grid
    xs, ys = splev(u_fine, tck)
    dx, dy = splev(u_fine, tck, der=1)
    ddx, ddy = splev(u_fine, tck, der=2)
    k_raw = (dx*ddy - dy*ddx) / (dx**2 + dy**2)**1.5

    # arc length along the raw samples
    raw = np.stack([xs, ys], axis=1)
    d = np.linalg.norm(np.diff(raw, axis=0, append=raw[:1]), axis=1)
    s_raw = np.concatenate([[0], d.cumsum()[:-1]])
    L = d.sum()

    # resample everything onto a uniform arc-length grid
    s = np.linspace(0, L, 3000, endpoint=False)
    path = np.stack([np.interp(s, s_raw, xs), np.interp(s, s_raw, ys)], axis=1)
    k = np.interp(s, s_raw, k_raw)

    
    V_MAX = 25.0 # m/s (~90 km/h) typical FSAE electric top speed
    A_LAT = 3.0 # m/s^2 -> below typical kinematic bicycle model's 4 m/s^2 (0.4g limit)
    A_BRAKE = 6.0 # m/s^2
    A_ACCEL = 4.0 # m/s^2, weaker than braking
    v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(k), 1e-6)))
    ds = s[1]-s[0]
    n = len(v)
    for _ in range(2):
        for i in range(n-1, -1, -1):
            j = (i+1)%n
            v[i] = min(v[i], np.sqrt(v[j]**2 + 2*A_BRAKE*ds))
    for _ in range(2):
        for i in range(n):
            j = (i+1)%n
            v[j] = min(v[j], np.sqrt(v[i]**2 + 2*A_ACCEL*ds))
        
    return (Path(path, s, k, v, d.sum()))
    

if __name__ == "__main__":
    spline(make_track(0))
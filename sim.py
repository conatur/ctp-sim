import os
import numpy as np
import matplotlib.pyplot as plt  
from path import spline
from track import make_track
from centerline import offsets
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

def report(profile, t, DT):
    print(f"{len(profile)} steps, {len(profile)*DT:.1f} s")
    e = offsets(profile[:, :2], t.truth)
    dist = np.cumsum(profile[:, 3] * DT)
    fig, ax = plt.subplots(figsize=(11,4))
    ax.plot(dist, e)
    ax.set_ylim(0, 2)
    ax.set_xlabel('distance travelled (m)')
    ax.set_ylabel('cross-track error (m)')
    ax.axhline(1.75, c='r', ls='--', label='track half-width')
    ax.axhline(e.mean(), c='0.6', ls=':', label=f'mean {e.mean():.2f} m')
    ax.legend()
    os.makedirs('figures', exist_ok=True)
    fig.savefig('figures/cross_track_error.png', dpi=150, bbox_inches='tight')
    print(f"cross-track: mean {e.mean():.3f}  p95 {np.percentile(e,95):.3f}  max {e.max():.3f}")

def plot(profile, p):
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(*p.path.T, '-', c='0.7', lw=1.5, label='path')
    ax.plot(profile[:, 0], profile[:, 1], '-', c='tab:red', lw=1.2, label='driven')
    ax.set_aspect('equal'); ax.legend()
    plt.show()
    

def sim(t):
    WHEELBASE = 1.5
    DT = 0.02
    MAX_STEER = np.radians(25)
    LOOKAHEAD_K = 0.4
    LOOKAHEAD_MIN = 3.0
    A_BRAKE = 6.0
    A_ACCEL = 4.0
    K_P = 1.0
    p = spline(t)
    x, y = p.path[0]
    dx, dy = p.path[10]-p.path[0]
    theta = np.arctan2(dy, dx)
    v = 0.0
    ds = p.length/len(p.path)
    target_idx = 0
    dist = 0
    profile = []
    while dist<=p.length:
        Ld = LOOKAHEAD_K*v + LOOKAHEAD_MIN
        for _ in range(len(p.path)):
            if np.linalg.norm(p.path[target_idx]-np.array([x,y])) >= Ld:
                break
            target_idx = (target_idx + 1) % len(p.path)
            
        dy = p.path[target_idx][1] - y
        dx = p.path[target_idx][0] - x
        a = np.arctan2(dy, dx) - theta
        alpha = np.arctan2(np.sin(a), np.cos(a))
        delta = np.clip(np.arctan2(2 * WHEELBASE * np.sin(alpha), Ld), -MAX_STEER, MAX_STEER)
        nearest_idx = (target_idx - int(Ld / ds)) % len(p.path)

        accel = np.clip(K_P * (p.v[nearest_idx] - v), -A_BRAKE, A_ACCEL)
        x += v*np.cos(theta)*DT; y += v*np.sin(theta)*DT
        theta += (v/WHEELBASE) * np.tan(delta) * DT
        v += accel*DT
        dist += v*DT
        profile.append([x,y,theta,v,delta])

    profile = np.array(profile)
    
    return profile

CAR = np.array([[ 1.7,  0.00], [ 1.3,  0.35], [ 0.9,  0.35],
                [ 0.9,  0.80], [ 0.4,  0.80], [ 0.4,  0.35],
                [-0.6,  0.35], [-0.6,  0.80], [-1.1,  0.80],
                [-1.1,  0.35], [-1.6,  0.35], [-1.6,  0.55],
                [-1.9,  0.55], [-1.9, -0.55], [-1.6, -0.55],
                [-1.6, -0.35], [-1.1, -0.35], [-1.1, -0.80],
                [-0.6, -0.80], [-0.6, -0.35], [ 0.4, -0.35],
                [ 0.4, -0.80], [ 0.9, -0.80], [ 0.9, -0.35],
                [ 1.3, -0.35]])

def animate(t, p, profile, out='figures/lap.gif', stride=5, dt=0.02):
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(*p.path.T, '-', c='0.75', lw=1.2, zorder=1, label='planned path')
    ax.scatter(*t.cones[t.color == 0].T, c='tab:blue',   s=18, zorder=2,
               label='inner cones')
    ax.scatter(*t.cones[t.color == 1].T, c='tab:orange', s=18, zorder=2,
               label='outer cones')

    trail, = ax.plot([], [], '-', c='tab:red', lw=1.5, zorder=3, label='driven')
    body = Polygon(np.zeros((len(CAR), 2)), closed=True, fc='k', ec='none',
                   zorder=4, label='car')
    ax.add_patch(body)
    label = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top',
                    fontfamily='monospace', fontsize=11)

    ax.set_aspect('equal')
    ax.set_xlim(t.cones[:, 0].min() - 5, t.cones[:, 0].max() + 5)
    ax.set_ylim(t.cones[:, 1].min() - 5, t.cones[:, 1].max() + 5)
    ax.axis('off')
    ax.legend(loc='lower right', bbox_to_anchor=(1.2, -0.08),
          framealpha=0.9, fontsize=9)

    def update(f):
        x, y, th = profile[f, 0], profile[f, 1], profile[f, 2]
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s], [s, c]])
        body.set_xy(CAR @ R.T + np.array([x, y]))
        trail.set_data(profile[:f, 0], profile[:f, 1])
        label.set_text(f"{f * dt:5.1f} s\n{profile[f, 3]:5.1f} m/s")
        return trail, body, label

    ani = FuncAnimation(fig, update, frames=range(0, len(profile), stride),
                        interval=40, blit=True)
    ani.save(out, writer=PillowWriter(fps=25))
    plt.close(fig)
    print("wrote", out)
    return ani

if __name__ == "__main__":
    seed = 3 # CHANGE SEED TO ANY NUMBER YOU WANT AND PRESS RUN, ANIMATION WILL APPEAR IN FIGURES FOLDER AS A GIF (usually takes up to 30 seconds for the animation)
    t = make_track(seed)
    animate(t, spline(t), sim(t), f'figures/lap{seed}.gif')
    report(sim(t), t, DT=0.02)
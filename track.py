import numpy as np
import matplotlib.pyplot as plt  
from dataclasses import dataclass


@dataclass
class Track:
    cones: np.ndarray
    color: np.ndarray
    truth: np.ndarray
    width: float
    length: float

def save(track, path):
    np.savez(path, cones = track.cones, color = track.color, truth = track.truth, width = track.width, length = track.length)

def load(path):
    z = np.load(path)
    return Track(z['cones'], z['color'], z['truth'], z['width'], z['length'])

def make_track(seed: int, radius: float = 40.0, width: float = 3.5, spacing: float = 5.0, noise_std: float = 0.05, dropout: float = 0.07, harmonics: tuple = ((2, 7.0), (3, 4.0), (5, 2.5))) -> Track:
    #start with creating a circle with 2000 points in polar form
    rng = np.random.default_rng(seed)
    th = np.linspace(0, 2*np.pi, 2000, endpoint=False)
    r = np.full_like(th, radius)
    # add deformations and alternations to the circle with a sinusoidal wave equation 
    for k, amp in harmonics:
        phase = rng.uniform(0, 2*np.pi)
        r += amp * np.sin(th*k + phase)
    cx, cy = r*np.cos(th), r*np.sin(th)
    #resample based on even spacing between points to make cones at every 5 meters
    d = np.hypot(np.diff(cx, append=cx[0]), np.diff(cy, append=cy[0]))
    L = d.sum()
    s = np.concatenate([[0], d.cumsum()[:-1]])
    targets = np.arange(0, L, spacing)
    #interpolate on points at even spacing; for example, x and y at distance 5.0, 10.0, 15.0 meters along the track
    xs, ys = np.interp(targets, s, cx),  np.interp(targets, s, cy)
    # find unit vectors to place cones normal to the center line
    tx, ty = np.gradient(xs), np.gradient(ys)
    mag = np.hypot(tx, ty)
    tx /= mag; ty /= mag
    nx, ny = -ty, tx
    #the derivative here is actually negative when angle is increasing  
    inner = np.stack([xs + nx*width/2, ys + ny*width/2], axis=1)
    outer = np.stack([xs - nx*width/2, ys - ny*width/2], axis=1)

    # display the track so far ->
    # fig, ax = plt.subplots(figsize=(8,8))
    # ax.scatter(inner[:, 0], inner[:, 1], c='tab:blue', s=20, label='inner')
    # ax.scatter(outer[:, 0], outer[:, 1], c='tab:orange', s=20, label='outer')
    # ax.plot(xs, ys, 'k--', lw=0.8, alpha=0.5, label='centerline')
    # ax.set_aspect('equal')
    # ax.legend()
    # plt.show()

    inner += rng.normal(0, noise_std, inner.shape)
    outer += rng.normal(0, noise_std, outer.shape) 
    inner = inner[rng.random(len(inner)) > dropout]
    outer = outer[rng.random(len(outer)) > dropout]

    cones = np.vstack([inner, outer]); color = np.concatenate([np.zeros(len(inner), dtype=np.int8), np.ones(len(outer), dtype=np.int8)])
    perm = rng.permutation(len(cones))
    color = color[perm]
    cones = cones[perm]

    return Track(cones = cones, color = color, truth = np.stack([cx, cy], axis=1), width = width, length = L)

if __name__ == "__main__":
    t = make_track(5)
    
    print(len(t.cones), "cones", t.length, "m lap")

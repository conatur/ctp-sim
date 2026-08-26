from scipy.spatial import Delaunay, KDTree
import numpy as np
import matplotlib.pyplot as plt
from track import make_track


def cross_edges(t):
    tri = Delaunay(t.cones)
    edges = set()
    for a, b, c in tri.simplices:
        edges.add((min(a, b), max(a, b)))
        edges.add((min(a, c), max(a, c)))
        edges.add((min(b, c), max(b, c)))
    E = np.array(sorted(edges))
    return E[t.color[E[:, 0]] != t.color[E[:, 1]]]

def order_pts(t, mids, radius=8.0):
    tree = KDTree(mids)
    pos = t.truth[0]
    direction = t.truth[20] - t.truth[0]
    direction /= np.linalg.norm(direction)

    path, visited = [], set()
    while True:
        cand = np.array(tree.query_ball_point(pos, r=radius), dtype=int)
        cand = cand[[i not in visited for i in cand]] if cand.size else cand # only get unvisited points
        if cand.size == 0:
            break

        vecs = mids[cand] - pos
        cand = cand[vecs @ direction > 0] # if angle between direction previous and new direction is obtuse, avoids going backward
        if cand.size == 0:
            break

        d = np.linalg.norm(mids[cand] - pos, axis=1) # distance between candidates and current pos
        idx = cand[d.argmin()] # smallest distance

        direction = mids[idx] - pos
        direction /= np.linalg.norm(direction)
        pos = mids[idx]
        visited.add(idx)
        path.append(idx)

    return mids[path]



def densify(truth, n=4000):
    d = np.linalg.norm(np.diff(truth, axis=0, append=truth[:1]), axis=1)
    s = np.concatenate([[0], d.cumsum()[:-1]])
    q = np.linspace(0, d.sum(), n, endpoint=False)
    return np.stack([np.interp(q, s, truth[:, 0]),
                     np.interp(q, s, truth[:, 1])], axis=1)


def offsets(mids, truth):
    return KDTree(truth).query(mids)[0]


def plot_track(t, E, mids):
    fig, ax = plt.subplots(figsize=(9, 9))
    for i, j in E:
        ax.plot([t.cones[i, 0], t.cones[j, 0]],
                [t.cones[i, 1], t.cones[j, 1]], c='0.8', lw=0.7, zorder=1)
    ax.scatter(*t.cones[t.color == 0].T, c='tab:blue',   s=25, zorder=3)
    ax.scatter(*t.cones[t.color == 1].T, c='tab:orange', s=25, zorder=3)
    ax.scatter(*mids.T, c='tab:red', s=12, zorder=4, label='midpoints')
    closed = np.vstack([t.truth, t.truth[:1]])
    #ax.plot(*closed.T, "--", c='tab:green', lw=1.6, zorder=2, label='truth')
    ax.scatter(*t.truth.T, c='tab:green', s=15, marker='x', zorder=2, label='truth')
    ax.set_aspect('equal')
    ax.legend()
    plt.show()


if __name__ == "__main__":
    print(f"{'seed':>4} {'n':>4} {'mean':>7} {'p95':>7} {'max':>7}   cut")
    for seed in range(50):
        t = make_track(seed)
        E = cross_edges(t)
        MAX_EDGE = 1.3 * np.hypot(5.0, t.width)
        lens = np.linalg.norm(t.cones[E[:, 0]] - t.cones[E[:, 1]], axis=1)
        mids = (t.cones[E[:, 0]] + t.cones[E[:, 1]]) / 2
        mask = lens <= MAX_EDGE
       
        for name, m in [("all", mids), ("cut", mids[mask])]:
            o = offsets(m, t.truth)
            tag = f"{seed:>4}" if name == "all" else "    "
            print(f"{tag} {len(m):>4} {o.mean():>7.3f} "
                  f"{np.percentile(o, 95):>7.3f} {o.max():>7.3f}   {name}")

        if mask.sum() < len(mask):
            print(f"     removed offsets: {np.round(offsets(mids[~mask], t.truth), 2)}")
 
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

def order_pts(t, mids, radius=20.0):
    tree = KDTree(mids)
    pos = t.truth[0]
    direction = t.truth[20] - t.truth[0]
    direction /= np.linalg.norm(direction)

    path, visited = [], set()
    while True:
        cand = np.array(tree.query_ball_point(pos, r=radius), dtype=int)
        MIN_STEP = 0.5
        if cand.size == 0:
            break
        cand = cand[[i not in visited for i in cand]] if cand.size else cand # only get unvisited points
        if cand.size == 0:
            break

        v = mids[cand] - pos
        dist = np.linalg.norm(v, axis=1)
        ok = (dist > MIN_STEP) & (v @ direction > 0)
        if not ok.any():
            break

        cand, dist = cand[ok], dist[ok]
        idx = cand[dist.argmin()]

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


def plot_track(t, E, mids, ordered):
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
    ax.plot(*ordered.T, '-', c='tab:purple', lw=2, zorder=5, label='greedy')
    ax.set_aspect('equal')
    ax.legend()
    plt.show()


if __name__ == "__main__":
    max_gap = 0
    count = 0
    print(f"{'seed':>4} {'n':>4} {'mean':>7} {'p95':>7} {'max':>7}   cut")
    for seed in range(6):
        t = make_track(seed)
        E = cross_edges(t)
        MAX_EDGE = 1.3 * np.hypot(5.0, t.width)
        lens = np.linalg.norm(t.cones[E[:, 0]] - t.cones[E[:, 1]], axis=1)
        mids = (t.cones[E[:, 0]] + t.cones[E[:, 1]]) / 2
        mask = lens <= MAX_EDGE
       
        # for name, m in [("all", mids), ("cut", mids[mask])]:
        #     o = offsets(m, t.truth)
        #     tag = f"{seed:>4}" if name == "all" else "    "
        #     print(f"{tag} {len(m):>4} {o.mean():>7.3f} "
        #           f"{np.percentile(o, 95):>7.3f} {o.max():>7.3f}   {name}")

        # if mask.sum() < len(mask):
        #     print(f"     removed offsets: {np.round(offsets(mids[~mask], t.truth), 2)}")
        
        ordered = order_pts(t, mids)
        gaps = np.linalg.norm(np.diff(ordered, axis=0), axis=1)
        # print(len(ordered), "of", len(mids))
        # print(f"gaps: median {np.median(gaps):.2f}  max {gaps.max():.2f}")
        # print(f"closed: {np.linalg.norm(ordered[-1] - ordered[0]) < 8.0}")
        # print(f"length ratio: {gaps.sum() / t.length:.3f}")
        if np.linalg.norm(ordered[-1] - ordered[0]) > 20.0:
            #print(ordered[-1], ordered[0])
            count += 1
        max_gap = max(max_gap, gaps.max())
        plot_track(t, E, mids, ordered)
    print(f"max gap: {max_gap}")
    print(f"unclosed count: {count}")
    
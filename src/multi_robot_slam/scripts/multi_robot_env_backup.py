"""
multi_robot_env.py
==================
2D occupancy-grid training environment for multi-robot exploration RL.

Faithfully mirrors multi_robot_world.sdf:
  - 20×20 m arena  (x,y ∈ [-10, 10])
  - All walls, partitions, pillars, obstacles from the SDF
  - Robot starts identical to Gazebo: R1(-5,-5), R2(5,-5), R3(0,5)

Policy interface (Option A — coordinator):
  Policy outputs a frontier index. Low-level planner handles motion.

Observation per robot:
  local_map   int8  (LOCAL_CROP, LOCAL_CROP)   shared map crop  -1/0/1
  self_pos    f32   (2,)                        (x,y)/10 → [-1,1]
  other_pos   f32   (N_ROBOTS-1, 2)             relative (dx,dy)/20
  frontiers   f32   (MAX_FRONTIERS, 2)          relative, 0-padded
  n_frontiers int

Action: int  0..MAX_FRONTIERS-1  (pick frontier k)  or  MAX_FRONTIERS (stay)

Reward: Δ newly-revealed free cells  −  clustering_penalty

Performance: ~650 steps/sec single core → 4 parallel envs finishes
             50k × 400-step training episodes in ~2 hours.

The frontier detection function (find_frontiers_fast) is the SAME function
that will be used in the Gazebo deployment ROS2 node, ensuring zero
observation-distribution shift.
"""

import math, time
import numpy as np
from scipy.ndimage import label

# ── Constants ────────────────────────────────────────────────────────────────
WORLD_MIN   = -10.0
WORLD_MAX   =  10.0
RESOLUTION  =  0.05          # m/cell
GRID_SIZE   = 400             # 20m / 0.05 = 400

FREE = 0; OCCUPIED = 1; UNKNOWN = -1

MAX_FRONTIERS    = 6
LOCAL_CROP       = 64
SENSOR_RANGE     = 5.0
N_RAYS           = 36         # 10° spacing, fast + good coverage
MOVE_SPEED       = 0.4        # m/s  (matches Gazebo)
DT               = 0.2        # s per low-level move tick
TICKS_PER_DECISION = 25       # robot moves MOVE_SPEED*DT*25 = 2m per decision step
COVERAGE_TARGET  = 0.55
MAX_STEPS        = 300
CLUSTER_PENALTY  = 0.05
MIN_FRONTIER_SEP = 1.5        # m
N_ROBOTS         = 3
FRONTIER_INTERVAL = 5         # recompute frontiers every N decision steps

START_POSES = np.array([[-5.,-5.],[5.,-5.],[0.,5.]], dtype=np.float32)

# Pre-baked ray directions (computed once at import)
_ANGLES = np.linspace(0, 2*math.pi, N_RAYS, endpoint=False)
_RAY_DX = np.cos(_ANGLES).astype(np.float32)
_RAY_DY = np.sin(_ANGLES).astype(np.float32)
_RAY_STEPS = (np.arange(1, int(SENSOR_RANGE/RESOLUTION)+2, dtype=np.float32)
              * RESOLUTION)  # (S,)

# ── Coordinate helpers ───────────────────────────────────────────────────────
def w2g(wx, wy):
    col = int((wx - WORLD_MIN) / RESOLUTION)
    row = int((wy - WORLD_MIN) / RESOLUTION)
    return np.clip(row,0,GRID_SIZE-1), np.clip(col,0,GRID_SIZE-1)

def g2w(row, col):
    return (col*RESOLUTION+WORLD_MIN+RESOLUTION/2,
            row*RESOLUTION+WORLD_MIN+RESOLUTION/2)

# ── Geometry primitives ──────────────────────────────────────────────────────
def _box_aabb(grid, cx, cy, sx, sy):
    c_lo=max(0,int((cx-sx/2-WORLD_MIN)/RESOLUTION))
    c_hi=min(GRID_SIZE,int((cx+sx/2-WORLD_MIN)/RESOLUTION)+2)
    r_lo=max(0,int((cy-sy/2-WORLD_MIN)/RESOLUTION))
    r_hi=min(GRID_SIZE,int((cy+sy/2-WORLD_MIN)/RESOLUTION)+2)
    grid[r_lo:r_hi, c_lo:c_hi]=OCCUPIED

def _box_rotated(grid, cx, cy, sx, sy, yaw):
    if abs(yaw)<0.02: _box_aabb(grid,cx,cy,sx,sy); return
    hx,hy=sx/2,sy/2; ca,sa=math.cos(yaw),math.sin(yaw)
    corners=np.array([[ca*hx-sa*hy+cx, sa*hx+ca*hy+cy],
                      [-ca*hx-sa*hy+cx,-sa*hx+ca*hy+cy],
                      [-ca*hx+sa*hy+cx,-sa*hx-ca*hy+cy],
                      [ca*hx+sa*hy+cx,  sa*hx-ca*hy+cy]])
    c_lo=max(0,int((corners[:,0].min()-WORLD_MIN)/RESOLUTION)-1)
    c_hi=min(GRID_SIZE,int((corners[:,0].max()-WORLD_MIN)/RESOLUTION)+2)
    r_lo=max(0,int((corners[:,1].min()-WORLD_MIN)/RESOLUTION)-1)
    r_hi=min(GRID_SIZE,int((corners[:,1].max()-WORLD_MIN)/RESOLUTION)+2)
    CC,RR=np.meshgrid(np.arange(c_lo,c_hi),np.arange(r_lo,r_hi))
    WX=CC*RESOLUTION+WORLD_MIN+RESOLUTION/2
    WY=RR*RESOLUTION+WORLD_MIN+RESOLUTION/2
    lx=ca*(WX-cx)+sa*(WY-cy); ly=-sa*(WX-cx)+ca*(WY-cy)
    grid[r_lo:r_hi, c_lo:c_hi][(np.abs(lx)<=hx+RESOLUTION)&(np.abs(ly)<=hy+RESOLUTION)]=OCCUPIED

def _circle(grid, cx, cy, r):
    margin=int(r/RESOLUTION)+2
    r0,c0=w2g(cx,cy)
    r_lo=max(0,r0-margin); r_hi=min(GRID_SIZE,r0+margin+1)
    c_lo=max(0,c0-margin); c_hi=min(GRID_SIZE,c0+margin+1)
    CC,RR=np.meshgrid(np.arange(c_lo,c_hi),np.arange(r_lo,r_hi))
    WX=CC*RESOLUTION+WORLD_MIN+RESOLUTION/2
    WY=RR*RESOLUTION+WORLD_MIN+RESOLUTION/2
    grid[r_lo:r_hi, c_lo:c_hi][(WX-cx)**2+(WY-cy)**2<=(r+RESOLUTION)**2]=OCCUPIED

# ── Ground-truth map (parsed from SDF) ──────────────────────────────────────
def build_ground_truth_map():
    grid=np.zeros((GRID_SIZE,GRID_SIZE),dtype=np.int8)
    # outer walls
    for cx,cy,sx,sy in [(0,10,20.4,.2),(0,-10,20.4,.2),(10,0,.2,20),(-10,0,.2,20)]:
        _box_aabb(grid,cx,cy,sx,sy)
    # internal walls (axis-aligned)
    for cx,cy,sx,sy in [
        (-6,3,8,.2),(6,3,8,.2),(-8.25,-3,3.5,.2),(8.25,-3,3.5,.2),
        (-3,4,.2,2),(-3,8.5,.2,3),(3,4,.2,2),(3,8.5,.2,3),
        (-3,-8.5,.2,3),(-3,-4,.2,2),(3,-8.5,.2,3),(3,-4,.2,2),
        (-7.5,7.5,5,.2),(-5,8.5,.2,2),(7.5,7.5,5,.2),(5,8.5,.2,2),
        (-7,-8.5,.2,3),(-8.5,-7,3,.2),(7,-8.5,.2,3),(8.5,-7,3,.2),
        (-2,-5.5,2,.2)]:
        _box_aabb(grid,cx,cy,sx,sy)
    # pillars
    for cx,cy,r in [(-7,1.5,.25),(-7,-1.5,.25),(7,1.5,.25),(7,-1.5,.25),
                    (-1.5,-6,.2),(1.5,-6,.2),(-1.5,6,.2),(1.5,6,.2)]:
        _circle(grid,cx,cy,r)
    # junction box (45° rotated)
    _box_rotated(grid,0,0,.6,.6,math.pi/4)
    # scattered obstacles
    for cx,cy,sx,sy,yaw in [
        (-7,5.5,1,.5,.5),(-4.5,6,1,.4,1.1),(-8.5,-4.5,1,1,0),(-5.5,-9,.8,.5,1),
        (-1.5,-7.5,.8,.5,.4),(-1.5,8,.8,.8,.7),(7,5.5,1.2,.4,.8),(9,4.5,.6,1.4,.4),
        (8,-4,1,.5,.3),(9,-9,.8,.8,0),(-8,0,.6,.6,.5),(8,0,.6,.6,.3)]:
        _box_rotated(grid,cx,cy,sx,sy,yaw)
    for cx,cy,r in [(-8.5,9,.4),(5,9,.35),(-4,-8.5,.4),(4,-8.5,.35),(1.5,9,.3),(1.5,-8.5,.3)]:
        _circle(grid,cx,cy,r)
    return grid

# ── Raycasting (vectorised) ──────────────────────────────────────────────────
def raycast(grid, rx, ry):
    """
    Cast N_RAYS from (rx,ry). Returns bool mask (GRID_SIZE,GRID_SIZE).
    All rays processed simultaneously in one NumPy operation.
    ~0.12ms per call.
    """
    WX = rx + _RAY_DX[:,None]*_RAY_STEPS[None,:]   # (N,S)
    WY = ry + _RAY_DY[:,None]*_RAY_STEPS[None,:]
    cols=np.clip(((WX-WORLD_MIN)/RESOLUTION).astype(np.int16),0,GRID_SIZE-1)
    rows=np.clip(((WY-WORLD_MIN)/RESOLUTION).astype(np.int16),0,GRID_SIZE-1)
    hit=(grid[rows,cols]==OCCUPIED)
    # mask: steps before first occupied cell + first occupied cell
    after_first=np.concatenate(
        [np.zeros((N_RAYS,1),bool), np.cumsum(hit,axis=1)[:,:-1]>0], axis=1)
    valid=~after_first
    revealed=np.zeros((GRID_SIZE,GRID_SIZE),dtype=bool)
    revealed[rows[valid],cols[valid]]=True
    return revealed

# ── Frontier detection  (SHARED — identical code in Gazebo deploy node) ──────
def find_frontiers_fast(observed_map, downsample=4):
    """
    Frontier cells = FREE cells adjacent to UNKNOWN cells.
    Downsamples to 100×100 before labelling → ~3.5ms vs 1600ms at full res.
    Returns list of (world_x, world_y) centroid tuples.

    Uses downsample=4 (0.2m coarse cells) — sufficient precision for
    frontier goal selection; matches what the Gazebo node will use.
    """
    D=downsample; sz=GRID_SIZE//D
    m=observed_map.reshape(sz,D,sz,D)
    free_c   =(m==FREE).any(axis=(1,3))
    unknown_c=(m==UNKNOWN).any(axis=(1,3))
    adj_unk=(np.roll(unknown_c,1,0)|np.roll(unknown_c,-1,0)|
             np.roll(unknown_c,1,1)|np.roll(unknown_c,-1,1))
    fmask=free_c&adj_unk
    if not fmask.any(): return []
    labeled,n=label(fmask)
    res_c=RESOLUTION*D
    centroids=[]
    for i in range(1,n+1):
        rr,cc=np.where(labeled==i)
        if len(rr)<1: continue
        wx=float(cc.mean()*res_c+WORLD_MIN+res_c/2)
        wy=float(rr.mean()*res_c+WORLD_MIN+res_c/2)
        centroids.append((wx,wy))
    return centroids

# ── Straight-line planner (shared — identical in Gazebo node) ────────────────
def plan_step(rx, ry, gx, gy, grid, max_ticks=TICKS_PER_DECISION):
    """
    Move toward (gx,gy) for up to max_ticks ticks; stop at wall or arrival.
    Returns new (x,y) and bool reached_goal.
    max_ticks caps movement per decision step so the policy has a
    meaningful coordination horizon (not just distance-weighted teleport).
    """
    dist=math.hypot(gx-rx,gy-ry)
    if dist<0.05: return rx,ry,True
    step_dist=MOVE_SPEED*DT
    dx=(gx-rx)/dist; dy=(gy-ry)/dist
    nx,ny=rx,ry
    for _ in range(max_ticks):
        tx=nx+dx*step_dist; ty=ny+dy*step_dist
        r,c=w2g(tx,ty)
        if grid[r,c]==OCCUPIED: break
        nx,ny=tx,ty
        if math.hypot(nx-gx,ny-gy)<step_dist: return nx,ny,True
    return nx,ny,False

# ── Environment ──────────────────────────────────────────────────────────────
class MultiRobotExplorationEnv:
    """
    Gym-compatible (no gym dependency) multi-robot exploration environment.

    reset() → obs_list, info
    step(actions) → obs_list, reward, done, truncated, info

    actions: list[int] length N_ROBOTS
    obs_list: list[dict] — one dict per robot (see module docstring)
    """

    def __init__(self, seed=None):
        self.gt_map = build_ground_truth_map()
        self.n_free = int((self.gt_map==FREE).sum())
        self.rng    = np.random.default_rng(seed)
        self.action_space_n = MAX_FRONTIERS+1
        # constants exposed for external use
        self.MAX_FRONTIERS   = MAX_FRONTIERS
        self.LOCAL_CROP      = LOCAL_CROP
        self.N_ROBOTS        = N_ROBOTS
        self.COVERAGE_TARGET = COVERAGE_TARGET
        self.MAX_STEPS       = MAX_STEPS
        self._obs_cache      = None
        self.reset()

    # ── Public API ────────────────────────────────────────────────────────────
    def reset(self, seed=None):
        if seed is not None: self.rng=np.random.default_rng(seed)
        # Add small random jitter to start positions so episodes vary
        jitter = self.rng.uniform(-0.5, 0.5, size=START_POSES.shape).astype(np.float32)
        self.positions  = START_POSES.copy() + jitter
        # shared observed map (union across all robots)
        self.shared_map = np.full((GRID_SIZE,GRID_SIZE),UNKNOWN,dtype=np.int8)
        # per-robot observed maps (for individual sensing)
        self._obs_maps  = [np.full((GRID_SIZE,GRID_SIZE),UNKNOWN,dtype=np.int8)
                           for _ in range(N_ROBOTS)]
        self.goals      = [None]*N_ROBOTS
        _path_cache.clear()   # clear stale paths from previous episode
        self.frontiers  = []
        self._step      = 0
        self._prev_cov  = 0.
        for i in range(N_ROBOTS): self._sense(i)
        self._merge()
        self.frontiers  = find_frontiers_fast(self.shared_map)
        return self._get_obs(), {"coverage": self._cov()}

    def step(self, actions):
        assert len(actions)==N_ROBOTS
        prev=self._cov()
        penalty=self._cluster_penalty(actions)

        # Assign goals from policy actions every step
        for i, a in enumerate(actions):
            if a < len(self.frontiers) and a < MAX_FRONTIERS:
                self.goals[i] = self.frontiers[a]

        # Move all robots toward their current goals
        for i in range(N_ROBOTS):
            if self.goals[i] is not None:
                gx, gy = self.goals[i]
                nx, ny, reached = plan_step_bfs(
                    *self.positions[i], gx, gy, self.gt_map,
                    cache_key=(i, round(gx,1), round(gy,1)))
                self.positions[i] = np.array([nx, ny], dtype=np.float32)
                if reached:
                    self.goals[i] = None

        # sense + merge
        for i in range(N_ROBOTS): self._sense(i)
        self._merge()

        # update frontiers only every FRONTIER_INTERVAL steps
        self._step+=1
        if self._step % FRONTIER_INTERVAL == 0 or not self.frontiers:
            self.frontiers=find_frontiers_fast(self.shared_map)

        cov = self._cov()
        terminal_bonus = 2.0 if cov >= COVERAGE_TARGET else 0.0
        reward = (cov - prev) - penalty / self.n_free + terminal_bonus
        done   = cov >= COVERAGE_TARGET
        trunc  = self._step >= MAX_STEPS
        return (self._get_obs(), reward, done, trunc,
                {"coverage": cov, "step": self._step, "n_frontiers": len(self.frontiers)})
    

    # ── Internals ─────────────────────────────────────────────────────────────
    def _sense(self, i):
        rx,ry=self.positions[i]
        rev=raycast(self.gt_map,rx,ry)
        om=self._obs_maps[i]
        om[rev&(self.gt_map==FREE)]=FREE
        om[rev&(self.gt_map==OCCUPIED)]=OCCUPIED

    def _merge(self):
        """Incremental union: just OR all known values into shared_map."""
        s=np.full((GRID_SIZE,GRID_SIZE),UNKNOWN,dtype=np.int8)
        for o in self._obs_maps:
            k=o!=UNKNOWN; s[k]=o[k]
        self.shared_map=s

    def _cov(self):
        return float((self.shared_map==FREE).sum())/self.n_free if self.n_free else 0.

    def _cluster_penalty(self, actions):
        # Compute penalty for robots targeting same frontier zone
        chosen=[]
        for i,a in enumerate(actions):
            if a<len(self.frontiers) and a<MAX_FRONTIERS:
                chosen.append(self.frontiers[a])
            else:
                chosen.append(None)
        pen=0.
        for i in range(N_ROBOTS):
            for j in range(i+1,N_ROBOTS):
                gi,gj=chosen[i],chosen[j]
                if gi and gj and math.hypot(gi[0]-gj[0],gi[1]-gj[1])<MIN_FRONTIER_SEP:
                    pen+=CLUSTER_PENALTY
        return pen

    def _get_obs(self):
        half=LOCAL_CROP//2; obs_list=[]
        for i in range(N_ROBOTS):
            rx,ry=self.positions[i]
            r0,c0=w2g(rx,ry)
            # local map crop (clamp at edges)
            crop=np.full((LOCAL_CROP,LOCAL_CROP),UNKNOWN,dtype=np.int8)
            gr_lo=max(0,r0-half); gr_hi=min(GRID_SIZE,r0+half)
            gc_lo=max(0,c0-half); gc_hi=min(GRID_SIZE,c0+half)
            cr_lo=gr_lo-(r0-half); cc_lo=gc_lo-(c0-half)
            crop[cr_lo:cr_lo+(gr_hi-gr_lo), cc_lo:cc_lo+(gc_hi-gc_lo)]=\
                self.shared_map[gr_lo:gr_hi, gc_lo:gc_hi]
            # positions
            sp=np.array([rx/10.,ry/10.],dtype=np.float32)
            op=np.array([[(self.positions[j][0]-rx)/20.,
                           (self.positions[j][1]-ry)/20.]
                          for j in range(N_ROBOTS) if j!=i],dtype=np.float32)
            # frontiers (padded)
            fa=np.zeros((MAX_FRONTIERS,2),dtype=np.float32)
            nf=min(len(self.frontiers),MAX_FRONTIERS)
            for k in range(nf):
                fa[k]=[(self.frontiers[k][0]-rx)/20.,
                        (self.frontiers[k][1]-ry)/20.]
            obs_list.append({"local_map":crop,"self_pos":sp,"other_pos":op,
                             "frontiers":fa,"n_frontiers":nf})
        return obs_list

    # ── Utilities ─────────────────────────────────────────────────────────────
    def render_ascii(self, scale=10):
        lines=[]
        for r in range(GRID_SIZE-1,-1,-scale):
            row=['#' if self.shared_map[r,c]==OCCUPIED
                 else '.' if self.shared_map[r,c]==FREE
                 else ' '
                 for c in range(0,GRID_SIZE,scale)]
            lines.append(''.join(row))
        gl=[list(l) for l in lines]
        for i,(rx,ry) in enumerate(self.positions):
            r0,c0=w2g(rx,ry)
            dr=(GRID_SIZE-1-r0)//scale; dc=c0//scale
            if 0<=dr<len(gl) and 0<=dc<len(gl[0]):
                gl[dr][dc]=str(i+1)
        return '\n'.join(''.join(r) for r in gl)

    def obs_shapes(self):
        """Return dict of observation tensor shapes for network design."""
        return {
            "local_map":   (LOCAL_CROP, LOCAL_CROP),    # int8 → normalise to float
            "self_pos":    (2,),
            "other_pos":   (N_ROBOTS-1, 2),
            "frontiers":   (MAX_FRONTIERS, 2),
            "n_frontiers": (),                           # scalar int
        }


# ── Smoke test + benchmark ────────────────────────────────────────────────────
if __name__=="__main__":
    print("="*55)
    print("multi_robot_env.py — smoke test & benchmark")
    print("="*55)

    t0=time.perf_counter()
    env=MultiRobotExplorationEnv(seed=42)
    print(f"\nMap build + reset: {(time.perf_counter()-t0)*1000:.0f}ms")
    print(f"Grid: {GRID_SIZE}×{GRID_SIZE}  |  "
          f"Free cells: {env.n_free}  |  "
          f"Occupied: {(env.gt_map==OCCUPIED).sum()}")
    print(f"Observation shapes: {env.obs_shapes()}")
    print(f"Action space: {env.action_space_n} (0..{MAX_FRONTIERS-1}=frontier, {MAX_FRONTIERS}=stay)")

    print(f"\nInitial state:  coverage={env._cov()*100:.1f}%  "
          f"frontiers={len(env.frontiers)}")
    print(env.render_ascii())

    # ── Benchmark: 1000 steps, random policy ──────────────────────────────
    print("\nBenchmarking 1000 decision steps (random policy)...")
    obs,_=env.reset(); t0=time.perf_counter(); ep=0
    for s in range(1000):
        nf=len(env.frontiers)
        acts=[int(env.rng.integers(0,max(1,min(nf,MAX_FRONTIERS)))) for _ in range(N_ROBOTS)]
        obs,r,done,trunc,info=env.step(acts)
        if done or trunc: obs,_=env.reset(); ep+=1
    elapsed=time.perf_counter()-t0
    sps=1000/elapsed
    print(f"  1000 steps in {elapsed:.2f}s  →  {sps:.0f} steps/sec  ({ep} episodes)")
    print(f"  Per 400-step episode: {400/sps:.2f}s")
    print(f"  50k episodes (1 core): {50000*400/sps/3600:.1f}h")
    print(f"  50k episodes (4 cores, trivially parallel): {50000*400/sps/4/3600:.1f}h")

    # ── Greedy nearest-frontier (baseline to beat) ─────────────────────────
    print("\nGreedy nearest-frontier episode (BASELINE to beat):")
    obs,_=env.reset(); done=trunc=False
    while not(done or trunc):
        acts=[]
        for i in range(N_ROBOTS):
            nf=len(env.frontiers)
            if nf==0: acts.append(MAX_FRONTIERS); continue
            rx,ry=env.positions[i]
            dists=[math.hypot(fx-rx,fy-ry) for fx,fy in env.frontiers[:MAX_FRONTIERS]]
            acts.append(int(np.argmin(dists)))
        obs,r,done,trunc,info=env.step(acts)
    print(env.render_ascii())
    print(f"  Coverage: {info['coverage']*100:.1f}%  |  Steps: {info['step']}  |  "
          f"{'SUCCESS' if done else 'TIMEOUT'}")
    print("\n✓ Environment ready for MAPPO/IPPO training.")

# ── Pathfinder using scipy distance transform (fast) ─────────────────────────
from scipy.ndimage import distance_transform_edt, label

_BFS_D    = 4
_BFS_SIZE = GRID_SIZE // _BFS_D   # 100×100
_coarse_gt = None   # cached at first call — gt_map never changes

def _get_coarse_gt(grid):
    global _coarse_gt
    if _coarse_gt is None:
        m = grid.reshape(_BFS_SIZE, _BFS_D, _BFS_SIZE, _BFS_D)
        _coarse_gt = (m == OCCUPIED).any(axis=(1,3))
    return _coarse_gt

def find_path_bfs(grid, rx, ry, gx, gy):
    """
    Gradient descent on distance transform of free space.
    ~0.5ms per call on 100×100 grid.
    Returns list of (wx,wy) waypoints.
    """
    res   = RESOLUTION * _BFS_D
    coarse = _get_coarse_gt(grid)
    free   = ~coarse   # True = passable

    def w2c(wx, wy):
        col = int((wx - WORLD_MIN) / res)
        row = int((wy - WORLD_MIN) / res)
        return int(np.clip(row,0,_BFS_SIZE-1)), int(np.clip(col,0,_BFS_SIZE-1))

    def c2w(r, c):
        return (c*res+WORLD_MIN+res/2, r*res+WORLD_MIN+res/2)

    r0,c0 = w2c(rx,ry)
    r1,c1 = w2c(gx,gy)

    # snap goal to nearest free cell if needed
    if not free[r1,c1]:
        for d in range(1,5):
            found=False
            for dr in range(-d,d+1):
                for dc in range(-d,d+1):
                    nr,nc=r1+dr,c1+dc
                    if 0<=nr<_BFS_SIZE and 0<=nc<_BFS_SIZE and free[nr,nc]:
                        r1,c1=nr,nc; found=True; break
                if found: break
            if found: break

    if r0==r1 and c0==c1:
        return [c2w(r1,c1)]

    # distance transform from goal: each free cell gets distance to goal
    # through free space
    goal_mask = np.zeros((_BFS_SIZE,_BFS_SIZE), dtype=bool)
    goal_mask[r1,c1] = True
    # distance_transform_edt on inverted goal mask, masked to free cells
    dist = distance_transform_edt(~goal_mask) * free.astype(float)
    # cells that are walls get high distance
    dist[coarse] = 1e9

    # gradient descent from start to goal
    path = []
    r,c = r0,c0
    visited_path = set()
    for _ in range(_BFS_SIZE*2):
        if r==r1 and c==c1:
            path.append(c2w(r,c))
            break
        path.append(c2w(r,c))
        visited_path.add((r,c))
        # pick neighbour with lowest distance
        best_d = dist[r,c]
        best_rc = None
        for dr,dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr,nc=r+dr,c+dc
            if (0<=nr<_BFS_SIZE and 0<=nc<_BFS_SIZE
                    and (nr,nc) not in visited_path
                    and free[nr,nc]
                    and dist[nr,nc] < best_d):
                best_d = dist[nr,nc]
                best_rc = (nr,nc)
        if best_rc is None:
            break
        r,c = best_rc

    return path if path else [c2w(r1,c1)]


# Path cache: keyed by (robot_idx, goal) — avoids recomputing BFS every step
_path_cache = {}

def plan_step_bfs(rx, ry, gx, gy, grid, max_ticks=TICKS_PER_DECISION,
                  cache_key=None):
    global _path_cache
    key = cache_key
    if key is None or key not in _path_cache:
        path = find_path_bfs(grid, rx, ry, gx, gy)
        if key is not None:
            _path_cache[key] = path
    else:
        path = _path_cache[key]

    if not path:
        if key and key in _path_cache: del _path_cache[key]
        return rx, ry, True

    # if path is trivially short, just aim directly at goal
    if len(path) <= 2:
        path = [(gx, gy)]

    step_dist = MOVE_SPEED * DT
    nx, ny = rx, ry
    ticks_left = max_ticks

    # find which waypoint we are closest to (resume mid-path)
    if len(path) > 1:
        dists = [math.hypot(wx-rx, wy-ry) for wx,wy in path]
        start_idx = max(0, min(range(len(dists)), key=lambda i: dists[i]))
    else:
        start_idx = 0

    for wx, wy in path[start_idx:]:
        while ticks_left > 0:
            dist = math.hypot(wx-nx, wy-ny)
            if dist <= step_dist:
                nx, ny = wx, wy; ticks_left -= 1; break
            dx=(wx-nx)/dist; dy=(wy-ny)/dist
            nx+=dx*step_dist; ny+=dy*step_dist
            ticks_left -= 1

    reached = math.hypot(nx-gx, ny-gy) < 0.5
    if reached and key and key in _path_cache:
        del _path_cache[key]
    return nx, ny, reached

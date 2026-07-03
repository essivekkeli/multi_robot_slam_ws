#generate_ground_truth.py
#!/usr/bin/env python3
import argparse, os, xml.etree.ElementTree as ET, math, json
import numpy as np

try:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

def parse_pose(t):
    if t is None: return 0.0, 0.0, 0.0
    v = [float(x) for x in t.strip().split()]
    return (v[0] if len(v)>0 else 0.0),(v[1] if len(v)>1 else 0.0),(v[5] if len(v)>5 else 0.0)

def parse_size(t):
    if t is None: return 0.0, 0.0
    v = [float(x) for x in t.strip().split()]
    return v[0], v[1]

def extract_walls(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    sr = world if world is not None else root
    walls = []
    for model in sr.findall("model"):
        name = model.get("name","")
        if not name.startswith("wall"): continue
        mx,my,myaw = parse_pose(getattr(model.find("pose"),"text",None))
        box = model.find(".//geometry/box/size")
        if box is None: continue
        sx,sy = parse_size(box.text)
        if sx==0 or sy==0: continue
        lp = model.find(".//link/pose")
        lx,ly,lyaw = parse_pose(getattr(lp,"text",None) if lp is not None else None)
        walls.append({"name":name,"cx":mx+lx,"cy":my+ly,
                      "yaw":myaw+lyaw,"sx":sx,"sy":sy})
    return walls

def get_inner_boundary(walls):
    """
    Find the inner face of the 4 outer walls.
    Outer walls are named wall_north/south/east/west.
    Returns (x_min, x_max, y_min, y_max) — the clipping box for interior walls.
    """
    boundary = {"x_min": -9.9, "x_max": 9.9, "y_min": -9.9, "y_max": 9.9}
    for w in walls:
        name = w["name"]
        cx, cy, sx, sy = w["cx"], w["cy"], w["sx"], w["sy"]
        if name == "wall_west":   boundary["x_min"] = cx + sx/2
        if name == "wall_east":   boundary["x_max"] = cx - sx/2
        if name == "wall_south":  boundary["y_min"] = cy + sy/2
        if name == "wall_north":  boundary["y_max"] = cy - sy/2
    return boundary

def clip_wall(w, b):
    """
    Clip a wall's bounding box to stay within boundary b.
    Returns clipped (cx, cy, sx, sy) or None if wall is entirely outside.
    Only applies to interior walls — outer walls are kept as-is.
    """
    name = w["name"]
    cx, cy, sx, sy = w["cx"], w["cy"], w["sx"], w["sy"]

    # Outer walls keep their original size
    if name in ("wall_north","wall_south","wall_east","wall_west"):
        return cx, cy, sx, sy

    # Compute current wall extent
    x_min_w = cx - sx/2
    x_max_w = cx + sx/2
    y_min_w = cy - sy/2
    y_max_w = cy + sy/2

    # Clip to inner boundary
    x_min_c = max(x_min_w, b["x_min"])
    x_max_c = min(x_max_w, b["x_max"])
    y_min_c = max(y_min_w, b["y_min"])
    y_max_c = min(y_max_w, b["y_max"])

    # Skip if wall is fully outside boundary
    if x_min_c >= x_max_c or y_min_c >= y_max_c:
        return None

    new_sx = x_max_c - x_min_c
    new_sy = y_max_c - y_min_c
    new_cx = (x_min_c + x_max_c) / 2
    new_cy = (y_min_c + y_max_c) / 2
    return new_cx, new_cy, new_sx, new_sy

def rasterise(walls, boundary, res, mn_x, mn_y, mx_x, mx_y):
    W = int(math.ceil((mx_x-mn_x)/res))
    H = int(math.ceil((mx_y-mn_y)/res))
    grid = np.zeros((H,W), dtype=np.int8)

    for w in walls:
        clipped = clip_wall(w, boundary)
        if clipped is None:
            continue
        cx, cy, sx, sy = clipped
        hx, hy = sx/2, sy/2
        yaw = w["yaw"]
        cy_ = math.cos(-yaw); sy_ = math.sin(-yaw)

        corners = [(cx+math.cos(yaw)*dx-math.sin(yaw)*dy,
                    cy+math.sin(yaw)*dx+math.cos(yaw)*dy)
                   for dx,dy in [(hx,hy),(hx,-hy),(-hx,hy),(-hx,-hy)]]
        bb_mn_x = min(c[0] for c in corners)-res
        bb_mx_x = max(c[0] for c in corners)+res
        bb_mn_y = min(c[1] for c in corners)-res
        bb_mx_y = max(c[1] for c in corners)+res

        c0=max(0,int((bb_mn_x-mn_x)/res)); c1=min(W,int((bb_mx_x-mn_x)/res)+1)
        r0=max(0,int((bb_mn_y-mn_y)/res)); r1=min(H,int((bb_mx_y-mn_y)/res)+1)
        if c0>=c1 or r0>=r1: continue

        cols=np.arange(c0,c1); rows=np.arange(r0,r1)
        cc,rr=np.meshgrid(cols,rows)
        wx=mn_x+(cc+0.5)*res; wy=mn_y+(rr+0.5)*res
        dx=wx-cx; dy=wy-cy
        lx= cy_*dx+sy_*dy; ly=-sy_*dx+cy_*dy
        inside=(np.abs(lx)<=hx)&(np.abs(ly)<=hy)
        grid[rr[inside],cc[inside]]=100

    return grid, W, H

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", default="/ros2_ws/src/multi_robot_slam/worlds/multi_robot_world.sdf")
    parser.add_argument("--res", type=float, default=0.05)
    parser.add_argument("--margin", type=float, default=1.0)
    parser.add_argument("--out_dir", default="/ros2_ws/results/gnn")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"Parsing: {args.sdf}")
    walls = extract_walls(args.sdf)
    boundary = get_inner_boundary(walls)
    print(f"Found {len(walls)} walls")
    print(f"Inner boundary: x=[{boundary['x_min']:.2f},{boundary['x_max']:.2f}]  "
          f"y=[{boundary['y_min']:.2f},{boundary['y_max']:.2f}]")

    print("\nClipping check:")
    for w in walls:
        clipped = clip_wall(w, boundary)
        if clipped is None:
            print(f"  REMOVED:  {w['name']}")
            continue
        cx,cy,sx,sy = clipped
        orig_sx, orig_sy = w["sx"], w["sy"]
        if abs(sx-orig_sx)>0.01 or abs(sy-orig_sy)>0.01:
            print(f"  CLIPPED:  {w['name']:<35} "
                  f"size ({orig_sx:.2f}x{orig_sy:.2f}) -> ({sx:.2f}x{sy:.2f})")
        else:
            print(f"  OK:       {w['name']}")

    all_x=[w["cx"]+w["sx"]/2 for w in walls]+[w["cx"]-w["sx"]/2 for w in walls]
    all_y=[w["cy"]+w["sy"]/2 for w in walls]+[w["cy"]-w["sy"]/2 for w in walls]
    mn_x=min(all_x)-args.margin; mn_y=min(all_y)-args.margin
    mx_x=max(all_x)+args.margin; mx_y=max(all_y)+args.margin

    print(f"\nRasterising at {args.res}m ...")
    grid,W,H = rasterise(walls, boundary, args.res, mn_x, mn_y, mx_x, mx_y)
    occ = np.sum(grid==100)
    print(f"Grid: {W}x{H} = {grid.size:,} cells  occupied={occ:,} ({occ/grid.size*100:.1f}%)")

    npy = os.path.join(args.out_dir,"ground_truth.npy")
    meta_path = os.path.join(args.out_dir,"ground_truth_meta.json")
    np.save(npy, grid)
    json.dump({"resolution":args.res,"world_min_x":mn_x,"world_min_y":mn_y,
               "world_max_x":mx_x,"world_max_y":mx_y,"width":W,"height":H,
               "n_walls":len(walls),"boundary":boundary},
              open(meta_path,"w"), indent=2)
    print(f"Saved: {npy}")

    if HAS_MPL:
        fig,ax=plt.subplots(figsize=(8,8))
        ax.imshow(np.flipud(grid),cmap="gray_r",origin="upper",
                  extent=[mn_x,mx_x,mn_y,mx_y])
        # Draw inner boundary for reference
        bx=[boundary['x_min'],boundary['x_max'],boundary['x_max'],
            boundary['x_min'],boundary['x_min']]
        by=[boundary['y_min'],boundary['y_min'],boundary['y_max'],
            boundary['y_max'],boundary['y_min']]
        ax.plot(bx,by,'r--',linewidth=1,alpha=0.5,label="inner boundary")
        ax.set_title(f"Ground Truth (clipped) — {len(walls)} walls  ({W}x{H} @ {args.res}m)")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.legend(); ax.grid(True,alpha=0.3)
        plt.tight_layout()
        png=os.path.join(args.out_dir,"ground_truth.png")
        plt.savefig(png,dpi=150); plt.close()
        print(f"Saved: {png}")
    print("\nDone.")

if __name__=="__main__":
    main()

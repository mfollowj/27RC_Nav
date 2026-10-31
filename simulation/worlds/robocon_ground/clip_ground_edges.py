#!/usr/bin/env python3
"""把 z≈0 的平面地块网格顶点裁剪到场地边界 ±5.5 内，消除超出围栏的毛边。"""
from pathlib import Path

BOUND = 5.5

MESHES = Path(__file__).resolve().parent.parent / 'robocon_mujoco/meshes'
for f in MESHES.glob('robocon_0*.obj'):
    lines = open(f).read().splitlines()
    verts = [l for l in lines if l.startswith('v ')]
    zs = [float(l.split()[3]) for l in verts]
    if not verts or max(zs) > 0.05:
        continue  # 只处理平面地块
    out = []
    changed = 0
    for l in lines:
        if l.startswith('v '):
            p = l.split()
            x, y = float(p[1]), float(p[2])
            nx = min(BOUND, max(-BOUND, x))
            ny = min(BOUND, max(-BOUND, y))
            if nx != x or ny != y:
                changed += 1
            out.append('v %g %g %s' % (nx, ny, ' '.join(p[3:])))
        else:
            out.append(l)
    open(f, 'w').write('\n'.join(out) + '\n')
    print('%s: %d 顶点被裁剪' % (f.name, changed))

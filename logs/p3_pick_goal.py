#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[一次性调试脚本] 从 2D 代价地图（pgm+yaml）里挑一个合适的 Nav2 目标点。

策略：取最大的 free 连通域；在里面选"离机器人起点 (0,0) 1.2~2.5 m"的格子的重心，
这样目标可达、又能明显看出车动了。输出 "x y"。
"""
import math
import re
import sys
from collections import deque


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    m = re.match(rb'(P[25])\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s', data)
    if not m:
        raise SystemExit('不是 PGM 文件：%s' % path)
    kind = m.group(1)
    w, h, maxv = int(m.group(2)), int(m.group(3)), int(m.group(4))
    body = data[m.end():]
    if kind == b'P5':
        vals = list(body[:w * h])
    else:
        vals = [int(v) for v in body.split()[:w * h]]
    return w, h, vals


def read_yaml(path):
    txt = open(path, encoding='utf-8').read()
    res = float(re.search(r'resolution:\s*([0-9.eE+-]+)', txt).group(1))
    m = re.search(r'origin:\s*\[\s*([-0-9.eE+]+)\s*,\s*([-0-9.eE+]+)', txt)
    return res, float(m.group(1)), float(m.group(2))


def main():
    yaml_path = sys.argv[1]
    res, ox, oy = read_yaml(yaml_path)
    pgm = re.search(r'image:\s*(\S+)', open(yaml_path, encoding='utf-8').read()).group(1)
    import os
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm)
    w, h, vals = read_pgm(pgm_path)

    def cell_xy(c, r):
        return ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res

    free = bytearray(w * h)
    for i, v in enumerate(vals):
        free[i] = 1 if v >= 250 else 0        # 254=free, 205=unknown, 0=occupied

    best = []
    seen = bytearray(w * h)
    for start in range(w * h):
        if not free[start] or seen[start]:
            continue
        q = deque([start])
        seen[start] = 1
        comp = []
        while q:
            i = q.popleft()
            comp.append(i)
            c, r = i % w, i // w
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if 0 <= nc < w and 0 <= nr < h:
                    j = nr * w + nc
                    if free[j] and not seen[j]:
                        seen[j] = 1
                        q.append(j)
        if len(comp) > len(best):
            best = comp

    if not best:
        print('%.2f %.2f' % (1.0, 0.0))
        return

    cand = []
    for i in best:
        x, y = cell_xy(i % w, i // w)
        d = math.hypot(x, y)
        if 1.2 <= d <= 2.5:
            cand.append((x, y))
    if not cand:
        cand = [cell_xy(i % w, i // w) for i in best]
    n = len(cand)
    print('%.2f %.2f' % (sum(p[0] for p in cand) / n, sum(p[1] for p in cand) / n))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""先验图体检：存图后立刻判断这张 3D 图能不能拿去重定位。

为什么需要它
------------
重定位（TEASER/GICP）失败时，日志里只有"内点只有几个"，很难看出是算法问题还是
地图问题。实测最常见的地图问题是**旋转拖影**：建图时雷达被快速转动/甩动，LIO 没跟住，
每一帧都以不同的 yaw 叠进地图，同一面墙被画成一圈圈同心圆环。这种图拿去做全局配准
必然失败，但点云看起来"很满"。

本脚本检查 5 项并给 PASS/FAIL：

  1. 轨迹覆盖   —— 原地打转（x/y 几乎不动）或点太少
  2. 旋转拖影   —— 墙带切片的"同心环"检测（极坐标下的角度覆盖率）
  3. 竖向分布   —— 是否大部分点堆在高处（天花板/拖影）
  4. meta 一致性—— imu_height 是否像"米"、ground_z 是否离谱
  5. 密度       —— 5 cm 格子每格点数

用法
----
    python3 check_prior_map.py --pcd robot/maps/prior_map.pcd \
        --meta robot/maps/prior_costmap.meta.json \
        --traj robot/maps/prior_traj.txt

退出码：0 = PASS（可拿去重定位），1 = FAIL（建议重建图）。
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np


# ----------------------------------------------------------------------------
# 读取
# ----------------------------------------------------------------------------
def read_pcd(path):
    if not os.path.isfile(path):
        raise SystemExit('找不到点云：%s' % path)
    header = {}
    with open(path, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise SystemExit('PCD 头不完整：%s' % path)
            txt = line.decode('ascii', 'replace').strip()
            if not txt or txt.startswith('#'):
                continue
            p = txt.split()
            header[p[0].upper()] = p[1:]
            if p[0].upper() == 'DATA':
                break
        fields = [s.lower() for s in header.get('FIELDS', [])]
        sizes = [int(s) for s in header.get('SIZE', [])]
        types = [s.upper() for s in header.get('TYPE', [])]
        npts = int(header.get('POINTS', ['0'])[0])
        kind = header['DATA'][0].lower()
        tmap = {'F': 'f', 'U': 'u', 'I': 'i'}
        dt = np.dtype([(fields[i], '<' + tmap.get(types[i], 'f') + str(sizes[i]))
                       for i in range(len(fields))])
        if kind == 'binary':
            arr = np.frombuffer(f.read(dt.itemsize * npts), dtype=dt, count=npts)
        elif kind == 'ascii':
            arr = np.loadtxt(f, dtype=dt, max_rows=npts)
        else:
            raise SystemExit('不支持的 PCD DATA 类型：%s' % kind)
    names = arr.dtype.names or ()
    cols = []
    for w in ('x', 'y', 'z'):
        hit = [n for n in names if n.lower() == w]
        if not hit:
            raise SystemExit('点云缺字段 %s' % w)
        cols.append(arr[hit[0]].astype(np.float64))
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def read_traj(path):
    if not path or not os.path.isfile(path):
        return None
    try:
        a = np.loadtxt(path, comments='#')
    except Exception:                                   # noqa: BLE001
        return None
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a if a.shape[0] and a.shape[1] >= 2 else None


# ----------------------------------------------------------------------------
# 指标
# ----------------------------------------------------------------------------
def polar_coverage(pts_xy, r_bin=0.5, sectors=72, min_cells=30):
    """按半径分箱，统计每个半径环上"被占用的角度扇区比例"。

    同心圆环（旋转拖影）的特征：某个半径上几乎 360° 都有点 -> coverage≈1。
    正常房间：一个半径环只与几面墙相交 -> coverage 远小于 1。
    返回 (ring_ratio, details)：ring_ratio = coverage>0.9 的半径箱占比。
    """
    if len(pts_xy) < 100:
        return 0.0, 0.0, []
    cell = 0.1
    key = np.floor(pts_xy / cell).astype(np.int64)
    cells = np.unique(key, axis=0)
    xy = cells * cell
    r = np.hypot(xy[:, 0], xy[:, 1])
    th = (np.arctan2(xy[:, 1], xy[:, 0]) + math.pi) / (2 * math.pi)   # [0,1)
    sec = np.clip((th * sectors).astype(np.int64), 0, sectors - 1)
    nb = int(r.max() / r_bin) + 1
    details = []
    ring_like = 0
    valid = 0
    for b in range(1, nb):
        m = (r >= (b - 1) * r_bin) & (r < b * r_bin)
        n = int(m.sum())
        if n < min_cells:
            continue
        cov = len(np.unique(sec[m])) / float(sectors)
        valid += 1
        if cov > 0.9:
            ring_like += 1
        details.append((b * r_bin, n, cov))
    ring_ratio = ring_like / valid if valid else 0.0
    near = [c for rad, n, c in details if 0.5 <= rad <= 5.0 and n >= 100]
    near_mean = float(np.mean(near)) if near else 0.0
    return ring_ratio, near_mean, details


def check(pcd, meta_path, traj_path, args):
    pts = read_pcd(pcd)
    n = len(pts)
    if n == 0:
        raise SystemExit('点云为空：%s' % pcd)
    meta = {}
    if meta_path and os.path.isfile(meta_path):
        try:
            meta = json.load(open(meta_path))
        except Exception:                               # noqa: BLE001
            meta = {}

    results = []          # (名称, 状态, 说明)
    fail = 0
    warn = 0

    def add(name, ok, detail, warn_only=False):
        nonlocal fail, warn
        if ok:
            results.append((name, 'PASS', detail))
        elif warn_only:
            warn += 1
            results.append((name, 'WARN', detail))
        else:
            fail += 1
            results.append((name, 'FAIL', detail))

    # ---- 1. 轨迹覆盖 ----
    traj = read_traj(traj_path)
    if traj is None:
        add('轨迹覆盖', False, '读不到轨迹文件 %s' % traj_path, warn_only=True)
    else:
        x, y, z = traj[:, 0], traj[:, 1], traj[:, 2] if traj.shape[1] > 2 else np.zeros(len(traj))
        span = max(x.max() - x.min(), y.max() - y.min())
        path_len = float(np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1).sum()) if len(traj) > 1 else 0.0
        zdrift = float(np.abs(z - np.median(z)).max())
        ok = (len(traj) >= 50) and (span >= args.min_span)
        add('轨迹覆盖', ok,
            '轨迹 %d 点，平面跨度 %.2f m（阈值 %.1f），路程 %.1f m%s'
            % (len(traj), span, args.min_span, path_len,
               '；原地打转，LIO 没有真正走过场地' if not ok else ''))
        add('轨迹 z 漂移', zdrift <= args.max_zdrift,
            '相对中位数最大偏移 %.2f m（阈值 %.2f）%s'
            % (zdrift, args.max_zdrift, '；竖向漂移，地图会分层' if zdrift > args.max_zdrift else ''),
            warn_only=True)

    # ---- 地面高度 ----
    gz = meta.get('ground_z', None)
    if gz is None:
        gz = float(np.percentile(pts[:, 2], 2.0))
        gz_src = '点云 2% 分位'
    else:
        gz = float(gz)
        gz_src = 'meta.ground_z'

    # ---- 2. 旋转拖影 ----
    band = pts[(pts[:, 2] > gz + 0.2) & (pts[:, 2] < gz + 1.8)]
    ratio, near_mean, det = polar_coverage(band[:, :2])
    ring_ok = ratio <= args.max_ring_ratio
    detail = ('墙带 %d 点（z∈[%.2f,%.2f]，%s）；同心环半径占比 %.0f%%（阈值 %.0f%%）'
              % (len(band), gz + 0.2, gz + 1.8, gz_src,
                 100 * ratio, 100 * args.max_ring_ratio))
    if not ring_ok:
        detail += '；墙被画成一圈圈同心圆环 = 建图时雷达被快速转动、LIO 没跟住'
    add('旋转拖影', ring_ok, detail)
    # 近场角度覆盖率只是提示：四面都有墙的场地，某个半径环本来就接近 360°。
    # 真正的拖影判据是上面的"同心环半径占比"。
    add('近场覆盖', near_mean <= args.max_near_cov,
        '近场角度覆盖率 %.2f（阈值 %.2f，仅提示）' % (near_mean, args.max_near_cov),
        warn_only=True)

    # ---- 3. 竖向分布 ----
    high = float((pts[:, 2] > gz + 2.5).mean())
    add('竖向分布', high <= args.max_high_frac,
        '地面以上 2.5 m 的点占 %.0f%%（阈值 %.0f%%）%s'
        % (100 * high, 100 * args.max_high_frac,
           '；大部分点堆在高处（天花板/拖影），地面结构缺失' if high > args.max_high_frac else ''),
        warn_only=True)

    # ---- 4. meta 一致性 ----
    imu_h = meta.get('imu_height', None)
    if imu_h is not None:
        bad_unit = (imu_h < 0.05 or imu_h > 2.0)
        add('imu_height', not bad_unit,
            'meta.imu_height = %s%s' % (imu_h, '（看起来不是米；30 cm 应写 0.30）' if bad_unit else ''))
        if not bad_unit:
            add('ground_z 合理性', abs(gz + imu_h) <= 0.35,
                'ground_z=%.3f，-imu_height=%.3f，差 %.2f m'
                % (gz, -imu_h, abs(gz + imu_h)),
                warn_only=True)
    else:
        add('imu_height', False, 'meta.json 里没有 imu_height', warn_only=True)

    # ---- 5. 密度 ----
    # 0.20 m 格子：累积器按 0.05 m 去重，所以 0.05 m 格子必然≈1 点，
    # 用 0.20 m 才能反映"同一块地方被扫到几次"。
    res = 0.20
    key = np.floor(pts / res).astype(np.int64)
    _, counts = np.unique(key, axis=0, return_counts=True)
    med = float(np.median(counts))
    single = float((counts <= 1).mean())
    add('0.2 m 密度', med >= 2.0,
        '格子 %d 个，每格中位 %.1f 点，单点格 %.0f%%%s'
        % (len(counts), med, 100 * single,
           '；数据太稀，2D 图和配准都会不稳' if med < 2.0 else ''))

    # ---- 输出 ----
    print('=' * 78)
    print('先验图体检：%s（%d 点）' % (pcd, n))
    print('  meta: %s' % (meta_path or '(无)'))
    print('  traj: %s' % (traj_path or '(无)'))
    print('-' * 78)
    for name, st, detail in results:
        print('  [%-4s] %-12s %s' % (st, name, detail))
    print('-' * 78)
    if fail:
        print('VERDICT: FAIL（%d 项不合格）—— 建议重做地图：雷达固定、慢速走遍场地，'
              '存图时 --imu-height 0.30' % fail)
    elif warn:
        print('VERDICT: WARN（%d 项可疑，但可先试重定位）' % warn)
    else:
        print('VERDICT: PASS —— 这张图可以拿去重定位')
    print('=' * 78)
    return 0 if fail == 0 else 1


def main():
    ap = argparse.ArgumentParser(description='先验图体检（重定位可用性）')
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--meta', default='')
    ap.add_argument('--traj', default='')
    ap.add_argument('--min-span', type=float, default=1.5,
                    help='轨迹平面跨度下限 (m)；低于它视为原地打转')
    ap.add_argument('--max-zdrift', type=float, default=0.30)
    ap.add_argument('--max-ring-ratio', type=float, default=0.25,
                    help='同心环半径占比上限；超过它判为旋转拖影')
    ap.add_argument('--max-near-cov', type=float, default=0.60,
                    help='近场(0.5~5 m)角度覆盖率上限；同心环会接近 1.0')
    ap.add_argument('--max-high-frac', type=float, default=0.60)
    a = ap.parse_args()
    return check(a.pcd, a.meta, a.traj, a)


if __name__ == '__main__':
    sys.exit(main())

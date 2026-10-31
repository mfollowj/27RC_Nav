"""把 xwd 转 PNG 并做场地颜色定位分析（无需 ImageMagick，仅用 stdlib + PIL）。"""
import struct, sys, zlib
from PIL import Image
from collections import Counter

def xwd_to_png(src, dst):
    d = open(src, 'rb').read()
    be = struct.unpack('>I', d[:4])[0]
    e = '>' if 90 <= be <= 200 else '<'
    h = struct.unpack(e + '25I', d[:100])
    hsize, w, hgt, bpp, bpl, ncolors = h[0], h[4], h[5], h[11], h[12], h[19]
    stride = bpl // w
    off = hsize + ncolors * 12
    pix = d[off:off + bpl * hgt]
    rows = []
    for y in range(hgt):
        line = pix[y * bpl:(y + 1) * bpl]
        row = bytearray([0])
        for x in range(w):
            p = line[x * stride:x * stride + 3]
            row += bytes((p[2], p[1], p[0]))
        rows.append(bytes(row))
    raw = b''.join(rows)
    def chunk(t, data):
        return struct.pack('>I', len(data)) + t + data + struct.pack('>I', zlib.crc32(t + data) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, hgt, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))
    open(dst, 'wb').write(png)
    return w, hgt

def analyze(path):
    im = Image.open(path).convert('RGB'); w, h = im.size; px = im.load()
    rules = {
        '场地·红方半场(浅粉)': lambda r, g, b: 200 < r and r > g + 15 and g >= b - 10 and b > 140,
        '场地·蓝方半场(浅蓝)': lambda r, g, b: b > 170 and b > r + 25 and g > 140,
        '场地·围栏/隔板(棕)': lambda r, g, b: 90 < r < 210 and 40 < g < 150 and b < 70 and r > g + 20,
        '场地·白色标线/高台面': lambda r, g, b: r > 225 and g > 225 and b > 215,
        '机器人(饱和蓝)': lambda r, g, b: b > 150 and r < 80 and g < 80,
        '机器人(纯黑轮)': lambda r, g, b: r < 25 and g < 25 and b < 25,
    }
    res = {}
    for name, f in rules.items():
        xs, ys, n = [], [], 0
        for y in range(0, h, 2):
            for x in range(0, w, 2):
                if f(*px[x, y]):
                    n += 1; xs.append(x); ys.append(y)
        if n:
            res[name] = (n, (min(xs), min(ys), max(xs), max(ys)))
    print(f'{path}: {w}x{h}')
    tot = (w // 2) * (h // 2)
    for k, (n, bb) in sorted(res.items(), key=lambda kv: -kv[1][0]):
        print(f'  {k:22s} {n/tot*100:5.2f}%  范围 x[{bb[0]},{bb[2]}] y[{bb[1]},{bb[3]}]')
    q = Counter()
    for y in range(0, h, 6):
        for x in range(0, w, 6):
            r, g, b = px[x, y]; q[(r // 24 * 24, g // 24 * 24, b // 24 * 24)] += 1
    print('  Top5 颜色: ' + '  '.join(f'{c}:{n/sum(q.values())*100:.1f}%' for c, n in q.most_common(5)))

if __name__ == '__main__':
    for src in sys.argv[1:]:
        png = src.rsplit('.', 1)[0] + '.png'
        xwd_to_png(src, png)
        analyze(png)

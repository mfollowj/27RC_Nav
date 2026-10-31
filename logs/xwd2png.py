"""把 xwd 截图转成 PNG（不依赖 ImageMagick / PIL，只用 zlib+struct）。"""
import struct, sys, zlib

def read_xwd(path):
    d = open(path, 'rb').read()
    # 头 25 个 CARD32；用第一字段 header_size 判断字节序
    if struct.unpack('>I', d[:4])[0] in (100, 104):
        e = '>'
    else:
        e = '<'
    h = struct.unpack(e + '25I', d[:100])
    (hsize, ver, fmt, depth, w, hgt, xoff, border, bunit, bbit, bpad,
     bpp, bpl, vclass, rmask, gmask, bmask, bitsrgb, cmapent, ncolors,
     ww, wh, wx, wy, bdr) = h
    off = hsize + ncolors * 12
    pix = d[off:off + bpl * hgt]
    return w, hgt, bpp, bpl, pix, rmask, gmask, bmask, e

def to_png(src, dst):
    w, hgt, bpp, bpl, pix, rm, gm, bm, e = read_xwd(src)
    def shift(mask):
        if mask == 0: return 0, 0
        s = 0
        while not (mask >> s) & 1: s += 1
        return s, bin(mask >> s).count('1')
    rs, rn = shift(rm); gs, gn = shift(gm); bs, bn = shift(bm)
    nbytes = bpp // 8
    rows = []
    for y in range(hgt):
        row = bytearray([0])  # PNG filter type 0
        line = pix[y*bpl:(y+1)*bpl]
        for x in range(w):
            px = line[x*nbytes:(x+1)*nbytes]
            if nbytes >= 4:  # BGRX
                b, g, r = px[0], px[1], px[2]
            elif nbytes == 3:
                b, g, r = px[0], px[1], px[2]
            else:
                b = g = r = px[0]
            row += bytes((r, g, b))
        rows.append(bytes(row))
    raw = b''.join(rows)
    def chunk(t, data):
        c = struct.pack('>I', len(data)) + t + data
        return c + struct.pack('>I', zlib.crc32(t + data) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, hgt, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw, 6))
    png += chunk(b'IEND', b'')
    open(dst, 'wb').write(png)
    print(f'{src} -> {dst}  {w}x{hgt} {bpp}bpp')

if __name__ == '__main__':
    to_png(sys.argv[1], sys.argv[2])

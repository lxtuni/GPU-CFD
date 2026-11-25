"""报告网页用：一个渲染算例的两个视角（q 涡、f 切片）左右并排，全部帧存成 JPEG（本机再用 ffmpeg 编 MP4）。 python3 viz_frames.py <算例目录> <输出目录> [高度]"""
import glob, os, sys
from PIL import Image, ImageChops
d, od = sys.argv[1], sys.argv[2]; H = int(sys.argv[3]) if len(sys.argv) > 3 else 360; os.makedirs(od, exist_ok=True)
fs = sorted(glob.glob(d + "/frames/*.png")); views = [v for v in ("q", "f") if any(os.path.basename(f).startswith(v + "-") for f in fs)]
def bbox_of(fr):
    bb = None
    for f in fr[:: max(1, len(fr) // 12)]:
        im = Image.open(f).convert("RGB"); b = ImageChops.difference(im, Image.new("RGB", im.size, (255, 255, 255))).getbbox()
        if b: bb = b if bb is None else (min(bb[0], b[0]), min(bb[1], b[1]), max(bb[2], b[2]), max(bb[3], b[3]))
    W, Hh = im.size; p = 12
    return (max(0, bb[0] - p), max(0, bb[1] - p), min(W, bb[2] + p), min(Hh, bb[3] + p))
fr = {v: sorted(f for f in fs if os.path.basename(f).startswith(v + "-")) for v in views}; bb = {v: bbox_of(fr[v]) for v in views}
n = min(len(fr[v]) for v in views)
for i in range(n):
    tiles = []
    for v in views:
        c = Image.open(fr[v][i]).convert("RGB").crop(bb[v]); tiles.append(c.resize((round(c.width * H / c.height), H), Image.LANCZOS))
    W = sum(t.width for t in tiles) + 12 * (len(tiles) - 1); canvas = Image.new("RGB", (W, H), "white"); x = 0
    for t in tiles:
        canvas.paste(t, (x, 0)); x += t.width + 12
    if canvas.width > 1280:
        canvas = canvas.resize((1280, round(canvas.height * 1280 / canvas.width)), Image.LANCZOS)
    canvas.save(f"{od}/{i:04d}.jpg", quality=92)
print(d, n, canvas.size)

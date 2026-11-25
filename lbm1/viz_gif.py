"""把一个渲染算例的帧（frames/<视角>-<步数>.png）按视角合成 GIF：裁白边、缩到 720 宽。 python3 viz_gif.py <算例目录>"""
import glob, os, sys
from PIL import Image, ImageChops
d = sys.argv[1]; fs = sorted(glob.glob(d + "/frames/*.png"))
views = sorted({os.path.basename(f).split("-")[0] for f in fs})
for v in views:
    fr = sorted(f for f in fs if os.path.basename(f).split("-")[0] == v)
    ims = [Image.open(f).convert("RGB") for f in fr]
    bbox = None
    for im in ims[:: max(1, len(ims) // 8)]:
        b = ImageChops.difference(im, Image.new("RGB", im.size, (255, 255, 255))).getbbox()
        if b: bbox = b if bbox is None else (min(bbox[0], b[0]), min(bbox[1], b[1]), max(bbox[2], b[2]), max(bbox[3], b[3]))
    pad = 16; W, H = ims[0].size
    bbox = (max(0, bbox[0] - pad), max(0, bbox[1] - pad), min(W, bbox[2] + pad), min(H, bbox[3] + pad)) if bbox else (0, 0, W, H)
    out = []
    for im in ims:
        c = im.crop(bbox); w = 720; c = c.resize((w, max(2, round(c.height * w / c.width))), Image.LANCZOS); out.append(c.quantize(colors=128, method=Image.MEDIANCUT, dither=Image.NONE))
    p = f"{d}/{os.path.basename(d)}_{v}.gif"
    out[0].save(p, save_all=True, append_images=out[1:], duration=70, loop=0, optimize=True)
    out[len(out) // 2].convert("RGB").save(f"{d}/{os.path.basename(d)}_{v}_mid.png")
    print(p, len(out), os.path.getsize(p) // 1024, "KB")

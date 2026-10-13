"""Textures of the exterior layer (drive markings, paving, lawn, ambulance livery), drawn with PIL.

    uv run --no-project --with pillow --with numpy python scene/exterior_textures.py

Writes textures/exterior/*.png (kept in git, so the USD build needs only usd-core). The atlas layout and the
asphalt texture frame are constants here; scene/build_exterior.py imports them for the UVs (no PIL import then).

The livery is a generic UK emergency ambulance: fluorescent yellow, green/yellow Battenburg chequer, the word
AMBULANCE (mirrored on the front). No logo and no name of any real service or trust.
"""

import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_DIR = os.path.join(REPO, "textures", "exterior")

# Asphalt texture: one image over the whole drive, world-planar. Square SIZE m from (X0, Y0).
ASPHALT_X0, ASPHALT_Y0, ASPHALT_SIZE, ASPHALT_PX = -14.0, -25.0, 19.4, 2048
# A&E asphalt (lane + apron west of x -14): a second world-planar image, AE_W x AE_H m from (AE_X0, AE_Y0).
AE_X0, AE_Y0, AE_W, AE_PX, AE_PY = -42.0, -13.65, 28.0, 2048, 1126   # 73 px per metre
AE_H = AE_PY * AE_W / AE_PX  # 15.39 m: to y 1.74
# Tiling textures: metres per tile.
TILE_M = {"paving": 1.2, "tactile": 0.4, "lawn": 2.0}

# Sign atlas (pixels, origin top-left). Generic wayfinding panels: no logo, no name of a real organisation.
SIGN_W, SIGN_H = 1024, 512
SIGN_REGIONS = {  # name: (px0, py0, width px, height px); each region has the aspect of its panel
    "canopy": (0, 0, 1024, 128),      # canopy fascia panel, 4.0 x 0.5 m: A&E  EMERGENCY
    "totem_ae": (0, 160, 512, 160),   # totem upper panel, 1.6 x 0.5 m: A&E Emergency, arrow left
    "totem_main": (0, 336, 512, 160),  # totem lower panel, 1.6 x 0.5 m: Main entrance, arrow ahead
    "back": (560, 160, 128, 128),     # sign backs and edges: plain grey
}

# Ambulance livery atlas (pixels, origin top-left), 200 px per metre. Vehicle frame: +x forward, +y left, z up.
ATLAS_W, ATLAS_H, PPM = 2048, 1024, 200
REGIONS = {  # name: (px0, py0, width px, height px)
    "side_left": (0, 0, 940, 490),       # box body +y side: x 1.30 -> -3.40 left to right, z 2.90 -> 0.45 down
    "side_right": (0, 500, 940, 490),    # box body -y side: x -3.40 -> 1.30
    "rear": (960, 0, 460, 490),          # rear face: y 1.15 -> -1.15
    "cab_side": (960, 500, 430, 410),    # cab sides: x 3.45 -> 1.30, z 2.50 -> 0.45
    "box_front": (1440, 0, 460, 490),    # box front face: y -1.15 -> 1.15 (seen from the front)
    "grille": (1440, 500, 400, 58),      # y -1.0 -> 1.0, bottom edge = bumper top
    "bonnet": (1440, 570, 400, 125),     # y -1.0 -> 1.0, bottom edge = grille top
    "bumper": (1440, 705, 400, 100),     # y -1.0 -> 1.0, z 0.95 -> 0.45
    "sw_yellow": (1920, 0, 120, 60), "sw_dark": (1920, 70, 120, 60), "sw_black": (1920, 140, 120, 60),
    "sw_green": (1920, 210, 120, 60), "sw_white": (1920, 280, 120, 60),
}

YELLOW = (238, 222, 18)
BB_YELLOW = (252, 232, 0)
GREEN = (0, 120, 70)
DARK = (44, 44, 46)
BLACK = (14, 14, 16)
GLASS = (24, 30, 38)
RED = (190, 18, 18)
AMBER = (235, 140, 10)
WHITE = (236, 236, 232)
SEAM = (70, 66, 30)

BB_W, BB_ROWS = 0.55, (0.95, 1.25, 1.55)   # Battenburg block length and row edges (z)
TOP_ROW = (2.55, 2.77)                      # single chequer row under the roof


def _font(px):
    from PIL import ImageFont
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, px)
    return ImageFont.load_default(px)


def _text(word, colour, bg, w, h, mirror=False):
    """Word stretched to fill w x h pixels (road markings and liveries use stretched letters)."""
    from PIL import Image, ImageDraw
    f = _font(200)
    x0, y0, x1, y1 = f.getbbox(word)
    im = Image.new("RGBA", (x1 - x0 + 8, y1 - y0 + 8), bg + (0,))
    ImageDraw.Draw(im).text((4 - x0, 4 - y0), word, font=f, fill=colour + (255,))
    im = im.resize((w, h), Image.LANCZOS)
    return im.transpose(Image.FLIP_LEFT_RIGHT) if mirror else im


# ---------------------------------------------------------------- livery


def _livery(np, x, z, face):
    """Colour per sample of vehicle coordinates. x is the long axis (or y on rear/front faces)."""
    img = np.zeros(x.shape + (3,), np.uint8)
    img[:] = YELLOW
    if face in ("side_left", "side_right", "cab_side", "rear"):
        bw = 2.30 / 6 if face == "rear" else BB_W
        ref = -1.15 if face == "rear" else -3.40
        k = np.floor((x - ref) / bw).astype(int)
        for r, (z0, z1) in enumerate(zip(BB_ROWS[:-1], BB_ROWS[1:])):
            band = (z >= z0) & (z < z1)
            img[band & ((k + r) % 2 == 0)] = GREEN
            img[band & ((k + r) % 2 == 1)] = BB_YELLOW
        if face != "cab_side":
            band = (z >= TOP_ROW[0]) & (z < TOP_ROW[1])
            img[band & (k % 2 == 1)] = GREEN
            img[band & (k % 2 == 0)] = BB_YELLOW
        img[z < 0.52] = DARK  # sill / rubbing strip
    return img


def _box(np, img, x, z, xr, zr, colour):
    img[(x >= xr[0]) & (x <= xr[1]) & (z >= zr[0]) & (z <= zr[1])] = colour


def _outline(np, img, x, z, xr, zr, colour=SEAM, t=0.02):
    inside = (x >= xr[0]) & (x <= xr[1]) & (z >= zr[0]) & (z <= zr[1])
    inner = (x >= xr[0] + t) & (x <= xr[1] - t) & (z >= zr[0] + t) & (z <= zr[1] - t)
    img[inside & ~inner] = colour


def make_atlas():
    import numpy as np
    from PIL import Image
    atlas = Image.new("RGB", (ATLAS_W, ATLAS_H), YELLOW)

    def grid(name, u_of_i, v_of_j):
        px0, py0, w, h = REGIONS[name]
        i, j = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        return u_of_i(i), v_of_j(j)

    # Box body sides.
    for name, xf in (("side_left", lambda i: 1.30 - i / PPM), ("side_right", lambda i: -3.40 + i / PPM)):
        x, z = grid(name, xf, lambda j: 2.90 - j / PPM)
        img = _livery(np, x, z, name)
        if name == "side_left":   # nearside (UK kerb side): crew side door with a window
            _outline(np, img, x, z, (0.05, 1.15), (0.55, 2.50))
            _box(np, img, x, z, (0.25, 0.95), (1.75, 2.30), GLASS)
            _box(np, img, x, z, (0.12, 0.28), (1.62, 1.68), DARK)
        else:                     # offside: equipment lockers
            for x0, x1 in ((-2.95, -1.95), (-1.70, -0.70)):
                _outline(np, img, x, z, (x0, x1), (0.55, 1.80))
                _box(np, img, x, z, (x0 + 0.40, x0 + 0.60), (1.62, 1.67), DARK)
        _box(np, img, x, z, (-3.40, -3.30), (0.60, 0.68), AMBER)  # rear side marker
        atlas.paste(Image.fromarray(img), REGIONS[name][:2])
        # AMBULANCE above the chequer, between the crew door and the rear (reads left to right on both sides).
        x0 = 1.30 - (-0.55) if name == "side_left" else -2.55 + 3.40
        t = _text("AMBULANCE", GREEN, YELLOW, int(2.0 * PPM), int(0.30 * PPM))
        atlas.paste(t, (REGIONS[name][0] + int(x0 * PPM), REGIONS[name][1] + int((2.90 - 2.27) * PPM)), t)
    # Rear: two doors with windows, lamp clusters.
    x, z = grid("rear", lambda i: 1.15 - i / PPM, lambda j: 2.90 - j / PPM)
    img = _livery(np, x, z, "rear")
    _box(np, img, x, z, (-1.15, 1.15), (0.45, 0.72), DARK)
    for s in (-1, 1):
        lo, hi = sorted((s * 0.15, s * 0.90))
        _box(np, img, x, z, (lo, hi), (1.85, 2.40), GLASS)
        lo, hi = sorted((s * 0.98, s * 1.12))
        _box(np, img, x, z, (lo, hi), (0.75, 1.30), RED)
        _box(np, img, x, z, (lo, hi), (0.75, 0.90), AMBER)
        lo, hi = sorted((s * 0.05, s * 0.20))
        _box(np, img, x, z, (lo, hi), (1.62, 1.68), DARK)
    _outline(np, img, x, z, (-1.05, 1.05), (0.72, 2.62))
    _box(np, img, x, z, (-0.012, 0.012), (0.72, 2.62), SEAM)
    _box(np, img, x, z, (-0.25, 0.25), (2.76, 2.83), RED)  # high-level brake light
    atlas.paste(Image.fromarray(img), REGIONS["rear"][:2])
    # Cab sides (shared by both sides): door seams, window area, Battenburg continues.
    x, z = grid("cab_side", lambda i: 3.45 - i / PPM, lambda j: 2.50 - j / PPM)
    img = _livery(np, x, z, "cab_side")
    _box(np, img, x, z, (1.39, 1.41), (0.52, 2.30), SEAM)
    _box(np, img, x, z, (2.61, 2.63), (0.52, 1.42), SEAM)
    win = (z >= 1.45) & (z <= 2.20) & (x >= 1.45) & (x <= 2.55 - (z - 1.45) * (0.55 / 0.75))
    img[win] = GLASS
    _box(np, img, x, z, (1.50, 1.65), (1.30, 1.36), DARK)
    atlas.paste(Image.fromarray(img), REGIONS["cab_side"][:2])
    # Box front above the cab: mirrored AMBULANCE.
    px0, py0, w, h = REGIONS["box_front"]
    t = _text("AMBULANCE", GREEN, YELLOW, int(1.9 * PPM), int(0.26 * PPM), mirror=True)
    atlas.paste(t, (px0 + int(0.20 * PPM), py0 + int(0.05 * PPM)), t)
    # Grille: dark grille, headlamps, blue grille lights.
    y, s = grid("grille", lambda i: -1.0 + i / PPM, lambda j: 0.29 - j / PPM)
    img = np.zeros(y.shape + (3,), np.uint8)
    img[:] = YELLOW
    img[(np.abs(y) < 0.45) & (s > 0.03)] = DARK
    img[(np.abs(y) >= 0.55) & (np.abs(y) <= 0.93) & (s > 0.04) & (s < 0.26)] = (225, 230, 235)
    img[(np.abs(y) >= 0.30) & (np.abs(y) <= 0.42) & (s > 0.06) & (s < 0.12)] = (40, 90, 255)
    atlas.paste(Image.fromarray(img), REGIONS["grille"][:2])
    # Bonnet: mirrored AMBULANCE, read in a mirror by the driver ahead.
    px0, py0, w, h = REGIONS["bonnet"]
    t = _text("AMBULANCE", GREEN, YELLOW, int(1.7 * PPM), int(0.22 * PPM), mirror=True)
    atlas.paste(t, (px0 + int(0.15 * PPM), py0 + int(0.20 * PPM)), t)
    # Bumper with a blank plate.
    px0, py0, w, h = REGIONS["bumper"]
    atlas.paste(YELLOW, (px0, py0, px0 + w, py0 + h))   # rows: z 0.95 (top) -> 0.45 (bottom)
    atlas.paste(DARK, (px0 + int(0.55 * PPM), py0 + int(0.03 * PPM), px0 + int(1.45 * PPM), py0 + int(0.22 * PPM)))
    atlas.paste(DARK, (px0, py0 + int(0.27 * PPM), px0 + w, py0 + h))
    atlas.paste(WHITE, (px0 + int(0.74 * PPM), py0 + int(0.31 * PPM), px0 + int(1.26 * PPM), py0 + int(0.42 * PPM)))
    for name, c in (("sw_yellow", YELLOW), ("sw_dark", DARK), ("sw_black", BLACK), ("sw_green", GREEN),
                    ("sw_white", WHITE)):
        px0, py0, w, h = REGIONS[name]
        atlas.paste(c, (px0, py0, px0 + w, py0 + h))
    atlas.save(os.path.join(TEX_DIR, "ambulance_livery.png"))


# ---------------------------------------------------------------- ground


def _asphalt_base(np, Image, w, h, seed):
    rng = np.random.default_rng(seed)
    base = 58 + rng.normal(0, 3.5, (h, w))
    low = np.asarray(Image.fromarray(rng.normal(0, 3, (32, round(32 * h / w) or 1)).astype(np.float32).T, "F")
                     .resize((w, h), Image.BICUBIC))
    base = np.clip(base + low, 0, 255)
    return Image.fromarray(np.stack([base, base, base + 2], -1).clip(0, 255).astype(np.uint8))


class _Painter:
    """Road markings in world metres on a world-planar asphalt image (x0, y0 = world point of the bottom-left)."""
    YELLOW, WHITE, ASPHALT = (236, 196, 34), (232, 232, 228), (58, 58, 60)
    LW = 0.1  # line width, m

    def __init__(self, img, x0, y0, ppm):
        from PIL import ImageDraw
        self.img, self.x0, self.y0, self.ppm = img, x0, y0, ppm
        self.top = y0 + img.size[1] / ppm
        self.d = ImageDraw.Draw(img)

    def P(self, x, y):
        return ((x - self.x0) * self.ppm, (self.top - y) * self.ppm)

    def rect(self, x0, y0, x1, y1, c):
        (a, b), (e, f) = self.P(x0, y1), self.P(x1, y0)
        self.d.rectangle((a, b, e, f), fill=c)

    def line(self, p, q, c, w=LW):
        self.d.line((self.P(*p), self.P(*q)), fill=c, width=max(1, round(w * self.ppm)))

    def text(self, word, c, x0, y0, x1, y1, rotate=0):
        """Word stretched into the world rectangle. rotate 0: letters upright for a driver heading north;
        90: for a driver heading west; -90: heading east."""
        w, h = round((x1 - x0) * self.ppm), round((y1 - y0) * self.ppm)
        t = _text(word, c, self.ASPHALT, *((h, w) if rotate else (w, h)))
        if rotate:
            t = t.rotate(rotate, expand=True)
        self.img.paste(t, tuple(round(v) for v in self.P(x0, y1)), t)

    def hatch(self, x0, y0, x1, y1, c, pitch=0.6, w=0.12):
        """Diagonal hatch clipped to the rectangle (no border lines)."""
        import numpy as np
        from PIL import Image, ImageDraw
        m = Image.new("L", self.img.size, 0)
        hd = ImageDraw.Draw(m)
        for k in np.arange(-(y1 - y0), x1 - x0, pitch):
            hd.line((self.P(x0 + k, y0), self.P(x0 + k + (y1 - y0), y1)), fill=255, width=round(w * self.ppm))
        box = Image.new("L", self.img.size, 0)
        (a, b), (e, f) = self.P(x0, y1), self.P(x1, y0)
        ImageDraw.Draw(box).rectangle((a, b, e, f), fill=255)
        m = Image.fromarray(np.minimum(np.asarray(m), np.asarray(box)))
        self.img.paste(Image.new("RGB", self.img.size, c), (0, 0), m)

    def bay(self, r, kind, open_side=None):
        """Bay box (white for patient transport, yellow for ambulance) and its words at the mouth (south end).
        open_side: a side ("N", "S") without a line, where the bay meets a kerb or a hatch."""
        c = self.YELLOW if kind == "ambulance" else self.WHITE
        x0, y0, x1, y1 = r
        lw = self.LW
        self.rect(x0 - lw / 2, y0, x0 + lw / 2, y1, c)
        self.rect(x1 - lw / 2, y0, x1 + lw / 2, y1, c)
        if open_side != "S":
            self.rect(x0, y0, x1, y0 + lw, c)
        if open_side != "N":
            self.rect(x0, y1 - lw, x1, y1, c)
        if kind == "ambulance":
            self.text("AMBULANCE", c, x0 + 0.3, y0 + 0.4, x1 - 0.3, y0 + 1.4)
        else:  # nearest line first for a driver coming from the south
            self.text("PATIENT", c, x0 + 0.5, y0 + 0.4, x1 - 0.5, y0 + 1.3)
            self.text("TRANSPORT", c, x0 + 0.3, y0 + 1.6, x1 - 0.3, y0 + 2.5)


def make_asphalt(E):
    import numpy as np
    from PIL import Image
    n, ppm = ASPHALT_PX, ASPHALT_PX / ASPHALT_SIZE
    img = _asphalt_base(np, Image, n, n, 7)
    p = _Painter(img, ASPHALT_X0, ASPHALT_Y0, ppm)
    white, yellow = p.WHITE, p.YELLOW
    # Bays inside this image: the two patient-transport bays at the main entrance (white). The bay head is the
    # flush kerb of the footway, so the north side has no line.
    for name, r in E["bays"].items():
        if ASPHALT_X0 <= r[0] and r[2] <= ASPHALT_X0 + ASPHALT_SIZE:
            p.bay(r, E["bay_kind"][name], open_side="N")
    # Keep-clear hatch beside the bays.
    x0, y0, x1, y1 = E["keep_clear"]
    p.hatch(x0, y0, x1, y1, yellow)
    for a, b in (((x0, y0), (x1, y0)), ((x0, y0), (x0, y1)), ((x1, y0), (x1, y1))):
        p.line(a, b, yellow)
    # Drop-off lay-by: white box and DROP OFF.
    x0, y0, x1, y1 = E["dropoff"]
    for a, b in (((x0, y0), (x1, y0)), ((x0, y0), (x0, y1)), ((x1, y0), (x1, y1))):
        p.line(a, b, white)
    p.text("DROP OFF", white, x0 + 0.5, y0 + 0.4, x1 - 0.5, y0 + 1.2)
    # Access road: centre line (1.5 m marks, 3 m gaps), one arrow per lane (keep left: northbound on the west).
    rx0, ry0, rx1, ry1 = E["road"]
    cx = (rx0 + rx1) / 2
    for y in np.arange(ry0 + 0.5, ry1 - 0.5, 4.5):
        p.rect(cx - p.LW / 2, y, cx + p.LW / 2, y + 1.5, white)
    for ax, sgn in (((rx0 + cx) / 2, 1), ((cx + rx1) / 2, -1)):
        _arrow(p, (ax, (ry0 + ry1) / 2 - 1.0), (0, sgn), white)
    img.save(os.path.join(TEX_DIR, "asphalt.jpg"), quality=88)


def _arrow(p, centre, direction, c):
    """Straight-ahead road arrow, 3.2 m long, along direction (unit vector (dx, dy))."""
    (mx, my), (dx, dy) = centre, direction
    nx, ny = -dy, dx
    tip, back = (mx + dx * 1.6, my + dy * 1.6), (mx - dx * 1.6, my - dy * 1.6)
    p.line(back, (mx, my), c, 0.16)
    p.d.polygon([p.P(*tip), p.P(mx + nx * 0.45, my + ny * 0.45), p.P(mx - nx * 0.45, my - ny * 0.45)], fill=c)


def make_ae_asphalt(E):
    """A&E lane and apron: AE1 bay (yellow), hatched unloading zone behind it, AMBULANCE ONLY on the lane entry,
    lane centre line, one arrow per lane, give-way line where the lane meets the main apron."""
    import numpy as np
    from PIL import Image
    ppm = AE_PX / AE_W
    img = _asphalt_base(np, Image, AE_PX, AE_PY, 11)
    p = _Painter(img, AE_X0, AE_Y0, ppm)
    white, yellow = p.WHITE, p.YELLOW
    for name, r in E["bays"].items():
        if AE_X0 <= r[0] and r[2] <= AE_X0 + AE_W:
            p.bay(r, E["bay_kind"][name])
    # Unloading zone between the AE1 head and the courtyard mouth: keep clear for the trolley.
    x0, y0, x1, y1 = E["bays"][E["ambulance"]["bay"]]
    ay1 = E["ae_apron"][3]
    p.hatch(x0, y1 + p.LW, x1, ay1, yellow)
    for a, b in (((x0, y1), (x0, ay1)), ((x1, y1), (x1, ay1))):
        p.line(a, b, yellow)
    # Lane: centre line, AMBULANCE ONLY for westbound traffic (keep left: westbound on the south half), arrows.
    lx0, ly0, lx1, ly1 = E["ae_lane"]
    cy = (ly0 + ly1) / 2
    for x in np.arange(E["ae_apron"][2] + 0.5, lx1 - 2.5, 4.5):
        p.rect(x, cy - p.LW / 2, x + 1.5, cy + p.LW / 2, white)
    p.text("AMBULANCE", yellow, lx1 - 5.0, ly0 + 0.3, lx1 - 2.4, cy - 0.3, rotate=90)
    p.text("ONLY", yellow, lx1 - 7.8, ly0 + 0.6, lx1 - 5.6, cy - 0.6, rotate=90)
    _arrow(p, (lx1 - 10.5, (ly0 + cy) / 2), (-1, 0), white)
    _arrow(p, (lx1 - 10.5, (cy + ly1) / 2), (1, 0), white)
    # Give way (double broken line) across the eastbound half at the lane mouth.
    for xo in (0.6, 0.9):
        for y in np.arange(cy + 0.1, ly1 - 0.2, 0.9):
            p.rect(lx1 - xo - 0.1, y, lx1 - xo + 0.1, y + 0.6, white)
    img.save(os.path.join(TEX_DIR, "asphalt_ae.jpg"), quality=88)


def make_signs():
    """Sign atlas: canopy fascia panel and the two totem panels. Generic colours; no logo."""
    from PIL import Image, ImageDraw
    red, slate, white, grey = (178, 28, 32), (38, 52, 66), (245, 245, 242), (120, 122, 124)
    im = Image.new("RGB", (SIGN_W, SIGN_H), grey)
    d = ImageDraw.Draw(im)

    def panel(name, bg, words, arrow=None):
        px0, py0, w, h = SIGN_REGIONS[name]
        d.rectangle((px0, py0, px0 + w - 1, py0 + h - 1), fill=bg)
        b = max(3, h // 24)
        d.rectangle((px0 + b, py0 + b, px0 + w - 1 - b, py0 + h - 1 - b), outline=white, width=max(2, b // 2))
        ah, pad, gap = h * 0.26, int(h * 0.25), int(h * 0.25)
        left, right = px0 + pad, px0 + w - pad
        if arrow == "left":   # arrow at the left end, pointing left
            cy = py0 + h / 2
            d.polygon([(left, cy), (left + ah, cy - ah), (left + ah, cy - ah * 0.4), (left + 2.2 * ah, cy - ah * 0.4),
                       (left + 2.2 * ah, cy + ah * 0.4), (left + ah, cy + ah * 0.4), (left + ah, cy + ah)], fill=white)
            left += int(2.2 * ah) + gap
        if arrow == "ahead":  # arrow at the right end, pointing up (straight on)
            cx, top = right - ah, py0 + h * 0.18
            d.polygon([(cx, top), (cx + ah, top + ah), (cx + ah * 0.4, top + ah), (cx + ah * 0.4, py0 + h * 0.82),
                       (cx - ah * 0.4, py0 + h * 0.82), (cx - ah * 0.4, top + ah), (cx - ah, top + ah)], fill=white)
            right -= int(2 * ah) + gap
        text = "  ".join(words)
        px = int(h * 0.46)
        while px > 8:   # largest size that fits between the arrow and the border
            f = _font(px)
            x0, y0, x1, y1 = f.getbbox(text)
            if x1 - x0 <= right - left:
                break
            px -= 2
        x = left + (right - left - (x1 - x0)) / 2 if arrow is None else (left if arrow == "left" else right - (x1 - x0))
        d.text((x - x0, py0 + (h - (y1 - y0)) / 2 - y0), text, font=f, fill=white)

    panel("canopy", red, ["A&E", "EMERGENCY"])
    panel("totem_ae", red, ["A&E", "Emergency"], arrow="left")
    panel("totem_main", slate, ["Main entrance"], arrow="ahead")
    px0, py0, w, h = SIGN_REGIONS["back"]
    d.rectangle((px0, py0, px0 + w - 1, py0 + h - 1), fill=grey)
    im.save(os.path.join(TEX_DIR, "signs.png"))


def make_tiles():
    import numpy as np
    from PIL import Image, ImageDraw
    rng = np.random.default_rng(3)
    n = 256
    # Paving: 2 x 2 concrete flags of 0.6 m per tile, 2 px joints.
    a = np.zeros((n, n, 3))
    for i in range(2):
        for j in range(2):
            a[j * 128:(j + 1) * 128, i * 128:(i + 1) * 128] = 176 + rng.normal(0, 4)
    a += rng.normal(0, 4, (n, n, 1))
    a[:, :, 2] -= 4
    a[[0, 1, 127, 128, 129, 255], :] = 120
    a[:, [0, 1, 127, 128, 129, 255]] = 120
    Image.fromarray(a.clip(0, 255).astype(np.uint8)).save(os.path.join(TEX_DIR, "paving.png"))
    # Tactile blister paving (buff, for an uncontrolled crossing): 6 x 6 blisters per 0.4 m tile.
    im = Image.new("RGB", (n, n), (196, 166, 118))
    d = ImageDraw.Draw(im)
    p = n / 6
    for i in range(6):
        for j in range(6):
            cx, cy, r = (i + 0.5) * p, (j + 0.5) * p, p * 0.32
            d.ellipse((cx - r + 2, cy - r + 2, cx + r + 2, cy + r + 2), fill=(150, 124, 86))
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(214, 186, 138))
    im.save(os.path.join(TEX_DIR, "tactile.png"))
    # Lawn: mown grass noise.
    g = rng.normal(0, 1, (n, n))
    g = (g + np.roll(g, 1, 0) + np.roll(g, 1, 1)) / 3
    rgb = np.stack([62 + 10 * g, 104 + 16 * g, 42 + 8 * g], -1)
    Image.fromarray(rgb.clip(0, 255).astype(np.uint8)).save(os.path.join(TEX_DIR, "lawn.png"))


def main():
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import layout as L
    os.makedirs(TEX_DIR, exist_ok=True)
    make_atlas()
    make_asphalt(L.EXTERIOR)
    make_ae_asphalt(L.EXTERIOR)
    make_signs()
    make_tiles()
    print("wrote", sorted(os.listdir(TEX_DIR)))


if __name__ == "__main__":
    main()

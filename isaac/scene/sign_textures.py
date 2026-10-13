"""Texture of the green "FIRE EXIT" sign over the final exit, drawn with PIL.

    uv run --no-project --with pillow python scene/sign_textures.py

Writes textures/signs/fire_exit.png (kept in git, so the USD build needs only usd-core). scene/fixtures_usd.py puts it
on a 0.6 m x 0.2 m sign over every "EXIT" doorset, on the inside face (BS 5499-4 / ISO 7010 E001 style: white
running figure and door on safety green, with the words FIRE EXIT). It is a generic sign drawn here, not a copy of a
product.
"""

import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "textures", "signs", "fire_exit.png")
SIZE_M = (0.6, 0.2)       # sign width, height (m); the image has the same aspect
PX_PER_M = 2000
GREEN = (0, 133, 66)      # safety green (ISO 3864-4, approx. sRGB)
WHITE = (250, 250, 248)


def _font(px):
    from PIL import ImageFont
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, px)
    return ImageFont.load_default(px)


def _limb(dr, pts, w):
    dr.line(pts, fill=WHITE, width=w, joint="curve")
    for x, y in pts:
        dr.ellipse((x - w / 2, y - w / 2, x + w / 2, y + w / 2), fill=WHITE)


def fire_exit():
    from PIL import Image, ImageDraw
    W, H = int(SIZE_M[0] * PX_PER_M), int(SIZE_M[1] * PX_PER_M)
    img = Image.new("RGB", (W, H), GREEN)
    dr = ImageDraw.Draw(img)
    dr.rectangle((10, 10, W - 11, H - 11), outline=WHITE, width=8)
    # Pictogram in the left H x H square: a figure running right, into a doorway.
    s = H / 400.0

    def P(*xy):  # pictogram units (400 per sign height), shifted 24 units right of the border
        return [((x + 24) * s, y * s) for x, y in zip(xy[::2], xy[1::2])]
    (a, b), (c, d) = P(250, 52, 362, 352)
    dr.rectangle((a, b, c, d), outline=WHITE, width=int(16 * s))                            # door frame
    (a, b), (c, d) = P(150, 50, 210, 110)
    dr.ellipse((a, b, c, d), fill=WHITE)                                                    # head
    _limb(dr, P(172, 130, 142, 238), int(44 * s))                                           # torso
    _limb(dr, P(168, 142, 214, 182, 258, 160), int(26 * s))                                 # front arm
    _limb(dr, P(160, 145, 112, 172, 76, 150), int(26 * s))                                  # back arm
    _limb(dr, P(142, 238, 204, 272, 222, 340), int(32 * s))                                 # front leg
    _limb(dr, P(142, 238, 100, 292, 42, 306), int(32 * s))                                  # back leg
    # Words, centred in the rest of the sign.
    text = "FIRE EXIT"
    px = int(150 * s)
    while True:  # largest size that fits between the pictogram and the border
        f = _font(px)
        l, t, r, b = dr.textbbox((0, 0), text, font=f)
        if r - l <= W - H - 70 * s or px < 20:
            break
        px -= 4
    x0 = H + 10 * s + (W - H - 10 * s - (r - l)) / 2 - l
    dr.text((x0, (H - (b - t)) / 2 - t), text, fill=WHITE, font=f)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    img.save(OUT)
    print(OUT, img.size)


if __name__ == "__main__":
    fire_exit()

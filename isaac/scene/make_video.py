"""Assemble the walkthrough video: title card, shots with crossfades and captions, credits, H.264 encode.

    uv run --no-project --with pillow --with numpy --with imageio-ffmpeg python scene/make_video.py [--draft]

Reads renders/walkthrough/<shot>/NNNN.png (from render_walkthrough.py) and writes
renders/video/hospital_walkthrough_1080p.mp4. --draft fills missing frames with the nearest rendered one,
so the edit can be checked before the full render finishes.
"""

import argparse
import glob
import os
import sys

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import walkthrough as W  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAMES = os.path.join(REPO, "renders", "walkthrough")
OUT = os.path.join(REPO, "renders", "video")
FONTS = "/usr/share/fonts/truetype/dejavu"
TITLE_SECONDS, CREDITS_SECONDS, FADE_SECONDS = 3.0, 5.0, 0.5
SIZE = W.RESOLUTION


def font(name, px):
    return ImageFont.truetype(os.path.join(FONTS, name), px)


def text_center(draw, y, text, fnt, fill=(255, 255, 255)):
    w = draw.textlength(text, font=fnt)
    draw.text(((SIZE[0] - w) / 2 + 2, y + 2), text, font=fnt, fill=(0, 0, 0))  # soft drop shadow
    draw.text(((SIZE[0] - w) / 2, y), text, font=fnt, fill=fill)


def backdrop(frame):
    """Blurred, darkened still used behind the title and credits."""
    img = frame.filter(ImageFilter.GaussianBlur(18))
    return Image.blend(img, Image.new("RGB", SIZE, (8, 12, 18)), 0.62)


def title_card(first):
    img = backdrop(first)
    d = ImageDraw.Draw(img)
    text_center(d, 430, W.CREDITS_TITLE, font("DejaVuSans-Bold.ttf", 84))
    d.line([(SIZE[0] / 2 - 180, 548), (SIZE[0] / 2 + 180, 548)], fill=(120, 190, 255), width=3)
    text_center(d, 575, W.CREDITS_SUBTITLE, font("DejaVuSans-ExtraLight.ttf", 40), (225, 232, 240))
    return img


def credits_card(last):
    img = backdrop(last)
    d = ImageDraw.Draw(img)
    text_center(d, 300, W.CREDITS_TITLE, font("DejaVuSans-Bold.ttf", 60))
    y = 430
    for heading, lines in W.CREDITS:
        text_center(d, y, heading.upper(), font("DejaVuSans-Bold.ttf", 24), (120, 190, 255))
        y += 48
        for line in lines:
            text_center(d, y, line, font("DejaVuSans.ttf", 32), (235, 240, 245))
            y += 50
        y += 40
    return img


def overlay(img, caption, alpha):
    """Room caption (lower left, fades with alpha) and the permanent asset watermark (lower right)."""
    layer = Image.new("RGBA", SIZE, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    wm = font("DejaVuSans.ttf", 20)
    w = d.textlength(W.WATERMARK, font=wm)
    d.text((SIZE[0] - w - 36, SIZE[1] - 52), W.WATERMARK, font=wm, fill=(255, 255, 255, 150))
    if alpha > 0:
        cap = font("DejaVuSans-Bold.ttf", 40)
        a = int(255 * alpha)
        d.rectangle([60, SIZE[1] - 148, 66, SIZE[1] - 100], fill=(120, 190, 255, a))
        d.text((84, SIZE[1] - 150), caption, font=cap, fill=(0, 0, 0, a // 2))
        d.text((82, SIZE[1] - 152), caption, font=cap, fill=(255, 255, 255, a))
    return Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")


def shot_frames(name, draft):
    files = sorted(glob.glob(os.path.join(FRAMES, name, "*.png")))
    have = {int(os.path.basename(f)[:4]): f for f in files}
    n = W.frames_per_shot()
    if not draft and len(have) < n:
        sys.exit(f"shot {name}: {len(have)}/{n} frames rendered (use --draft to preview)")
    keys = sorted(have)
    if not keys:  # draft only: shot not rendered yet
        yield from (Image.new("RGB", SIZE, (30, 30, 30)) for _ in range(n))
        return
    for i in range(n):
        yield Image.open(have[min(keys, key=lambda k: abs(k - i))]).convert("RGB")


def caption_alpha(i, n, fps):
    t, fade = i / fps, 0.6
    return max(0.0, min(1.0, (t - 0.4) / fade, (n / fps - 0.6 - t) / fade))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--draft", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    fps, fade = W.FPS, int(W.CROSSFADE_SECONDS * W.FPS)
    path = os.path.join(OUT, "hospital_walkthrough_1080p" + ("_draft" if args.draft else "") + ".mp4")
    writer = imageio_ffmpeg.write_frames(
        path, SIZE, fps=fps, codec="libx264", quality=None, pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
        macro_block_size=8,
        output_params=["-crf", "16", "-preset", "slow", "-profile:v", "high", "-movflags", "+faststart"])
    writer.send(None)
    count = 0

    def emit(img):
        nonlocal count
        writer.send(np.ascontiguousarray(np.asarray(img, dtype=np.uint8)))
        count += 1

    black = Image.new("RGB", SIZE, "black")
    shots = [(s[0], s[1], list(shot_frames(s[0], args.draft))) for s in W.SHOTS]
    title = title_card(shots[0][2][0])
    for i in range(int(TITLE_SECONDS * fps)):  # title: fade in from black, hold
        emit(Image.blend(black, title, min(1.0, i / (FADE_SECONDS * fps))))
    carry = [title] * fade  # the tail that the next section crossfades from
    for name, caption, frames in shots:
        n = len(frames)
        body = [overlay(f, caption, caption_alpha(i, n, fps)) for i, f in enumerate(frames)]
        for k in range(fade):  # crossfade previous tail into this shot's head
            emit(Image.blend(carry[k], body[k], (k + 1) / (fade + 1)))
        for img in body[fade:n - fade]:
            emit(img)
        carry = body[n - fade:]
    credits = credits_card(shots[-1][2][-1])
    for k in range(fade):
        emit(Image.blend(carry[k], credits, (k + 1) / (fade + 1)))
    hold = int(CREDITS_SECONDS * fps) - fade
    for i in range(hold):  # credits: hold, then fade to black
        out = hold - i
        emit(Image.blend(black, credits, min(1.0, out / (FADE_SECONDS * fps))))
    writer.close()
    print(f"{path}: {count} frames, {count / fps:.1f} s at {fps} fps, {SIZE[0]}x{SIZE[1]}")


if __name__ == "__main__":
    main()

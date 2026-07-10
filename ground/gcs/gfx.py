"""
PIL FreeType text rendering, with cv2 Hershey fallback.

Usage per panel:
    begin()                          # clear the queue
    put_text(panel, text, pos, ...)  # queue a text draw
    size(text, font_scale)           # measure text (PIL-based)
    flush(panel)                     # one-shot PIL render, then clear
"""
import cv2
import numpy as np

try:
    from PIL import ImageFont, ImageDraw, Image as _Img

    _FONTS = [
        '/System/Library/Fonts/Helvetica.ttc',
        '/System/Library/Fonts/SFNSText.ttf',
        '/System/Library/Fonts/HelveticaNeue.ttc',
        '/Library/Fonts/Arial.ttf',
    ]

    # Multiply physical OpenCV font_scale by this to get PIL point size.
    # Calibrated so 0.40*PR ~ 16pt ~ same cap-height as OpenCV Hershey at 0.40*PR.
    _PT = 20
    # Minimum safe point size - Helvetica.ttc raises division-by-zero below ~9pt.
    _PT_MIN = 10

    _cache: dict[int, ImageFont.FreeTypeFont] = {}

    def _font(pt: int) -> ImageFont.FreeTypeFont:
        if pt not in _cache:
            loaded = False
            for path in _FONTS:
                try:
                    _cache[pt] = ImageFont.truetype(path, pt)
                    # Verify the font works at this size before caching
                    _cache[pt].getbbox('A')
                    loaded = True
                    break
                except Exception:
                    _cache.pop(pt, None)
            if not loaded:
                _cache[pt] = ImageFont.load_default()
        return _cache[pt]

    _q: list = []   # queued draws: (text, x, y, pt, rgb)

    def begin() -> None:
        _q.clear()

    def size(text: str, font_scale: float, _thickness: int = 1) -> tuple:
        """cv2.getTextSize-compatible: returns ((w, h), baseline)."""
        pt = max(_PT_MIN, int(font_scale * _PT))
        try:
            bbox = _font(pt).getbbox(text)
            return (bbox[2] - bbox[0], bbox[3] - bbox[1]), 0
        except Exception:
            return cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)

    def put_text(panel: np.ndarray, text: str, pos: tuple,
                 font_scale: float, color: tuple, _thickness: int = 1) -> None:
        pt = max(_PT_MIN, int(font_scale * _PT))
        _q.append((text, pos[0], pos[1], pt, (color[2], color[1], color[0]), font_scale))

    def flush(panel: np.ndarray) -> None:
        """Render all queued text onto panel via PIL FreeType in one pass."""
        if not _q:
            return
        img  = _Img.fromarray(panel[:, :, ::-1])   # BGR->RGB, no copy
        draw = ImageDraw.Draw(img)
        for item in _q:
            text, x, y, pt, rgb, font_scale = item
            try:
                fnt = _font(pt)
                try:
                    asc = fnt.getmetrics()[0]
                except AttributeError:
                    asc = pt * 3 // 4
                draw.text((x, y - asc), text, font=fnt, fill=rgb)
            except Exception:
                # Per-item fallback to cv2 so one bad glyph doesn't lose all text
                panel_bgr = np.array(img)[:, :, ::-1].copy()
                cv2.putText(panel_bgr, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                            font_scale, (rgb[2], rgb[1], rgb[0]), 1, cv2.LINE_AA)
                img = _Img.fromarray(panel_bgr[:, :, ::-1])
                draw = ImageDraw.Draw(img)
        panel[:, :, ::-1] = np.array(img)
        _q.clear()

except ImportError:
    # Graceful fallback: use OpenCV Hershey fonts
    def begin() -> None:
        pass

    def size(text: str, font_scale: float, _thickness: int = 1) -> tuple:
        return cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, max(1, _thickness))

    def put_text(panel: np.ndarray, text: str, pos: tuple,
                 font_scale: float, color: tuple, _thickness: int = 1) -> None:
        cv2.putText(panel, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, color, max(1, _thickness), cv2.LINE_AA)

    def flush(panel: np.ndarray) -> None:
        pass

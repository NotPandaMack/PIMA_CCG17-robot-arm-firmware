from __future__ import annotations


def map_widget_point_to_frame(
    *,
    widget_width: int,
    widget_height: int,
    frame_width: int,
    frame_height: int,
    widget_x: float,
    widget_y: float,
) -> dict | None:
    if widget_width <= 0 or widget_height <= 0 or frame_width <= 0 or frame_height <= 0:
        return None

    scale = min(widget_width / frame_width, widget_height / frame_height)
    display_width = frame_width * scale
    display_height = frame_height * scale
    offset_x = (widget_width - display_width) / 2.0
    offset_y = (widget_height - display_height) / 2.0

    image_x = widget_x - offset_x
    image_y = widget_y - offset_y
    if image_x < 0 or image_y < 0 or image_x > display_width or image_y > display_height:
        return None

    frame_x = image_x / scale
    frame_y = image_y / scale
    return {
        "widgetX": float(widget_x),
        "widgetY": float(widget_y),
        "displayX": float(image_x),
        "displayY": float(image_y),
        "displayRect": (float(offset_x), float(offset_y), float(display_width), float(display_height)),
        "pixelX": int(round(max(0.0, min(frame_width - 1, frame_x)))),
        "pixelY": int(round(max(0.0, min(frame_height - 1, frame_y)))),
        "scale": float(scale),
    }

import cv2
import numpy as np


TAG_ID = 1
MARKER_PIXELS = 1024
CANVAS_PIXELS = 1280


dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)

marker = cv2.aruco.generateImageMarker(
    dictionary,
    TAG_ID,
    MARKER_PIXELS,
)

# Add a white quiet zone around the marker.
canvas = np.full(
    (CANVAS_PIXELS, CANVAS_PIXELS),
    255,
    dtype=np.uint8,
)

offset = (CANVAS_PIXELS - MARKER_PIXELS) // 2

canvas[
    offset:offset + MARKER_PIXELS,
    offset:offset + MARKER_PIXELS
] = marker

cv2.imwrite(
    "materials/textures/tag.png",
    canvas,
)

print("Generated AprilTag 36h11 ID 1")

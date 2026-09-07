"""Occupancy-grid rendering without ROS, shared by the web node and tests."""
import numpy as np


def occupancy_bgr(data, width, height):
    """Return a top-down BGR image; ROS occupancy rows run south to north."""
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full((height, width, 3), (22, 22, 21), dtype=np.uint8)
    image[(cells >= 0) & (cells < 65)] = (35, 35, 34)
    image[cells >= 65] = (225, 224, 217)
    return np.ascontiguousarray(image[::-1])

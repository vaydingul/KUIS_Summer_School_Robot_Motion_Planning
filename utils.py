import numpy as np


def cosd(angle_degree):
  return np.cos(np.radians(angle_degree))


def sind(angle_degree):
  return np.sin(np.radians(angle_degree))


def is_inside_circle(px, py, cx, cy, r):
  return (px - cx) ** 2 + (py - cy) ** 2 <= r ** 2
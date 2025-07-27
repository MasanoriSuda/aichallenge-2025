import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import remove_small_holes
from PIL import Image
from skimage.draw import line_aa
import matplotlib.patches as plt_patches

# Colors
OBSTACLE = '#2E4053'

############
# Obstacle #
############

class Obstacle:
    def __init__(self, cx, cy, radius):
        self.cx = cx
        self.cy = cy
        self.radius = radius

    def show(self):
        circle = plt_patches.Circle(xy=(self.cx, self.cy), radius=self.radius,
                                    color=OBSTACLE, zorder=20)
        ax = plt.gca()
        ax.add_patch(circle)

#######
# Map #
#######

class Map:
    def __init__(self, file_path, origin, resolution, threshold_occupied=100):
        self.threshold_occupied = threshold_occupied
        self.origin = origin
        self.resolution = resolution
        self.obstacles = []
        self.boundaries = []

        if file_path is not None:
            self.data = np.array(Image.open(file_path))[:, :, 0]
            self.process_map()
            self.height = self.data.shape[0]
            self.width = self.data.shape[1]
        else:
            # AWSIM用ダミーマップ（広域）
            self.height = 50000
            self.width = 50000
            self.data = np.ones((self.height, self.width), dtype=np.int8)

    def w2m(self, x, y):
        dx = int(np.floor((x - self.origin[0]) / self.resolution))
        dy = int(np.floor((y - self.origin[1]) / self.resolution))
        return dx, dy

    def m2w(self, dx, dy):
        x = (dx + 0.5) * self.resolution + self.origin[0]
        y = (dy + 0.5) * self.resolution + self.origin[1]
        return x, y

    def process_map(self):
        self.data = np.where(self.data >= self.threshold_occupied, 1, 0)
        self.data = remove_small_holes(self.data, area_threshold=5, connectivity=8).astype(np.int8)

    def add_obstacles(self, obstacles):
        self.obstacles.extend(obstacles)
        for obstacle in obstacles:
            radius_px = int(np.ceil(obstacle.radius / self.resolution))
            cx_px, cy_px = self.w2m(obstacle.cx, obstacle.cy)
            y, x = np.ogrid[-radius_px: radius_px, -radius_px: radius_px]
            index = x**2 + y**2 <= radius_px**2
            self.data[cy_px-radius_px:cy_px+radius_px, cx_px-radius_px:cx_px+radius_px][index] = 0

    def add_boundary(self, boundaries):
        self.boundaries.extend(boundaries)
        for boundary in boundaries:
            sx = self.w2m(boundary[0][0], boundary[0][1])
            gx = self.w2m(boundary[1][0], boundary[1][1])
            path_x, path_y, _ = line_aa(sx[0], sx[1], gx[0], gx[1])
            for x, y in zip(path_x, path_y):
                self.data[y, x] = 0
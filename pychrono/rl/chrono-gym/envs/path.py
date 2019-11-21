import random
import math
from functools import cmp_to_key
import sys

import pychrono as chrono


class RandomPathGenerator:
    def __init__(self, width, height, maxDisplacement, steps):
        self.width = width
        self.height = height
        self.maxDisplacement = maxDisplacement
        self.pushIterations = 3
        self.minDistance = 1
        self.angle = 100
        self.isLooping = True
        self.steps = steps

    def generatePath(self, difficulty, seed):
        self.seed = seed
        self.difficulty = difficulty
        random.seed(self.seed)
        self.points = []
        self.hull = []
        self.generatePoints()
        self.computeConvexHull(self.points)

        for i in range(self.pushIterations):
            self.pushApart()

        self.displace()
        for i in range(10):
            self.fixAngles()
            self.pushApart()
        self.normalizeSize()
        self.hull.append(self.hull[0])

    def generatePoints(self):
        pointCount = 20
        for i in range(pointCount):
            x = (random.random() - 0.5) * self.width
            y = (random.random() - 0.5) * self.height
            self.points.append([x, y])

    def pushApart(self):
        for i in range(len(self.hull)):
            for j in range(len(self.hull)):
                hl = (
                    (self.hull[i][0] - self.hull[j][0]) ** 2
                    + (self.hull[i][1] - self.hull[j][1]) ** 2
                ) ** 1 / 2
                if hl == 0:
                    hl = 0.1

                if hl < self.minDistance:
                    hx = self.hull[j][0] - self.hull[i][0]
                    hy = self.hull[j][1] - self.hull[i][1]
                    hx /= hl
                    hy /= hl
                    diff = self.minDistance - hl
                    hx *= diff
                    hy *= diff
                    self.hull[j][0] += hx
                    self.hull[j][1] += hy
                    self.hull[i][0] -= hx
                    self.hull[i][1] -= hy

    def normalizeSize(self):
        maxX = 0
        maxY = 0
        for i in range(len(self.hull)):
            if abs(self.hull[i][0]) > maxX:
                maxX = abs(self.hull[i][0])

            if abs(self.hull[i][1]) > maxY:
                maxY = abs(self.hull[i][1])

        for i in range(len(self.hull)):
            self.hull[i][0] = (self.hull[i][0] / maxX) * self.width / 2
            self.hull[i][1] = (self.hull[i][1] / maxY) * self.height / 2

    def displace(self):
        newHull = []
        for i in range(len(self.hull)):
            dispLen = (random.random() ** self.difficulty) * self.maxDisplacement
            disp = [0, 1]
            disp = self.rotateVector(disp, random.random() * 360)
            disp = [dispLen * disp[0], dispLen * disp[1]]
            newHull.append(self.hull[i])
            point = self.hull[i]
            point2 = self.hull[(i + 1) % len(self.hull)]
            x = (point[0] + point2[0]) / 2 + disp[0]
            y = (point[1] + point2[1]) / 2 + disp[1]
            newHull.append([x, y])

        self.hull = newHull
        # push apart again, so we can stabilize the points distances.
        for i in range(self.pushIterations):
            self.pushApart()

    def rotateVector(self, v, degrees):
        radians = degrees * (math.pi / 180)
        sin = math.sin(radians)
        cos = math.cos(radians)

        tx = v[0]
        ty = v[1]

        return [cos * tx - sin * ty, sin * tx + cos * ty]

    def fixAngles(self):
        for i in range(len(self.hull)):
            previous = len(self.hull) - 1 if i - 1 < 0 else i - 1
            next = (i + 1) % len(self.hull)
            px = self.hull[i][0] - self.hull[previous][0]
            py = self.hull[i][1] - self.hull[previous][1]
            pl = (px * px + py * py) ** 1 / 2
            px /= pl
            py /= pl

            nx = self.hull[i][0] - self.hull[next][0]
            ny = self.hull[i][1] - self.hull[next][1]
            nx = -nx
            ny = -ny
            nl = (nx * nx + ny * ny) ** 1 / 2
            nx /= nl
            ny /= nl
            # I got a vector going to the next and to the previous points, normalised.

            a = math.atan2(
                px * ny - py * nx, px * nx + py * ny
            )  # perp dot product between the previous and next point.

            if abs(a * (180 / math.pi)) <= self.angle:
                continue

            nA = self.angle * self.sign(a) * (math.pi / 180)
            diff = nA - a
            cos = math.cos(diff)
            sin = math.sin(diff)
            newX = nx * cos - ny * sin
            newY = nx * sin + ny * cos
            newX *= nl
            newY *= nl
            self.hull[next][0] = self.hull[i][0] + newX
            self.hull[next][1] = self.hull[i][1] + newY
            # I got the difference between the current angle and 100degrees, and built a new vector that puts the next point at 100 degrees.

    def sign(self, num):
        return math.copysign(1, num)

    def computeConvexHull(self, points):
        def sort_fun(a, b):
            if a[0] == b[0]:
                return a[1] - b[1]
            elif a[0] > b[0]:
                return 1
            else:
                return -1

        points.sort(key=cmp_to_key(sort_fun))


        L_upper = [points[0], points[1]]		# Initialize upper part
        # Compute the upper part of the hull
        for i in range(2,len(points)):
            L_upper.append(points[i])
            while len(L_upper) > 2 and not self.cw(L_upper[-1],L_upper[-2],L_upper[-3]):
                del L_upper[-2]

        L_lower = [points[-1], points[-2]]	# Initialize the lower part
        # Compute the lower part of the hull
        for i in range(len(points)-3,-1,-1):
            L_lower.append(points[i])
            while len(L_lower) > 2 and not self.cw(L_lower[-1],L_lower[-2],L_lower[-3]):
                del L_lower[-2]
        del L_lower[0]
        del L_lower[-1]
        self.hull = L_upper + L_lower		# Build the full hull

    def cw(self, p1, p2, p3):
        if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
            return False
        return True

    def GetChPath(self, z=0.5):
        ch_hull = chrono.vector_ChVectorD()
        for point in self.hull:
            ch_vector = chrono.ChVectorD(point[0], point[1], z)
            ch_hull.push_back(ch_vector)
        return ch_hull

    def GetPath(self, z=0.5):
        hull = []
        for point in self.hull:
            point.append(z)
            hull.append(point)
        return hull

    def plot(self):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        plot_ax = plt.axes([0.1, 0.3, 0.8, 0.65])
        seed_axes = plt.axes([0.1, 0.15, 0.8, 0.05])
        diff_axes = plt.axes([0.1, 0.10, 0.8, 0.05])
        displ_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
        step_axes = plt.axes([0.1, 0.0, 0.8, 0.05])
        seed_slider = Slider(
            seed_axes, "Seed", 1, 100, valinit=int(self.seed), valstep=1
        )
        diff_slider = Slider(
            diff_axes, "Diff", 1, 1000, valinit=self.difficulty, valstep=1
        )
        displ_slider = Slider(
            displ_axes, "Displ", 1, 100, valinit=self.maxDisplacement, valstep=1
        )
        step_slider = Slider(step_axes, "Step", 1, 100, valinit=self.steps, valstep=1)

        plt.sca(plot_ax)
        x = [point[0] for point in self.hull]
        y = [point[1] for point in self.hull]
        xx = [point[0] for point in self.points]
        yy = [point[1] for point in self.points]
        plt.plot(x, y)
        plt.scatter(xx,yy)

        def update(val):
            self.generatePath(difficulty=diff_slider.val, seed=seed_slider.val)
            self.maxDisplacement = displ_slider.val
            self.steps = step_slider.val
            plt.cla()
            x = [point[0] for point in self.hull]
            y = [point[1] for point in self.hull]
            xx = [point[0] for point in self.points]
            yy = [point[1] for point in self.points]
            plt.plot(x, y)
            plt.scatter(xx,yy)

        seed_slider.on_changed(update)
        diff_slider.on_changed(update)
        displ_slider.on_changed(update)
        step_slider.on_changed(update)
        plt.show()


class Path(chrono.ChBezierCurve):
    def __init__(self, generator):
        chrono.ChBezierCurve.__init__(self, generator.GetChPath())

        self.path = generator.GetChPath()

    def GetChPath(self):
        return self.path

    def calcClosestIndex(self,loc):
        best_i = 0
        best_len = 1000
        for i in range(self.path.size()-1):
            vec = loc - self.path[i]
            len = vec.Length()
            if len < best_len:
                best_i = i
                best_len = len
        return best_i

    def GetArcLength(self,index):
        len = 0
        for i,j in zip(range(0,self.path.size()-1), range(1,self.path.size())):
            p1 = self.path[i]
            p2 = self.path[j]
            vec = p2 - p1
            len += vec.Length()
            if j >= index:
                return len
        return len


class PathTracker(chrono.ChBezierCurveTracker):
    def __init__(self, path):
        chrono.ChBezierCurveTracker.__init__(self, path, True)
        self.path = path
        self.ch_path = self.path.GetChPath()
        self.starting_index = 1

    def GetInitLoc(self):
        return self.ch_path[self.starting_index]

    def GetInitRot(self):
        y_axis = chrono.ChVectorD(1,0,0)
        vec = self.ch_path[self.starting_index+1] - self.ch_path[self.starting_index]
        theta = math.acos((y_axis^vec)/(vec.Length()*y_axis.Length()))
        q = chrono.ChQuaternionD()
        q.Q_from_AngZ(-theta)
        return q


if __name__ == "__main__":
    del chrono
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1
    random_path_generator = RandomPathGenerator(
        width=100, height=100, maxDisplacement=2, steps=1,
    )
    random_path_generator.generatePath(
        difficulty=50, seed=seed,
    )
    random_path_generator.plot()

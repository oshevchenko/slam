from math import sin, cos, atan2, sqrt, pi, tan, hypot, acos
from lego_robot import LegoLogfile
import numpy as np
import itertools
import random

def least_squares_line(points):
    mean_x, mean_y = 0.0, 0.0
    A, B, C = 0.0, 0.0, 0.0
    for p in points:
        x, y = p
        mean_x += x
        mean_y += y
    mean_x = mean_x / len(points)
    mean_y = mean_y / len(points)
    for p in points:
        x, y = p
        A += (x - mean_x) * (y - mean_y)
        B += (x - mean_x)**2
    C = B * mean_y - A * mean_x
    B = -B
    return (np.array([A, B, C]))
    # m = A / B
    # b = mean_y - m * mean_x
def distance_point_to_line(point, line):
    A, B, C = line
    x, y = point
    distance = abs(A*x + B*y +C)/sqrt(A**2+B**2)
    return distance

def distance_point_to_point(P1, P2):
    x1, y1 = P1
    x2, y2 = P2

    distance = hypot(x2 - x1, y2 - y1)
    return distance

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[1] * L2[2] - L1[2] * L2[1] 
    Dy = L1[2] * L2[0] - L1[0] * L2[2] 
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

def angle_line_to_line(L1, L2):
    A1, B1, C1 = L1
    A2, B2, C2 = L2
    
    denom = hypot(A1, B1)*hypot(A2, B2)
    
    if denom != 0:
        angle = acos((A1*A2+B1*B2)/denom)
        # angle = (angle + pi) % (2*pi) - pi
        angle = (angle + pi/2) % (pi) - pi/2
        return abs(angle)
    else:
        return 0


def plot_line(line, min_x, max_x, min_y, max_y, n_points):
    A, B, C = line

    if (A != 0):
        cross_x = abs(-C/A)
    if (B != 0):
        cross_y = abs(-C/B)
    if (A == 0 or cross_x > cross_y):
        X = np.linspace(min_x, max_x, n_points)  # 100 evenly-spaced values from 0 to 50
        Y = [-(A/B)*x-C/B for x in X]
    if (B == 0 or cross_y > cross_x or cross_y == cross_x):
        Y = np.linspace(min_y, max_y, n_points)  # 100 evenly-spaced values from 0 to 50
        X = [-(B/A)*y-C/A for y in Y]

    return ((X,Y))

def plot_ray(ray, n_points):
    A, B, C, min_x, min_y, max_x, max_y = ray

    if (A != 0):
        cross_x = abs(-C/A)
    if (B != 0):
        cross_y = abs(-C/B)
    if (A == 0 or cross_x > cross_y):
        X = np.linspace(min_x, max_x, n_points)  # 100 evenly-spaced values from 0 to 50
        Y = [-(A/B)*x-C/B for x in X]
    if (B == 0 or cross_y > cross_x or cross_y == cross_x):
        Y = np.linspace(min_y, max_y, n_points)  # 100 evenly-spaced values from 0 to 50
        X = [-(B/A)*y-C/A for y in Y]

    return ((X,Y))

class BestLine:
    def __init__(self, line, n_inliners):
        self.best_line = line
        self.n_inliners = n_inliners

    def ProcessScan(self, scan, threshold):
        self.n_inliners = 0
        processed_scan = []
        for offset in xrange(len(scan)):
            point = scan[offset]
            distance = distance_point_to_line(point, self.best_line)
            if distance < threshold:
                self.n_inliners += 1
            else:
                processed_scan.append(point)
        return processed_scan


class SectorInliners:
    def __init__(self, inliners, rays):
        self.best_inliners = inliners
        self.best_inliners_len = len(self.best_inliners)
        self.ray_l, self.ray_r = rays
        self.FindBestLine()

    def FindBestLine(self):
        self.valid = True
        self.best_line = least_squares_line(self.best_inliners)
        self.point_l = intersection(self.best_line, self.ray_l)
        self.point_r = intersection(self.best_line, self.ray_r)


class Sector(SectorInliners):
    def __init__(self, scan, rays, inline_threshold = 60, attempts = 10, valid_threshold = 0.8):
        self.scan = scan
        self.best_inliners = []
        self.best_inliners_len = 0
        self.scan_len = len(self.scan)
        self.valid_threshold = self.scan_len * valid_threshold
        self.ray_l, self.ray_r = rays
        self.inline_threshold = inline_threshold


        self.best_line = []
        self.attempts = attempts
        self.point_l = []
        self.point_r = []
        self.valid = False


        if (self.scan_len < 10):
            return

        for j in xrange(self.attempts):
            points = random.sample(self.scan, 5)
            line = least_squares_line(points)
            inliners = self.get_inliners(line, self.inline_threshold)
            if (len(inliners) > self.best_inliners_len):
                self.best_inliners_len = len(inliners)
                self.best_inliners = inliners

        if (self.best_inliners_len > self.valid_threshold):
            self.FindBestLine()

    def get_inliners(self, line, threshold):
        inliners = []
        for offset in xrange(self.scan_len):
            point = self.scan[offset]
            distance = distance_point_to_line(point, line)

            # print("distance to line:%d"%distance)
            if distance < threshold:
                inliners.append(point)
      
        return inliners



class Ransac:
    def __init__(self, scan, cylinders, points_per_sector = 66,
                                    min_distance = 300, inline_threshold=45,
                                    attempts = 10, valid_threshold = 0.8):
        self.points_per_sector = points_per_sector
        self.scan = []
        self.sector_rays = []
        self.min_distance = min_distance
        self.n_valid_sectors = 0
        self.valid_sectors = []
        self.best_lines = []
        self.inline_threshold = inline_threshold
        self.landmarks = []
        self.walls = []

        offset = 0
        self.sectors = []
        while offset < len(scan) :

            bearing = LegoLogfile.beam_index_to_angle(offset)
            x, y = 3000*cos(bearing), 3000*sin(bearing)
            self.sector_rays.append(np.array([y/x, -1, 0, 0, 0, x, y]))
            ray_r = np.array([y/x, -1, 0])

            bearing = LegoLogfile.beam_index_to_angle(offset+self.points_per_sector)
            x, y = 3000*cos(bearing), 3000*sin(bearing)
            ray_l = np.array([y/x, -1, 0])

            sec_points = self.get_sector_scans_without_landmarks (scan, cylinders,
                                                                         offset)
            self.scan += sec_points
            # for point in sec_points:
            #     self.scan.append(point)

            offset += self.points_per_sector
            sector = Sector(sec_points, (ray_l, ray_r), inline_threshold=self.inline_threshold,
                attempts=attempts, valid_threshold=valid_threshold)
            if sector.valid:
                self.n_valid_sectors += 1
            self.sectors.append(sector)

        self.scan_len = len(self.scan)
        # print(self.n_valid_sectors)


        bearing = LegoLogfile.beam_index_to_angle(offset)
        x, y = 3000*cos(bearing), 3000*sin(bearing)
        self.sector_rays.append(np.array([y/x, -1, 0, 0, 0, x, y]))

        # print(self.set_of_sectors)
        # return self.scan


    def get_sector_scans_without_landmarks (self, scan, cylinders, offset):
        result = []
        for i in xrange(self.points_per_sector):
            j = i + offset
            distance = scan[j]
            save_scan = True
            if (distance < self.min_distance):
                continue

            for measurement, measurement_in_scanner_system, indxs in cylinders:
                # print("indxs[0] %d indxs[1] %d" %(indxs[0], indxs[1]))
                if (j >= indxs[0] and j <= indxs[1]):
                    save_scan = False
                    break
            if save_scan:
                bearing = LegoLogfile.beam_index_to_angle(j)
                x, y = distance*cos(bearing), distance*sin(bearing)
                result.append(np.array([x, y]))
        return result
    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Measurement function. Takes a (x, y, theta) state and a (x, y)
           landmark, and returns the corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return np.array([r, alpha])

    def try_merge_sectors (self):
        n_valid_sectors_new = 0
        new_sectors =[]

        if self.n_valid_sectors < 2:
            return 0

        while self.n_valid_sectors != n_valid_sectors_new:
            self.n_valid_sectors = n_valid_sectors_new
            n_valid_sectors_new = 0
            candidate = None 

            new_sectors =[]
            for sector in self.sectors:
                if sector.valid and candidate == None:
                    candidate = sector
                elif sector.valid and candidate != None:
                    distance = distance_point_to_point(candidate.point_l, sector.point_r)
                    angle = angle_line_to_line(candidate.best_line, sector.best_line)
                    if (distance < 100  and angle < 0.2):
                        candidate.best_inliners += sector.best_inliners
                        candidate = SectorInliners(candidate.best_inliners, (sector.ray_l, candidate.ray_r))
                    else:
                        new_sectors.append(candidate)
                        n_valid_sectors_new += 1
                        candidate = sector
                elif not sector.valid:
                    if candidate != None:
                        new_sectors.append(candidate)
                        n_valid_sectors_new += 1
                        candidate = None
                    new_sectors.append(sector)
            if candidate != None:
                new_sectors.append(candidate)
                n_valid_sectors_new += 1
            self.sectors = new_sectors
            # print("self.n_valid_sectors %d n_valid_sectors_new %d"%(self.n_valid_sectors,n_valid_sectors_new))

        for sector in self.sectors:
            if sector.valid:
                self.best_lines.append(BestLine(sector.best_line, sector.best_inliners_len))
                

        self.best_lines = sorted(self.best_lines, key = lambda x: x.n_inliners, reverse=True)
        for line in self.best_lines:
            self.scan = line.ProcessScan(self.scan, self.inline_threshold)
        new_lines = []
        for line in self.best_lines:
            if (line.n_inliners > 50):
                new_lines.append(line)
        self.best_lines = new_lines
        for subset in itertools.combinations(self.best_lines, 2):
            angle = angle_line_to_line(subset[0].best_line, subset[1].best_line)
            # print("angle",angle)
            if (angle > 0.43):
                x,y = intersection(subset[0].best_line, subset[1].best_line)
                # result.append( (np.array([distance, bearing]), np.array([x, y]), c[1]))
                angle_coordinates = self.h((0,0,0), (x,y), 0)
                self.landmarks.append( (angle_coordinates, np.array([x, y]), (0.0,0.0)))
        for line in self.best_lines:
            # if (line.n_inliners > 30):
            self.walls.append(plot_line(line.best_line, -2000, 4000, -2000, 4000, 2))

        # print(self.landmarks)




            # break
            
    

from math import sin, cos, atan2, sqrt, pi, tan, hypot, acos
from lego_robot import LegoLogfile
import numpy as np

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
        angle = (angle + pi) % (2*pi) - pi
        return angle
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


# class SectorInliners:
#     def __init__(self, inliners, rays, inline_threshold = 50, attempts = 10, valid_threshold = 0.8, from_inliners = False):
#             self.scan=[]
#             self.best_inliners = inliners
#             self.best_inliners_len = len(self.best_inliners)
#             self.valid_threshold = 0

class Sector:
    def __init__(self, scan, rays, inline_threshold = 50, attempts = 10, valid_threshold = 0.8, from_inliners = False):
        if from_inliners:
            self.scan=[]
            self.best_inliners = scan
            self.best_inliners_len = len(self.best_inliners)
            self.valid_threshold = 0
        else:    
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

        if not from_inliners:
            for j in xrange(self.attempts):
                points = self.get_random_points(10)
                line = least_squares_line(points)
                inliners = self.get_inliners(line, self.inline_threshold)
                if (len(inliners) > self.best_inliners_len):
                    self.best_inliners_len = len(inliners)
                    self.best_inliners = inliners

        if (self.best_inliners_len > self.valid_threshold):
            self.valid = True
            self.best_line = least_squares_line(self.best_inliners)
            self.point_l = intersection(self.best_line, self.ray_l)
            self.point_r = intersection(self.best_line, self.ray_r)

    def get_random_points (self, n_points):
        X = np.random.randint(0, high=self.scan_len, size=n_points)
        points = []
        for x in X:
            points.append(self.scan[x])
        return points
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
                                                            min_distance = 300):
        self.points_per_sector = points_per_sector
        self.scan = []
        self.sector_rays = []
        self.min_distance = min_distance
        self.n_valid_sectors = 0

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
            for point in sec_points:
                self.scan.append(point)

            offset += self.points_per_sector
            sector = Sector(sec_points, (ray_l, ray_r))
            if sector.valid:
                self.n_valid_sectors += 1
            self.sectors.append(sector)
        print(self.n_valid_sectors)


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
    def try_merge_sectors (self):
        n_valid_sectors_new = 0
        new_sectors =[]
        candidate_inliners = []
        candidate_rays = []
        if self.n_valid_sectors < 2:
            return 0

        while self.n_valid_sectors != n_valid_sectors_new:
            self.n_valid_sectors = n_valid_sectors_new
            n_valid_sectors_new = 0
            sector_last_candidate = None
            n_candidate_to_merge = 0
            new_sectors =[]
            for sector in self.sectors:
                if sector.valid and n_candidate_to_merge == 0:
                    n_candidate_to_merge = 1
                    candidate_inliners = sector.best_inliners
                    candidate_rays = (sector.ray_l, sector.ray_r)
                    sector_last_candidate = sector
                elif sector.valid and n_candidate_to_merge != 0:
                    distance = distance_point_to_point(sector_last_candidate.point_l, sector.point_r)
                    angle = angle_line_to_line(sector_last_candidate.best_line, sector.best_line)
                    if (distance < 100  and angle < 0.2):
                        print("we are about to merge sectors len: %d"%len(candidate_inliners))
                        print candidate_rays
                        n_candidate_to_merge += 1
                        candidate_inliners += sector.best_inliners
                        candidate_rays = (sector.ray_l, candidate_rays[1])
                        print("we are about to merge sectors len: %d"%len(candidate_inliners))
                        print candidate_rays
                        sector_last_candidate = Sector(candidate_inliners, candidate_rays, from_inliners=True)
                        candidate_inliners = sector_last_candidate.best_inliners
                        candidate_rays = (sector_last_candidate.ray_l, sector_last_candidate.ray_r)
                    else:
                        new_sectors.append(sector_last_candidate)
                        n_valid_sectors_new += 1
                        # n_candidate_to_merge  == 1
                        sector_last_candidate = sector
                        candidate_inliners = sector.best_inliners
                        candidate_rays = (sector.ray_l, sector.ray_r)
                    # print(distance, angle)
                    # sector_last_candidate = sector
                elif not sector.valid:
                    if n_candidate_to_merge != 0:
                        new_sectors.append(sector_last_candidate)
                        n_valid_sectors_new += 1
                    n_candidate_to_merge = 0
                    new_sectors.append(sector)
            if n_candidate_to_merge != 0:
                new_sectors.append(sector_last_candidate)
                n_valid_sectors_new += 1
            self.sectors = new_sectors
            break
        print("n_valid_sectors_new %d"%n_valid_sectors_new)
    




                    # candidate_inliners.append(sector.best_inliners)
                    # candidate_rays.append(sector.rays)








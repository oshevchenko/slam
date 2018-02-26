from math import sin, cos, pi, ceil
from lego_robot import *
from numpy import *
from slam_ransac import *
from pylab import *
from slam_g_library import get_cylinders_from_scan

if __name__ == '__main__':
    depth_jump = 70.0
    minimum_valid_distance = 20.0
    cylinder_offset = 5.0
    points_per_scan = 1000
    max_cylinder_d = 250

    # Read all ticks of left and right motor.
    # Format is:
    # M timestamp[in ms] pos-left[in ticks] * * * pos-right[in ticks] ...
    # so we are interested in field 2 (left) and 6 (right).
    m = array([[8.0, 3.0],
                [2.0, 10.0],
                [11.0, 3.0],
                [6.0, 6.0],
                [5.0, 8.0],
                [4.0, 12.0],
                [12.0, 1.0],
                [9.0, 4.0],
                [6.0, 9.0],
                [1.0, 14.0]])

    line = []
    line = least_squares_line(m)

    for p in m:
        distance = distance_point_to_line(p, line)
        # print(distance)
    logfile = LegoLogfile()
    logfile.read("test_8/output")
    logfile.read("test_8/robo_scan_777.txt")
    # for i in xrange(len(logfile.motor_ticks)-5):
    global ransac
    for i in xrange(170, 171):
    # for i in xrange(100, 101):
    # for i in xrange(97, 98):
        intersection_points=[]
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset, points_per_scan,
            max_cylinder_d)
        ransac = Ransac(logfile.scan_data[i], cylinders)
        points = ransac.scan
        scatter([c[0] for c in points], [c[1] for c in points],
            c='r', s=50)

        for n_sector in xrange(len(ransac.sector_rays)):
            # print(ransac.sector_rays[n_sector])
            P = plot_ray(ransac.sector_rays[n_sector], 5)
            # plot(P[0], P[1])

        points_left = []
        points_right = []
        ransac.try_merge_sectors()
        for sector in ransac.sectors:
            # print("+++")
            if (sector.valid):
                scatter([c[0] for c in sector.best_inliners], [c[1] for c in sector.best_inliners],
                    c='b', s=5)

                P = plot_line(sector.best_line, -2000, 4000, -2000, 4000, 5)
                plot(P[0], P[1])

                points_left.append(sector.point_l)
                points_right.append(sector.point_r)
        # print(points_left)
        scatter([c[0] for c in points_left], [c[1] for c in points_left],
                c='g', s=200)
        scatter([c[0] for c in points_right], [c[1] for c in points_right],
                c='r', s=200)

        # angle = angle_line_to_line(ransac.sectors[0].best_line, ransac.sectors[1].best_line)
        # print(angle)



  

    # x = np.linspace(0, 4000, 100)  # 100 evenly-spaced values from 0 to 50
    # y = 1000*np.sin(x)
    show()

# This file contains helper functions for Unit G of the SLAM lecture.
# Claus Brenner, 27 JAN 2013
from math import sin, cos, atan2, sqrt, pi, tan
from lego_robot import LegoLogfile
import numpy as np

def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        # --->>> Insert here the conversion from polar to Cartesian coordinates.
        # c is a tuple (beam_index, range).
        # For converting the beam index to an angle, use
        # LegoLogfile.beam_index_to_angle(beam_index)
        radius = c[1] + cylinder_offset
        angle = LegoLogfile.beam_index_to_angle(c[0])
        x = radius * cos(angle)
        y = radius * sin(angle)
        result.append( (x, y) ) # Replace this by your (x,y)
    return result

# Utility to write a list of cylinders to (one line of) a given file.
# Line header defines the start of each line, e.g. "D C" for a detected
# cylinder or "W C" for a world cylinder.
def write_cylinders(file_desc, line_header, cylinder_list):
    print >> file_desc, line_header,
    for c in cylinder_list:
        print >> file_desc, "%.1f %.1f" % tuple(c),
    print >> file_desc

# Utility to write a list of error ellipses to (one line of) a given file.
# Line header defines the start of each line.
# Note that in contrast to previous versions, this takes a list of covariance
# matrices instead of list of ellipses.
def write_error_ellipses(file_desc, line_header, covariance_matrix_list):
    print >> file_desc, line_header,
    for m in covariance_matrix_list:
        eigenvals, eigenvects = np.linalg.eig(m)
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        print >> file_desc, "%.3f %.1f %.1f" % \
                 (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1])),
    print >> file_desc


# Find the derivative in scan data, ignoring invalid measurements.

def compute_derivative_old(scan, min_dist):
    jumps = [ 0 ]
    for i in xrange(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

def compute_derivative111(scan, min_dist):
    jumps = [ 0 ]

    for i in xrange(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        # if l > min_dist and r > min_dist:
        derivative = (r - l) / 2.0
        jumps.append(derivative)
        # else:
        #     jumps.append(0)
    jumps.append(0)
    return jumps

def compute_derivative(scan, min_dist):
    jumps = [ ]
    jumps.append(0)
    jumps.append(0)
    jumps.append(0)
    for i in xrange(3, len(scan) - 3):
        h=1
        r1 = scan[i+h]
        r2 = scan[i+2*h]
        l1 = scan[i-h]
        l2 = scan[i-2*h]
        if l1 > min_dist and r2 > min_dist:
            # derivative = (-r2+8*r1-8*l1+l2)/(12*h)
            derivative = (-1*scan[i-3]+9*scan[i-2]-45*scan[i-1]+0*scan[i+0]+
                45*scan[i+1]-9*scan[i+2]+1*scan[i+3])/(60*1.0*h**1)
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    jumps.append(0)
    jumps.append(0)
    return jumps


# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders_old(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0

    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] < -jump:
            # Start a new cylinder, independent of on_cylinder.
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
        elif scan_derivative[i] > jump:
            # Save cylinder if there was one.
            if on_cylinder and rays:
                cylinder_list.append((sum_ray/rays, sum_depth/rays))
            on_cylinder = False
        # Always add point, if it is a valid measurement.
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
    return cylinder_list
def find_cylinders(scan, scan_derivative, jump, min_dist, points_per_scan,
                                                            max_cylinder_d):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    start_cylinder_index = 0
    stop_cylinder_index = 0

    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] == -1:
            # Start a new cylinder, independent of on_cylinder.
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
            start_cylinder_index = i
        elif scan_derivative[i] == 1:
            # Save cylinder if there was one.
            if on_cylinder and rays:
                sigma = (2*pi / points_per_scan) * rays
                D = sum_depth/rays
                d = 2*D*tan(sigma/2)
                # print ("diameter %d "%(d))
                if (d < max_cylinder_d):
                    stop_cylinder_index = i
                    cylinder_list.append(
                        (np.array([sum_ray/rays, sum_depth/rays]),
                        np.array([start_cylinder_index, stop_cylinder_index])))
            on_cylinder = False
        # Always add point, if it is a valid measurement.
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
        #     print ("rays %d "%(rays))
        # print ("i %d onc %d"%(i, on_cylinder))
    return cylinder_list

# Detects cylinders and computes range, bearing and cartesian coordinates
# (in the scanner's coordinate system).
# The result is modified from previous versions: it returns a list of
# tuples of two numpy arrays, the first being (distance, bearing), the second
# being (x, y) in the scanner's coordinate system.
def get_cylinders_from_scan(scan, jump, min_dist, cylinder_offset,
                                points_per_scan, max_cylinder_d):

    scan_f = filter2(scan)

    # der = compute_derivative(scan, min_dist)
    # cylinders = find_cylinders(scan, der, jump, min_dist)
    der = compute_derivative(scan_f, min_dist)
    # der111 = compute_derivative111(scan_f, min_dist)
    der2 = compute_derivative111(der, 0)
    mul_der=[]
    for i in xrange(len(der2)):
        mul_der.append(der[i]*abs(der2[i]))
    mul_der = filter2(mul_der)
    start_stop = []
    start_stop = convert_to_start_stop(mul_der, jump)
    cylinders = find_cylinders(scan_f, start_stop, jump,
                               min_dist, points_per_scan, max_cylinder_d)

    # der = compute_derivative(scan, min_dist)
    # cylinders = find_cylinders(scan, der, jump, min_dist)
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        bearing = LegoLogfile.beam_index_to_angle(c[0][0])
        distance = c[0][1] + cylinder_offset
        # Compute x, y of cylinder in the scanner system.
        x, y = distance*cos(bearing), distance*sin(bearing)
        result.append( (np.array([distance, bearing]), np.array([x, y]), c[1]))
    return result

def filterxxx(scan):
    scan_filtered = [0]
    for i in xrange(1, len(scan) - 1):
        f = scan[i-1] + scan[i]
        f /= 2
        scan_filtered.append(f)
    scan_filtered.append(scan[len(scan) - 1])

    return scan_filtered

def filter1(scan):
    scan_filtered = [0]
    f=[]
    scan_filtered.append(scan[0])
    scan_filtered.append(scan[1])
    for i in xrange(2, len(scan) - 2):
        pyList = [scan[i-2], scan[i-1], scan[i], scan[i+1], scan[i+2]]
        f = sorted(pyList)
        scan_filtered.append(f[2])
    scan_filtered.append(scan[len(scan) - 1])

    return scan_filtered

def filter2(scan):
    scan_filtered = [0]
    scan_filtered.append(scan[0])
    scan_filtered.append(scan[1])
    for i in xrange(2, len(scan) - 2):
        sum = scan[i-2] + scan[i-1] + scan[i] + scan[i+1] + scan[i+2]
        sum /= 5
        scan_filtered.append(sum)
    scan_filtered.append(scan[len(scan) - 1])

    return scan_filtered
def convert_to_start_stop(mul_der, threshold):
    out_list=[]
    on_gorb = False
    on_vpadina = False
    gorb_list=[]
    vpadina_list=[]
    for i in xrange(len(mul_der)):
        if mul_der[i] > threshold and not on_gorb:
            on_gorb = True
            rays_gorb = 1
            start_gorb = i
            if on_vpadina:
                stop_vpadina = i
                if (rays_vpadina > 4):
                    # print("i %d end vpadina rays vpadina %d" %(i,
                    #                                             rays_vpadina))
                    vpadina_list.append((start_vpadina+stop_vpadina)/2)
                on_vpadina = False
            continue
        if mul_der[i] < threshold and on_gorb:
            stop_gorb = i
            if (rays_gorb > 4):
                # print("i %d end gorb rays gorb %d" %(i, rays_gorb))
                gorb_list.append((start_gorb+stop_gorb)/2)
            on_gorb = False
            continue
        if mul_der[i] > threshold and on_gorb:
            rays_gorb+=1
            continue

        if mul_der[i] < -threshold and not on_vpadina:
            on_vpadina = True
            rays_vpadina = 1
            start_vpadina = i
            if (on_gorb):
                stop_gorb = i
                if (rays_gorb > 4):
                    # print("i %d end gorb rays gorb %d" %(i, rays_gorb))
                    gorb_list.append((start_gorb+stop_gorb)/2)
                on_gorb = False
            continue
        if mul_der[i] > -threshold and on_vpadina:
            stop_vpadina = i
            if (rays_vpadina > 4):
                # print("i %d end vpadina rays vpadina %d" %(i, rays_vpadina))
                vpadina_list.append((start_vpadina+stop_vpadina)/2)
            on_vpadina = False
            continue
        if mul_der[i] < threshold and on_vpadina:
            rays_vpadina+=1
            continue
    for i in xrange(len(mul_der)):
        out = 0
        for x in vpadina_list:
            if i == x:
                out=-1
        for x in gorb_list:
            if i == x:
                out=1
        out_list.append(out)        

    
    return out_list

def get_mean(particles):
    """Compute mean position and heading from a given set of particles."""
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    mean_x, mean_y = 0.0, 0.0
    head_x, head_y = 0.0, 0.0
    for p in particles:
        x, y, theta = p.pose
        mean_x += x
        mean_y += y
        head_x += cos(theta)
        head_y += sin(theta)
    n = max(1, len(particles))
    return np.array([mean_x / n, mean_y / n, atan2(head_y, head_x)])

def get_error_ellipse_and_heading_variance(particles, mean):
    """Given a set of particles and their mean (computed by get_mean()),
       returns a tuple: (angle, stddev1, stddev2, heading-stddev) which is
       the orientation of the xy error ellipse, the half axis 1, half axis 2,
       and the standard deviation of the heading."""
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    center_x, center_y, center_heading = mean
    n = len(particles)
    if n < 2:
        return (0.0, 0.0, 0.0, 0.0)

    # Compute covariance matrix in xy.
    sxx, sxy, syy = 0.0, 0.0, 0.0
    for p in particles:
        x, y, theta = p.pose
        dx = x - center_x
        dy = y - center_y
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    cov_xy = np.array([[sxx, sxy], [sxy, syy]]) / (n-1)

    # Get variance of heading.
    var_heading = 0.0
    for p in particles:
        dh = (p.pose[2] - center_heading + pi) % (2*pi) - pi
        var_heading += dh * dh
    var_heading = var_heading / (n-1)

    # Convert xy to error ellipse.
    eigenvals, eigenvects = np.linalg.eig(cov_xy)
    ellipse_angle = atan2(eigenvects[1,0], eigenvects[0,0])

    return (ellipse_angle, sqrt(abs(eigenvals[0])),
            sqrt(abs(eigenvals[1])),
            sqrt(var_heading))

# Takes one scan and subsamples the measurements, so that every sampling'th
# point is taken. Returns a list of (x, y) points in the scanner's
# coordinate system.
def get_subsampled_points(scan, sampling = 10):
    # Subsample from scan
    index_range_tuples = []
    for i in xrange(0, len(scan), sampling):
        index_range_tuples.append( (i, scan[i]) )
    return compute_cartesian_coordinates(index_range_tuples, 0.0)


def print_particles(particles, file_desc):
    # Note this function would more likely be a part of FastSLAM or a base class
    # of FastSLAM. It has been moved here for the purpose of keeping the
    # FastSLAM class short in this tutorial.
    """Prints particles to given file_desc output."""
    if not particles:
        return
    print >> file_desc, "PA",
    for p in particles:
        print >> file_desc, "%.0f %.0f %.3f" % tuple(p.pose),
    print >> file_desc

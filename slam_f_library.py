# This file contains helper functions for Unit D of the SLAM lecture,
# most of which were developed in earlier units.
# Claus Brenner, 11 DEC 2012
from math import sin, cos, pi
from lego_robot import LegoLogfile

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

# Takes one scan and subsamples the measurements, so that every sampling'th
# point is taken. Returns a list of (x, y) points in the scanner's
# coordinate system.
def get_subsampled_points(scan, sampling = 10):
    # Subsample from scan
    index_range_tuples = []
    for i in xrange(0, len(scan), sampling):
        index_range_tuples.append( (i, scan[i]) )
    return compute_cartesian_coordinates(index_range_tuples, 0.0)

# Utility to write a list of cylinders to (one line of) a given file.
# Line header defines the start of each line, e.g. "D C" for a detected
# cylinder or "W C" for a world cylinder.
def write_cylinders(file_desc, line_header, cylinder_list):
    print >> file_desc, line_header,
    for c in cylinder_list:
        print >> file_desc, "%.1f %.1f" % c,
    print >> file_desc
    
# Utility to write a list of error ellipses to (one line of) a given file.
# Line header defines the start of each line.
def write_error_ellipses(file_desc, line_header, error_ellipse_list):
    print >> file_desc, line_header,
    for e in error_ellipse_list:
        print >> file_desc, "%.3f %.1f %.1f" % e,
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
            derivative = (-1*scan[i-3]+9*scan[i-2]-45*scan[i-1]+0*scan[i+0]+45*scan[i+1]-9*scan[i+2]+1*scan[i+3])/(60*1.0*h**1)
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
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0

    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] == -1:
            # Start a new cylinder, independent of on_cylinder.
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
        elif scan_derivative[i] == 1:
            # Save cylinder if there was one.
            if on_cylinder and rays:
                sigma = (2*pi / 660) * rays
                D = sum_depth/rays
                d = 2*D*tan(sigma/2)
                print("d=%d"%d)

                cylinder_list.append((sum_ray/rays, sum_depth/rays))
            on_cylinder = False
        # Always add point, if it is a valid measurement.
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
        #     print ("rays %d "%(rays))
        # print ("i %d onc %d"%(i, on_cylinder))
    return cylinder_list

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
                    # print("i %d end vpadina rays vpadina %d" %(i, rays_vpadina))
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


# This function does all processing needed to obtain the cylinder observations.
# It matches the cylinders and returns distance and angle observations together
# with the cylinder coordinates in the world system, the scanner
# system, and the corresponding cylinder index (in the list of estimated parameters).
# In detail:
# - It takes scan data and detects cylinders.
# - For every detected cylinder, it computes its world coordinate using
#   the polar coordinates from the cylinder detection and the robot's pose,
#   taking into account the scanner's displacement.
# - Using the world coordinate, it finds the closest cylinder in the
#   list of current (estimated) landmarks, which are part of the current state.
#   
# - If there is such a closest cylinder, the (distance, angle) pair from the
#   scan measurement (these are the two observations), the (x, y) world
#   coordinates of the cylinder as determined by the measurement, the (x, y)
#   coordinates of the same cylinder in the scanner's coordinate system,
#   and the index of the matched cylinder are added to the output list.
#   The index is the cylinder number in the robot's current state.
# - If there is no matching cylinder, the returned index will be -1.
def get_observations(scan, jump, min_dist, cylinder_offset,
                     robot,
                     max_cylinder_distance):
    # scan = filter1(scan)

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
                               min_dist)

    # Compute scanner pose from robot pose.
    scanner_pose = (
        robot.state[0] + cos(robot.state[2]) * robot.scanner_displacement,
        robot.state[1] + sin(robot.state[2]) * robot.scanner_displacement,
        robot.state[2])

    # For every detected cylinder which has a closest matching pole in the
    # cylinders that are part of the current state, put the measurement
    # (distance, angle) and the corresponding cylinder index into the result list.
    result = []
    for c in cylinders:
        # Compute the angle and distance measurements.
        angle = LegoLogfile.beam_index_to_angle(c[0])
        distance = c[1] + cylinder_offset
        # Compute x, y of cylinder in world coordinates.
        xs, ys = distance*cos(angle), distance*sin(angle)
        x, y = LegoLogfile.scanner_to_world(scanner_pose, (xs, ys))
        # Find closest cylinder in the state.
        best_dist_2 = max_cylinder_distance * max_cylinder_distance
        best_index = -1
        for index in xrange(robot.number_of_landmarks):
            pole_x, pole_y = robot.state[3+2*index : 3+2*index+2]
            dx, dy = pole_x - x, pole_y - y
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_index = index
        best_index_2 = robot.find_cylinder((distance,angle), float(0.1))
        # Always add result to list. Note best_index may be -1.
        # print(">>> best_index %d"%best_index)
        if (best_index != best_index_2):
            print("best_index %d best_index_2 %d"%(best_index, best_index_2))
        result.append(((distance, angle), (x, y), (xs, ys), best_index))

    return result

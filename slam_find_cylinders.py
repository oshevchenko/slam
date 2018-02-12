# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

# Find the derivative in scan data, ignoring invalid measurements.
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
                    print("i %d end vpadina rays vpadina %d" %(i, rays_vpadina))
                    vpadina_list.append((start_vpadina+stop_vpadina)/2)
                on_vpadina = False
            continue
        if mul_der[i] < threshold and on_gorb:
            stop_gorb = i
            if (rays_gorb > 4):
                print("i %d end gorb rays gorb %d" %(i, rays_gorb))
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
                    print("i %d end gorb rays gorb %d" %(i, rays_gorb))
                    gorb_list.append((start_gorb+stop_gorb)/2)
                on_gorb = False
            continue
        if mul_der[i] > -threshold and on_vpadina:
            stop_vpadina = i
            if (rays_vpadina > 4):
                print("i %d end vpadina rays vpadina %d" %(i, rays_vpadina))
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
            print ("rays %d "%(rays))
        print ("i %d onc %d"%(i, on_cylinder))
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

def filter1(scan):
    scan_filtered = []
    f=[]
    scan_filtered.append(scan[0])
    scan_filtered.append(scan[1])
    for i in xrange(2, len(scan) - 2):
        pyList = [scan[i-2], scan[i-1], scan[i], scan[i+1], scan[i+2]]
        f = sorted(pyList)
        scan_filtered.append(f[2])
    scan_filtered.append(scan[len(scan) - 2])
    scan_filtered.append(scan[len(scan) - 1])
    print("len inp %d len out %d"%(len(scan), len(scan_filtered)))
    return scan_filtered

def filter2(scan):
    scan_filtered = []
    scan_filtered.append(scan[0])
    scan_filtered.append(scan[1])
    for i in xrange(2, len(scan) - 2):
        sum = scan[i-2] + scan[i-1] + scan[i] + scan[i+1] + scan[i+2]
        sum /= 5
        scan_filtered.append(sum)
    scan_filtered.append(scan[len(scan) - 2])
    scan_filtered.append(scan[len(scan) - 1])
    print("len inp %d len out %d"%(len(scan), len(scan_filtered)))
    return scan_filtered

if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robo_scan_777_256.txt")

    # Pick one scan.
    scan = logfile.scan_data[14]
    # scan = logfile.scan_data[16]
    scan_f = filter1(scan)
    scan_f = filter2(scan_f)
    # Find cylinders.
    der = compute_derivative(scan_f, minimum_valid_distance)
    # der111 = compute_derivative111(scan_f, minimum_valid_distance)
    der2 = compute_derivative111(der, 0)
    mul_der=[]
    for i in xrange(len(der2)):
        mul_der.append(der[i]*abs(der2[i]))
    mul_der = filter2(mul_der)
    start_stop = []
    start_stop = convert_to_start_stop(mul_der, depth_jump)
    cylinders = find_cylinders(scan_f, start_stop, depth_jump,
                               minimum_valid_distance)

    # Plot results.
    # plot(scan)
    der = [d*10 for d in der]
    # plot(mul_der)
    plot(der)
    plot(der2)
    plot(scan_f)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()

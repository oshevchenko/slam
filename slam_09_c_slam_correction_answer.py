# EKF SLAM - prediction, landmark assignment and correction.
#
# slam_09_c_slam_correction
# Claus Brenner, 20 JAN 13
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_f_library import get_observations, write_cylinders,\
     write_error_ellipses, filter1, get_subsampled_points


class ExtendedKalmanFilterSLAM:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev,
                 max_error_ellipse):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

        # Currently, the number of landmarks is zero.
        self.number_of_landmarks = 0
        self.out_landmarks =[]
        self.out_landmarks_error_ellipses=[]
        self.max_error_ellipse = max_error_ellipse

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:
            alpha = (r-l)/w
            theta_ = theta + alpha
            rpw2 = l/alpha + w/2.0
            m = array([[1.0, 0.0, rpw2*(cos(theta_) - cos(theta))],
                       [0.0, 1.0, rpw2*(sin(theta_) - sin(theta))],
                       [0.0, 0.0, 1.0]])
        else:
            m = array([[1.0, 0.0, -l*sin(theta)],
                       [0.0, 1.0,  l*cos(theta)],
                       [0.0, 0.0,  1.0]])
        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:
            rml = r - l
            rml2 = rml * rml
            theta_ = theta + rml/w
            dg1dl = w*r/rml2*(sin(theta_)-sin(theta))  - (r+l)/(2*rml)*cos(theta_)
            dg2dl = w*r/rml2*(-cos(theta_)+cos(theta)) - (r+l)/(2*rml)*sin(theta_)
            dg1dr = (-w*l)/rml2*(sin(theta_)-sin(theta)) + (r+l)/(2*rml)*cos(theta_)
            dg2dr = (-w*l)/rml2*(-cos(theta_)+cos(theta)) + (r+l)/(2*rml)*sin(theta_)
            
        else:
            dg1dl = 0.5*(cos(theta) + l/w*sin(theta))
            dg2dl = 0.5*(sin(theta) - l/w*cos(theta))
            dg1dr = 0.5*(-l/w*sin(theta) + cos(theta))
            dg2dr = 0.5*(l/w*cos(theta) + sin(theta))

        dg3dl = -1.0/w
        dg3dr = 1.0/w
        m = array([[dg1dl, dg1dr], [dg2dl, dg2dr], [dg3dl, dg3dr]])
            
        return m

    def predict(self, control):
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        G3 = self.dg_dstate(self.state, control, self.robot_width)
        left, right = control
        left_var = (self.control_motion_factor * left)**2 +\
                   (self.control_turn_factor * (left-right))**2
        right_var = (self.control_motion_factor * right)**2 +\
                    (self.control_turn_factor * (left-right))**2
        control_covariance = diag([left_var, right_var])
        V = self.dg_dcontrol(self.state, control, self.robot_width)
        R3 = dot(V, dot(control_covariance, V.T))
        #print self.state
        N = self.number_of_landmarks
        N2 = N*2

        G = zeros((3+N2,3+N2))
        G[0:3,0:3] = G3
        G[3:,3:] = eye(N2)

        R = zeros((3+N2,3+N2))
        R[0:3,0:3] = R3
        
        # Now enlarge G3 and R3 to accomodate all landmarks. Then, compute the
        # new covariance matrix self.covariance.
        #self.covariance = dot(G3, dot(self.covariance, G3.T)) + R3  # Replace this.
        self.covariance = dot(G, dot(self.covariance, G.T)) + R
        # state' = g(state, control)
        new_state = self.g(self.state[:3,], control, self.robot_width)

        self.state = hstack((new_state[:,],self.state[3:,]))

    def add_landmark_to_state(self, initial_coords):
        """Enlarge the current state and covariance matrix to include one more
           landmark, which is given by its initial_coords (an (x, y) tuple).
           Returns the index of the newly added landmark."""

        # --->>> Put here your new code to augment the robot's state and
        #        covariance matrix.
        #        Initialize the state with the given initial_coords and the
        #        covariance with 1e10 (as an approximation for "infinity".
        # Hints:
        # - If M is a matrix, use M[i:j,k:l] to obtain the submatrix of
        #   rows i to j-1 and colums k to l-1. This can be used on the left and
        #   right side of the assignment operator.
        # - zeros(n) gives a zero vector of length n, eye(n) an n x n identity
        #   matrix.
        # - Do not forget to increment self.number_of_landmarks.
        # - Do not forget to return the index of the newly added landmark. I.e.,
        #   the first call should return 0, the second should return 1.
        index = self.number_of_landmarks
        self.number_of_landmarks +=1

        x1, y1 = initial_coords
        new_landmark = array([x1,y1])

        dim_x,dim_y = self.covariance.shape
        new_matrix = zeros((dim_x+2,dim_y+2))

        new_matrix[0:dim_x,0:dim_y] = self.covariance
        new_matrix[dim_x:,dim_y:] = diag([10**10,10**10])
        
        self.covariance = new_matrix
        #print self.state
        new_state = hstack((self.state[:,], new_landmark[:,]))
        self.state = new_state
        #print self.state

        return index  # Replace this

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        theta = state[2]
        cost, sint = cos(theta), sin(theta)
        dx = landmark[0] - (state[0] + scanner_displacement * cost)
        dy = landmark[1] - (state[1] + scanner_displacement * sint)
        q = dx * dx + dy * dy
        sqrtq = sqrt(q)
        drdx = -dx / sqrtq
        drdy = -dy / sqrtq
        drdtheta = (dx * sint - dy * cost) * scanner_displacement / sqrtq
        dalphadx =  dy / q
        dalphady = -dx / q
        dalphadtheta = -1 - scanner_displacement / q * (dx * cost + dy * sint)

        return array([[drdx, drdy, drdtheta],
                      [dalphadx, dalphady, dalphadtheta]])

    def correct(self, measurement, landmark_index):
        """The correction step of the Kalman filter."""
        # Get (x_m, y_m) of the landmark from the state vector.
        landmark = self.state[3+2*landmark_index : 3+2*landmark_index+2]
        H3 = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        # --->>> Add your code here to set up the full H matrix.
        N = self.number_of_landmarks
        new_H = zeros((2,3+2*N))
        new_H[0:2,0:3] = H3
        new_H[0:2, 3+2*landmark_index : 3+2*landmark_index+2] = new_H[0:2,0:2]*-1

        H = new_H

        # This is the old code from the EKF - no modification necessary!
        Q = diag([self.measurement_distance_stddev**2,
                  self.measurement_angle_stddev**2])
        
        # Psy = linalg.inv(dot(H, dot(self.covariance, H.T)) + Q)
        # print(Psy)
        
        K = dot(self.covariance,
                dot(H.T, linalg.inv(dot(H, dot(self.covariance, H.T)) + Q)))

        innovation = array(measurement) -\
                     self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        self.state = self.state + dot(K, innovation)
        self.covariance = dot(eye(size(self.state)) - dot(K, H),
                              self.covariance)
        self.out_landmarks=[]
        self.out_landmarks_error_ellipses=[]
        for j in xrange(self.number_of_landmarks):
            k = 3 + 2 * j
            error_ellipse = self.get_error_ellipse(self.covariance[k:k+2, k:k+2])
            S_error_ellipse = error_ellipse[1]*error_ellipse[2]
            if (S_error_ellipse < self.max_error_ellipse):
                # print("S=%d"%(error_ellipse[1]*error_ellipse[2]))
                self.out_landmarks.append((self.state[3+2*j], self.state[3+2*j+1]))
                self.out_landmarks_error_ellipses.append(error_ellipse)
        print(">>>>")

    def find_cylinder(self, measurement, threshold):
        # angle and distance to newly found cylinder
        best_index = -1
        print("+++++++++")

        for index in xrange(self.number_of_landmarks):
            landmark = self.state[3+2*index : 3+2*index+2]
            innovation = array(measurement) -\
                     self.h(self.state, landmark, self.scanner_displacement)
            H3 = self.dh_dstate(self.state, landmark, self.scanner_displacement)
            E = self.covariance[0:2,0:2]
            HH = H3[0:2,0:2]
            Q = diag([self.measurement_distance_stddev**2,
                      self.measurement_angle_stddev**2])

            Psy = linalg.inv(dot(HH, dot(self.covariance[0:2,0:2], HH.T)) + Q)
            epsilon = dot(innovation.T, dot(Psy, innovation))
            print(epsilon)
            if (epsilon < threshold):
                best_index = index
        # print(best_index)
        return best_index




    def get_landmarks(self):
        """Returns a list of (x, y) tuples of all landmark positions."""
        # return ([(self.state[3+2*j], self.state[3+2*j+1])
        #          for j in xrange(self.number_of_landmarks)])
        return self.out_landmarks

    def get_landmark_error_ellipses(self):
        """Returns a list of all error ellipses, one for each landmark."""
        # ellipses = []
        # for i in xrange(self.number_of_landmarks):
        #     j = 3 + 2 * i
        #     ellipses.append(self.get_error_ellipse(
        #         self.covariance[j:j+2, j:j+2]))
        # return ellipses
        return self.out_landmarks_error_ellipses

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 0
    ticks_to_mm = 1.037
    robot_width = 200

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 70.0
    cylinder_offset = 5.0
    max_cylinder_distance = 500.0
    max_error_ellipse = 100000;
    # Filter constants.
    #gym_5
    # control_motion_factor = 0.35  # Error in motor control.
    # control_turn_factor = 0.6  # Additional error due to slip when turning.
    #gym_4
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.8  # Additional error due to slip when turning.

    measurement_distance_stddev = 600.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 45. / 180.0 * pi  # Angle measurement error.

    # Arbitrary start position.
    # gym_5
    initial_state = array([500, 2000.0, 350.0 / 180.0 * pi])
    # gym_4
    # initial_state = array([2200, 2000.0, 250.0 / 180.0 * pi])
    # initial_state = array([1000.0, 500.0, 45.0 / 180.0 * pi])

    # Covariance at start position.
    initial_covariance = zeros((3,3))

    # Setup filter.
    kf = ExtendedKalmanFilterSLAM(initial_state, initial_covariance,
                                  robot_width, scanner_displacement,
                                  control_motion_factor, control_turn_factor,
                                  measurement_distance_stddev,
                                  measurement_angle_stddev, max_error_ellipse)

    # Read data.
    logfile = LegoLogfile()
    # logfile.read("robot4_motors.txt")
    # logfile.read("robot4_scan.txt")
    # logfile.read("output_256.txt")
    # logfile.read("robo_scan_777_256.txt")

    logfile.read("gym_5/output")
    logfile.read("gym_5/robo_scan_777_180.txt")

    # logfile.read("gym_4/output.txt")
    # logfile.read("gym_4/robo_scan_777_186.txt")

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the EKF SLAM loop.
    f = open("ekf_slam_correction.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)
        logfile.scan_data[i] = filter1(logfile.scan_data[i])
        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf, max_cylinder_distance)
        for obs in observations:
            measurement, cylinder_world, cylinder_scanner, cylinder_index = obs
            if cylinder_index == -1:
                cylinder_index = kf.add_landmark_to_state(cylinder_world)
            kf.correct(measurement, cylinder_index)

        # End of EKF SLAM - from here on, data is written.

        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(kf.state[0:3] + [scanner_displacement * cos(kf.state[2]),
                                   scanner_displacement * sin(kf.state[2]),
                                   0.0])
        # Write covariance matrix in angle stddev1 stddev2 stddev-heading form.
        e = ExtendedKalmanFilterSLAM.get_error_ellipse(kf.covariance)
        print >> f, "E %f %f %f %f" % (e + (sqrt(kf.covariance[2,2]),))
        # Write estimates of landmarks.
        write_cylinders(f, "W C", kf.get_landmarks())

        # Subsample points.
        pose = (kf.state[0], kf.state[1], kf.state[2])
        subsampled_points = get_subsampled_points(logfile.scan_data[i], 1)
        world_points = [LegoLogfile.scanner_to_world(pose, c)
                        for c in subsampled_points]
        write_cylinders(f, "W P", world_points)

        # Write error ellipses of landmarks.
        write_error_ellipses(f, "W E", kf.get_landmark_error_ellipses())
        # Write cylinders detected by the scanner.
        write_cylinders(f, "D C", [(obs[2][0], obs[2][1])
                                   for obs in observations])

    f.close()

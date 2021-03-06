# Python routines to inspect a ikg LEGO robot logfile.
# Author: Claus Brenner, 28 OCT 2012
from Tkinter import *
import tkFileDialog
from lego_robot import *
from math import sin, cos, pi, ceil
import time
import Image, ImageDraw, ImageTk

# import sys



# The canvas and world extents of the scene.
# Canvas extents in pixels, world extents in millimeters.
canvas_extents = (600, 600)
world_extents = (6000.0, 6000.0)

# The extents of the sensor canvas.
sensor_canvas_extents = canvas_extents

# The maximum scanner range used to scale scan measurement drawings,
# in millimeters.
max_scanner_range = 4000.0

class DrawableObject(object):
    def draw(self, at_step):
        print "To be overwritten - will draw a certain point in time:", at_step

    def background_draw(self):
        print "Background draw."

    @staticmethod
    def get_ellipse_points(center, main_axis_angle, radius1, radius2,
                           start_angle = 0.0, end_angle = 2 * pi):
        """Generate points of an ellipse, for drawing (y axis down)."""
        points = []
        ax = radius1 * cos(main_axis_angle)
        ay = radius1 * sin(main_axis_angle)
        bx = - radius2 * sin(main_axis_angle)
        by = radius2 * cos(main_axis_angle)
        N_full = 40  # Number of points on full ellipse.
        N = int(ceil((end_angle - start_angle) / (2 * pi) * N_full))
        N = max(N, 1)
        increment = (end_angle - start_angle) / N
        for i in xrange(N + 1):
            a = start_angle + i * increment
            c = cos(a)
            s = sin(a)
            x = c*ax + s*bx + center[0]
            y = - c*ay - s*by + center[1]
            points.append((x,y))
        return points


class Trajectory(DrawableObject):
    def __init__(self, points, canvas,
                 world_extents, canvas_extents,
                 standard_deviations = [],
                 point_size2 = 2,
                 background_color = "gray", cursor_color = "red",
                 position_stddev_color = "green", theta_stddev_color = "#ffc0c0"):
        self.points = points
        self.standard_deviations = standard_deviations
        self.canvas = canvas
        self.world_extents = world_extents
        self.canvas_extents = canvas_extents
        self.point_size2 = point_size2
        self.background_color = background_color
        self.cursor_color = cursor_color
        self.position_stddev_color = position_stddev_color
        self.theta_stddev_color = theta_stddev_color
        self.cursor_object = None
        self.cursor_object2 = None
        self.cursor_object3 = None
        self.cursor_object4 = None

    def background_draw(self):
        # if self.points:
        #     p_xy_only = []
        #     for p in self.points:
        #         self.canvas.create_oval(\
        #             p[0]-self.point_size2, p[1]-self.point_size2,
        #             p[0]+self.point_size2, p[1]+self.point_size2,
        #             fill=self.background_color, outline="")
        #         p_xy_only.append(p[0:2])
        #     self.canvas.create_line(*p_xy_only, fill=self.background_color)
        # print("background draw!")
        pass

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
            self.canvas.delete(self.cursor_object2)
            self.cursor_object2 = None
        if at_step < len(self.points):
            p = self.points[at_step]

            p_xy_only = []
            pp = self.points[0]
            p_xy_only.append(pp[0:2])
            for pp in self.points:
                self.canvas.create_oval(\
                    pp[0]-self.point_size2, pp[1]-self.point_size2,
                    pp[0]+self.point_size2, pp[1]+self.point_size2,
                    fill=self.background_color, outline="")
                p_xy_only.append(pp[0:2])
                if pp == p:
                    break
            self.canvas.create_line(*p_xy_only, fill=self.background_color)

            # Draw position (point).
            self.cursor_object = self.canvas.create_oval(\
                p[0]-self.point_size2-1, p[1]-self.point_size2-1,
                p[0]+self.point_size2+1, p[1]+self.point_size2+1,
                fill=self.cursor_color, outline="")
            # Draw error ellipse.
            if at_step < len(self.standard_deviations):
                stddev = self.standard_deviations[at_step]
                # Note this assumes correct aspect ratio.
                factor = canvas_extents[0] / world_extents[0]
                points = self.get_ellipse_points(p, stddev[0],
                    stddev[1] * factor, stddev[2] * factor)
                if self.cursor_object4:
                    self.canvas.delete(self.cursor_object4)
                self.cursor_object4 = self.canvas.create_line(
                    *points, fill=self.position_stddev_color)
            if len(p) > 2:
                # Draw heading standard deviation.
                if at_step < len(self.standard_deviations) and\
                   len(self.standard_deviations[0]) > 3:
                    angle = min(self.standard_deviations[at_step][3], pi)
                    points = self.get_ellipse_points(p, p[2], 30.0, 30.0,
                                                     -angle, angle)
                    points = [p[0:2]] + points + [p[0:2]]
                    if self.cursor_object3:
                        self.canvas.delete(self.cursor_object3)
                    self.cursor_object3 = self.canvas.create_polygon(
                        *points, fill=self.theta_stddev_color)
                # Draw heading.
                self.cursor_object2 = self.canvas.create_line(p[0], p[1],
                    p[0] + cos(p[2]) * 50,
                    p[1] - sin(p[2]) * 50,
                    fill = self.cursor_color)

class ScannerData(DrawableObject):
    def __init__(self, list_of_scans, canvas, canvas_extents, scanner_range):
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.cursor_object = None
        self.extents_x = self.canvas_extents[0]/2
        self.extents_y = self.canvas_extents[1]/2 - 50

        # Convert polar scanner measurements into xy form, in canvas coords.
        # Store the result in self.scan_polygons.
        self.scan_polygons = []
        for s in list_of_scans:
            poly = [ to_sensor_canvas((0,0), canvas_extents, scanner_range) ]
            i = 0
            for m in s:
                angle = LegoLogfile.beam_index_to_angle(i)
                x = m * cos(angle)
                y = m * sin(angle)
                poly.append(to_sensor_canvas((x,y), canvas_extents, scanner_range))
                i += 1
            poly.append(to_sensor_canvas((0,0), canvas_extents, scanner_range))
            self.scan_polygons.append(poly)

    def background_draw(self):
        # Draw x axis.
        self.canvas.create_line(
            self.extents_x, self.extents_y,
            self.extents_x, 20,
            fill="black")
        self.canvas.create_text(
            self.extents_x + 10, 20, text="x" )
        # Draw y axis.
        self.canvas.create_line(
            self.extents_x, self.extents_y,
            20, self.extents_y,
            fill="black")
        self.canvas.create_text(
            20, self.extents_y - 10, text="y" )
        # Draw big disk in the scan center.
        self.canvas.create_oval(
            self.extents_x-20, self.extents_y-20,
            self.extents_x+20, self.extents_y+20,
            fill="gray", outline="")

    def draw(self, at_step):
        if self.cursor_object:
            self.canvas.delete(self.cursor_object)
            self.cursor_object = None
        if at_step < len(self.scan_polygons):
            self.cursor_object = self.canvas.create_polygon(self.scan_polygons[at_step], fill="blue")

class Landmarks(DrawableObject):
    # In contrast other classes, Landmarks stores the original world coords and
    # transforms them when drawing.
    def __init__(self, landmarks, canvas, canvas_extents, world_extents, color = "gray"):
        self.landmarks = landmarks
        self.canvas = canvas
        self.canvas_extents = canvas_extents
        self.world_extents = world_extents
        self.color = color

    def background_draw(self):
        for l in self.landmarks:
            if l[0] =='C':
                x, y = l[1:3]
                ll = to_world_canvas((x - l[3], y - l[3]), self.canvas_extents, self.world_extents)
                ur = to_world_canvas((x + l[3], y + l[3]), self.canvas_extents, self.world_extents)
                self.canvas.create_oval(ll[0], ll[1], ur[0], ur[1], fill=self.color)

    def draw(self, at_step):
        # Landmarks are background only.
        pass
    
class Points(DrawableObject):
    # Points, optionally with error ellipses.
    def __init__(self, points, canvas, color = "red", radius = 5, ellipses = [], ellipse_factor = 1.0):
        self.points = points
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.ellipses = ellipses
        self.ellipse_factor = ellipse_factor
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.points):
            for i in xrange(len(self.points[at_step])):
                # Draw point.
                c = self.points[at_step][i]
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color))
                # Draw error ellipse if present.
                if at_step < len(self.ellipses) and i < len(self.ellipses[at_step]):
                    e = self.ellipses[at_step][i]
                    points = self.get_ellipse_points(c, e[0], e[1] * self.ellipse_factor,
                                                     e[2] * self.ellipse_factor)
                    self.cursor_objects.append(self.canvas.create_line(
                        *points, fill=self.color))

class Walls(DrawableObject):
    # Points, optionally with error ellipses.
    def __init__(self, points_1, points_2, canvas, color = "red", radius = 5):
        self.points_1 = points_1
        self.points_2 = points_2
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.points_1):
            for i in xrange(len(self.points_1[at_step])):
                # Draw point.
                p1 = self.points_1[at_step][i]
                p2 = self.points_2[at_step][i]
                print("p1", p1)
                print("p2", p2)

                self.cursor_objects.append(self.canvas.create_line(p1[0], p1[1], p2[0], p2[1],
                    fill=self.color))

class PolygonPoints(DrawableObject):
    # Points, optionally with error ellipses.
    def __init__(self, points, canvas, color = "red", radius = 5, ellipses = [], ellipse_factor = 1.0, background_image=None, background_d=None):
        self.points = points
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.ellipses = ellipses
        self.ellipse_factor = ellipse_factor
        self.cursor_objects = []
        self.scan_polygon = []
        self.background_image = Image.new('RGBA', canvas_extents)
        # global image
        # global draw_bkg
        # image = Image.new('RGBA', canvas_extents)
        # draw_bkg = ImageDraw.Draw(image)
        # draw_bkg.rectangle(((0, 00), canvas_extents), fill="black")
        self.background_d = ImageDraw.Draw(self.background_image)
        self.background_d.rectangle(((0, 00), canvas_extents), fill="black")

    def background_draw(self):
        self.canvas.create_rectangle(((0, 0), canvas_extents), fill="black")

        # pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.points):
            # self.scan_polygon=[]
            arena_scan_image = Image.new('RGBA', canvas_extents)
            draw = ImageDraw.Draw(arena_scan_image)

            for i in xrange(len(self.points[at_step])):
                # Draw point.
                c = self.points[at_step][i]
                # draw.ellipse((c[0]-2, c[1]-2, c[0]+2, c[1]+2), fill=(0,0,255,100))
            # self.cursor_objects.append(self.canvas.create_polygon(self.points[at_step], fill="blue"))
            # arena_scan_image = Image.new("RGB", canvas_extents, (0, 0, 0))


            # 0,0,0 - black
            # 255,255,255 - white
            draw.polygon(self.points[at_step],fill=(255,255,255,20), outline = (255,255,255,20))

            self.background_image.paste(arena_scan_image, (0,0), arena_scan_image)
            global photo
            global image
            # photo = ImageTk.PhotoImage(self.background_image)
            # photo = ImageTk.PhotoImage(image1)
            # image = Image.open("slam1.bmp")

            image.paste(arena_scan_image, (0,0), arena_scan_image)
            # image = Image.blend(image, arena_scan_image, alpha=0.5)
            # image = Image.alpha_composite(image, arena_scan_image)
            photo = ImageTk.PhotoImage(image)
            
            self.cursor_objects.append(self.canvas.create_rectangle(((0, 0), canvas_extents), fill="white"))
            self.cursor_objects.append(self.canvas.create_image(canvas_extents[0]/2, canvas_extents[1]/2, image=photo))
            # image.save("slam1.bmp")

            # draw.rectangle(((0,0),(10,10)), fill="black", outline = "blue")
            # draw.line(cos_list, red)
            # PIL image can be saved as .png .jpg .gif or .bmp file
            # filename = "my_drawing.bmp"
            filename = "my_drawing.png"
            # arena_scan_image.save(filename)
            self.background_image.save(filename, 'PNG')


# Particles are like points but add a direction vector.
class Particles(DrawableObject):
    def __init__(self, particles, canvas, color = "red", radius = 1.0,
                 vector = 8.0):
        self.particles = particles
        self.canvas = canvas
        self.color = color
        self.radius = radius
        self.vector = vector
        self.cursor_objects = []

    def background_draw(self):
        pass

    def draw(self, at_step):
        if self.cursor_objects:
            map(self.canvas.delete, self.cursor_objects)
            self.cursor_objects = []
        if at_step < len(self.particles):
            for c in self.particles[at_step]:
                self.cursor_objects.append(self.canvas.create_oval(
                    c[0]-self.radius, c[1]-self.radius,
                    c[0]+self.radius, c[1]+self.radius,
                    fill=self.color, outline=self.color))
                self.cursor_objects.append(self.canvas.create_line(
                    c[0], c[1],
                    c[0] + cos(c[2]) * self.vector,
                    c[1] - sin(c[2]) * self.vector,
                    fill = self.color))

# World canvas is x right, y up, and scaling according to canvas/world extents.
def to_world_canvas(world_point, canvas_extents, world_extents):
    """Transforms a point from world coord system to world canvas coord system."""
    x = int(world_point[0] / world_extents[0] * canvas_extents[0])
    y = int(canvas_extents[1] - 1 - world_point[1] / world_extents[1] * canvas_extents[1])
    return (x, y)

# Sensor canvas is "in driving direction", with x up, y left, (0,0) in the center
# and scaling according to canvas_extents and max_scanner_range.
def to_sensor_canvas(sensor_point, canvas_extents, scanner_range):
    """Transforms a point from sensor coordinates to sensor canvas coord system."""
    scale = canvas_extents[0] / 2.0 / scanner_range
    x = int(canvas_extents[0] / 2.0 - sensor_point[1] * scale)
    y = int(canvas_extents[1] / 2.0 - 50 - 1 - sensor_point[0] * scale)
    return (x, y)

def slider_moved(index):
    """Callback for moving the scale slider."""
    i = int(index)
    # global photo
    # global image
    # photo = ImageTk.PhotoImage(self.background_image)
    # photo = ImageTk.PhotoImage(image1)
    # image = Image.open("slam1.bmp")
    # photo = ImageTk.PhotoImage(image)

    # self.cursor_objects.append(self.canvas.create_image(canvas_extents[0]/2, canvas_extents[1]/2, image=photo))

    # world_canvas.create_image(300, 300, image=photo)
    # Call all draw objects.
    for d in draw_objects:
        d.draw(i)
    world_canvas.postscript(file="file_name.ps", colormode='color')
    # Print info about current point.
    info.config(text=logfile.info(i))
def play_pause():
    global playback
    if (playback):
        playback = False
    else:
        playback = True

def add_file():
    filename = tkFileDialog.askopenfilename(filetypes = [("all files", ".*"), ("txt files", ".txt")])
    if filename:
        # If the file is in the list already, remove it (so it will be appended
        # at the end).
        if filename in all_file_names:
            all_file_names.remove(filename)
        all_file_names.append(filename)
        load_data()

def save_scan_arena_bmp():
    pass

def load_data():
    global canvas_extents, sensor_canvas_extents, world_extents, max_scanner_range, num_points
    for filename in all_file_names:
        logfile.read(filename)

    global draw_objects
    draw_objects = []
    scale.configure(to=logfile.size()-1)

    if logfile.world_walls:
        positions = [[to_world_canvas(pos, canvas_extents, world_extents)
                      for pos in cylinders_one_scan]
                      for cylinders_one_scan in logfile.world_walls]
        # cyl_for_pos=[]
        ppp=[]
        test_positions=[]
        for cylinders_one_scan in logfile.world_walls:
            # cyl_for_pos.append(cylinders_one_scan)
            ppp=[]
            for pos in cylinders_one_scan:
                ppp.append(to_world_canvas(pos, canvas_extents, world_extents))
            test_positions.append(ppp)
            
        # print("ppp:")
        # print(ppp)



        # Also setup cylinders if present.
        # Note this assumes correct aspect ratio.
        # factor = canvas_extents[0] / world_extents[0]
        # draw_objects.append(Points(test_positions, world_canvas, "#00f2ff", 1))
        draw_objects.append(PolygonPoints(test_positions, world_canvas, "#00f2ff", 1))

    # Insert: landmarks.
    draw_objects.append(Landmarks(logfile.landmarks, world_canvas, canvas_extents, world_extents))

    # Insert: reference trajectory.
    positions = [to_world_canvas(pos, canvas_extents, world_extents) for pos in logfile.reference_positions]
    draw_objects.append(Trajectory(positions, world_canvas, world_extents, canvas_extents,
        cursor_color="red", background_color="#FFB4B4"))

    # Insert: scanner data.
    draw_objects.append(ScannerData(logfile.scan_data, sensor_canvas,
        sensor_canvas_extents, max_scanner_range))
    # draw_objects.append(ScannerData(logfile.scan_data_without_landmarks, world_canvas,
    #     canvas_extents, max_scanner_range))
    # >>>>>>>>>>>>>
    if logfile.scan_data_without_landmarks:
        positions = [[to_world_canvas(tuple((pos[0] + world_extents[0]/2, pos[1] + world_extents[1]/2) ), canvas_extents, world_extents)
                     for pos in cylinders_one_scan ]
                     for cylinders_one_scan in logfile.scan_data_without_landmarks ]
        draw_objects.append(Points(positions, world_canvas, "#88FF88", 1))

    # Insert: detected cylinders, in scanner coord system.
    if logfile.detected_cylinders:
        positions = [[to_sensor_canvas(pos, sensor_canvas_extents, max_scanner_range)
                     for pos in cylinders_one_scan ]
                     for cylinders_one_scan in logfile.detected_cylinders ]
        draw_objects.append(Points(positions, sensor_canvas, "#88FF88"))

    if logfile.detected_walls:
        positions_1 = []
        positions_2 = []
        for walls_one_scan in logfile.detected_walls:
            p1 = []
            p2 = []
            for W in walls_one_scan:
                P1 = (W[0], W[1])
                P2 = (W[2], W[3])
                print("P1", P1)
                print("P2", P2)

                p1.append(to_sensor_canvas(P1, sensor_canvas_extents, max_scanner_range))
                p2.append(to_sensor_canvas(P2, sensor_canvas_extents, max_scanner_range))
            positions_1.append(p1)
            positions_2.append(p2)
        draw_objects.append(Walls(positions_1, positions_2, sensor_canvas, "#DC23C5"))


    # Insert: world objects, cylinders and corresponding world objects, ellipses.
    if logfile.world_cylinders:
        positions = [[to_world_canvas(pos, canvas_extents, world_extents)
                      for pos in cylinders_one_scan]
                      for cylinders_one_scan in logfile.world_cylinders]
        # Also setup cylinders if present.
        # Note this assumes correct aspect ratio.
        factor = canvas_extents[0] / world_extents[0]
        draw_objects.append(Points(positions, world_canvas, "#DC23C5",
        # draw_objects.append(Points(positions, world_canvas, "#000000",
                                   ellipses = logfile.world_ellipses,
                                   ellipse_factor = factor))
   
    # Insert: detected cylinders, transformed into world coord system.
    if logfile.detected_cylinders and logfile.filtered_positions and \
        len(logfile.filtered_positions[0]) > 2:
        positions = []
        for i in xrange(min(len(logfile.detected_cylinders), len(logfile.filtered_positions))):
            this_pose_positions = []
            pos = logfile.filtered_positions[i]
            dx = cos(pos[2])
            dy = sin(pos[2])
            for pole in logfile.detected_cylinders[i]:
                x = pole[0] * dx - pole[1] * dy + pos[0]
                y = pole[0] * dy + pole[1] * dx + pos[1]
                p = to_world_canvas((x,y), canvas_extents, world_extents)
                this_pose_positions.append(p)
            positions.append(this_pose_positions)
        draw_objects.append(Points(positions, world_canvas, "#88FF88"))

    if logfile.detected_walls and logfile.filtered_positions and \
        len(logfile.filtered_positions[0]) > 2:
        positions_1 = []
        positions_2 = []

        for i in xrange(min(len(logfile.detected_walls), len(logfile.filtered_positions))):
            pos = logfile.filtered_positions[i]
            dx = cos(pos[2])
            dy = sin(pos[2])
            p1 = []
            p2 = []
            for W in logfile.detected_walls[i]:
                P = (W[0], W[1])
                x = P[0] * dx - P[1] * dy + pos[0]
                y = P[0] * dy + P[1] * dx + pos[1]
                p1.append(to_world_canvas((x,y), canvas_extents, world_extents))
                P = (W[2], W[3])
                x = P[0] * dx - P[1] * dy + pos[0]
                y = P[0] * dy + P[1] * dx + pos[1]
                p2.append(to_world_canvas((x,y), canvas_extents, world_extents))
            positions_1.append(p1)
            positions_2.append(p2)
        draw_objects.append(Walls(positions_1, positions_2, world_canvas, "#DC23C5"))


    # Insert: particles.
    if logfile.particles:
        positions = [
            [(to_world_canvas(pos, canvas_extents, world_extents) + (pos[2],))
             for pos in particles_one_scan]
             for particles_one_scan in logfile.particles]
        draw_objects.append(Particles(positions, world_canvas, "#80E080"))

    # Insert: filtered trajectory.
    if logfile.filtered_positions:
        num_points = len(logfile.filtered_positions)
        if len(logfile.filtered_positions[0]) > 2:
            positions = [tuple(list(to_world_canvas(pos, canvas_extents, world_extents)) + [pos[2]]) for pos in logfile.filtered_positions]
        else:
            positions = [to_world_canvas(pos, canvas_extents, world_extents) for pos in logfile.filtered_positions]
        # If there is error ellipses, insert them as well.
        draw_objects.append(Trajectory(positions, world_canvas, world_extents, canvas_extents,
            standard_deviations = logfile.filtered_stddev,
            cursor_color="blue", background_color="lightblue",
            position_stddev_color = "#8080ff", theta_stddev_color="#c0c0ff"))

    # Start new canvas and do all background drawing.
    world_canvas.delete(ALL)
    sensor_canvas.delete(ALL)
    for d in draw_objects:
        d.background_draw()
    global global_counter
    global_counter = 0

# Main program.
if __name__ == '__main__':
    # PIL create an empty image and draw object to draw on
    # memory only, not visible
    white = (255, 255, 255)
    black = (0, 0, 0)
    blue = (0, 0, 255)
    red = (255, 0, 0)
    green = (0,128,0)

    width = 400
    height = 300
    center = height//2
    x_increment = 1
    # width stretch
    x_factor = 0.04
    # height stretch
    y_amplitude = 80
    global image1, image
    global draw
    image1 = Image.new("RGBA", (width, height), white)

    draw = ImageDraw.Draw(image1)
    # do the PIL image/draw (in memory) drawings
    # str1 = "sin(x)=blue  cos(x)=red"
    # draw.text((10, 20), str1, black)
# create the coordinate list for the sin() curve
# have to be integers for Tkinter
    sine_list = []
    for x in range(400):
        # x coordinates
        sine_list.append(x * x_increment)
        # y coordinates
        sine_list.append(int(sin(x * x_factor) * y_amplitude) + center)
    # create the coordinate list for the cos() curve
    cos_list = []
    for x in range(400):
        # x coordinates
        cos_list.append(x * x_increment)
    # y coordinates
    cos_list.append(int(cos(x * x_factor) * y_amplitude) + center)
    draw.line([0, center, width, center], green)
    draw.line(sine_list, blue)
    poly = []
    poly.append(tuple([0,0]))
    poly.append(tuple([0,10]))
    poly.append(tuple([10,10]))
    draw.polygon(poly,fill="black", outline = "blue")
    # draw.rectangle(((0,0),(10,10)), fill="black", outline = "blue")
    # draw.line(cos_list, red)
    # PIL image can be saved as .png .jpg .gif or .bmp file
    # filename = "my_drawing.bmp"

    # image1.save(filename)
    num_points=0
    # Construct logfile (will be read in load_data()).
    logfile = LegoLogfile()

    # Setup GUI stuff.
    root = Tk()
    frame1 = Frame(root)
    frame1.pack()
    world_canvas = Canvas(frame1,width=canvas_extents[0],height=canvas_extents[1],bg="white")

    # global image
    # image = Image.open("slam.bmp")
    global image
    image = Image.new('RGBA', canvas_extents)
    global draw_bkg
    draw_bkg = ImageDraw.Draw(image)
    draw_bkg.rectangle(((0, 00), canvas_extents), fill="black")

    global photo
    photo = ImageTk.PhotoImage(image)
    world_canvas.create_image(canvas_extents[0]/2, canvas_extents[1]/2, image=photo)
    # world_canvas.create_image(100, 100, image=photo)
    world_canvas.pack(side=LEFT)
    world_canvas.postscript(file="file_name.ps", colormode='color')
    sensor_canvas = Canvas(frame1,width=sensor_canvas_extents[0],height=sensor_canvas_extents[1]/2,bg="white")
    sensor_canvas.pack(anchor=N,side=RIGHT)
    scale = Scale(root, orient=HORIZONTAL, command = slider_moved)
    scale.pack(fill=X)
    info = Label(root)
    info.pack()
    frame2 = Frame(root)
    frame2.pack()
    load = Button(frame2,text="Load (additional) logfile",command=add_file)
    load.pack(side=LEFT)
    play = Button(frame2,text="Play/Pause",command=play_pause)
    play.pack(side=LEFT)

    reload_all = Button(frame2,text="Reload all",command=load_data)
    reload_all.pack(side=RIGHT)

    # The list of objects to draw.
    draw_objects = []

    # Ask for file.
    all_file_names = []
    add_file()

    # root.mainloop()
    millis = int(round(time.time() * 1000))
    print("current_milli_time = %d"%(millis))
    global global_counter, num_points, playback
    playback=False
    global_counter=0
    while True:
        root.update_idletasks()
        root.update()
        if (playback):
            millis_new = int(round(time.time() * 1000))
            if (global_counter < num_points):
                if (millis_new - millis)>500:
                    slider_moved(global_counter)
                    print("global_counter %d num_points %d"%(global_counter, num_points))

                    millis = millis_new
                    global_counter += 1
            else:
                playback=False

    root.destroy()

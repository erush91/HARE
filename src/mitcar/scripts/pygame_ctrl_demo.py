'''
this guy deserves a beer
https://github.com/ioarun/ai-for-robotics-udacity/blob/master/visual-implementation/racetrack-control.py
'''

import pygame
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt

import time
start_time = time.time()
# code chunk here
print(time.time() - start_time)

display_width = 600
display_height = 600

world_size = display_width

red = (200, 0, 0)
blue = (0, 0, 255)
green = (0, 155, 0)
yellow = (200, 200, 0)
white = (255, 255, 255)
black = (0, 0, 0)
grey = (67, 70, 75)

car_length = 80.0
car_width = 60.0

wheel_length = 20
wheel_width = 6

max_steering_angle = pi/8
max_speed = 4

cam_fixed_theta = pi/4 # positive, wrt to car forward and to the right
cam_FOV_angle = np.deg2rad(80)
cam_dist_points = 50 # number of measurement points
cam_max_dist = 400 # dist in pixels

# car_img = pygame.image.load("car60_40.png")


origin = (display_width/2, display_height/2)


class robot:
	def __init__(self):
		self.x = random.random() * world_size
		self.y = random.random() * world_size
		self.orientation = random.random() * 2.0 * pi
		self.steering_angle = 0.0
		self.steering_drift = 0.0
		self.forward_noise = 0.0
		self.turn_noise = 0.0
		self.sense_noise = 0.0

	def set(self, x, y, orientation, steering_angle):
		# if x >= world_size or x < 0:
		# 	raise ValueError, 'X coordinate out of bound'
		# if y >= world_size or y < 0:
		# 	raise ValueError, 'Y coordinate out of bound'
		# if orientation >= 2 * pi or orientation < 0:
		# 	raise ValueError, 'Orientation must be in [0..2pi]'
		# if abs(steering_angle) > max_steering_angle:
		# 	raise ValueError, 'Exceeding max steering angle'

		self.x = x
		self.y = y
		self.orientation = orientation
		self.steering_angle = steering_angle

	def move(self, turn, forward):
		theta = self.orientation  # initial orientation
		alpha = turn  # steering angle
		dist = forward  # distance to be moved
		length = car_length  # length of the robot
		# if abs(alpha) > max_steering_angle:
		# 	raise ValueError, 'Exceeding max steering angle'
		#
		# if dist < 0.0:
		# 	raise ValueError, 'Moving backwards is not valid'

		# in local coordinates of robot
		beta = (dist / length) * tan(alpha)  # turning angle
		# print degrees(beta)
		_x = _y = _theta = 0.0
		if beta > 0.001 or beta < -0.001:
			radius = dist / beta  # turning radius
			cx = self.x - sin(theta) * radius  # center of the circle
			cy = self.y - cos(theta) * radius  # center of the circle

			# Uncomment to see the center of the circle the robot is going about
			# pygame.draw.circle(screen, red, (int(cx), int(cy)), 5)
			# pygame.display.update()

			# in global coordinates of robot
			_x = cx + sin(theta + beta) * radius
			_y = cy + cos(theta + beta) * radius
			_theta = (theta + beta) % (2 * pi)

		else:  # straight motion
			_x = self.x + dist * cos(theta)
			_y = self.y - dist * sin(theta)
			_theta = (theta + beta) % (2 * pi)

		self.x = _x
		# self.y = _y
		self.orientation = _theta
		self.steering_angle = alpha

		self.x %= world_size
		# self.y %= world_size




def draw_rect(center, corners, rotation_angle, color):
    c_x = center[0]
    c_y = center[1]
    delta_angle = rotation_angle
    rotated_corners = []

    for p in corners:
        temp = []
        length = sqrt((p[0] - c_x) ** 2 + (c_y - p[1]) ** 2)
        angle = atan2(c_y - p[1], p[0] - c_x)
        angle += delta_angle
        temp.append(c_x + length * cos(angle))
        temp.append(c_y - length * sin(angle))
        rotated_corners.append(temp)

    # draw rectangular polygon --> car body
    rect = pygame.draw.polygon(screen, color,
                               (rotated_corners[0], rotated_corners[1], rotated_corners[2], rotated_corners[3]))


def draw_robot(robot):
	car_x = robot.x
	car_y = robot.y
	orientation = robot.orientation
	steering_angle = robot.steering_angle

	p1 = [car_x-car_length/4,car_y-car_width/2]
	p2 = [car_x+(0.75*car_length),car_y-car_width/2]
	p3 = [car_x+(0.75*car_length),car_y+car_width/2]
	p4 = [car_x-car_length/4,car_y+car_width/2]

	# car body
	draw_rect([car_x, car_y], [p1, p2, p3, p4], orientation, yellow)

	# heading direction
	# h = [car_x+car_length/2,car_y]
	# length = car_length/2
	# angle = atan2(car_y - h[1], h[0] - car_x)
	# angle += orientation
	# h[0] = car_x + length*cos(angle)
	# h[1] = car_y - length*sin(angle)

	# wheels
	# rotate center of wheel1
	w1_c_x = car_x
	w1_c_y = car_y - car_width/3
	length = sqrt((w1_c_x - car_x)**2 + (car_y - w1_c_y)**2)
	angle = atan2(car_y - w1_c_y, w1_c_x - car_x)
	angle += orientation
	w1_c_x = car_x + length*cos(angle)
	w1_c_y = car_y - length*sin(angle)

	# draw corners of wheel1
	w1_p1 = [w1_c_x-wheel_length/2, w1_c_y-wheel_width/2]
	w1_p2 = [w1_c_x+wheel_length/2, w1_c_y-wheel_width/2]
	w1_p3 = [w1_c_x+wheel_length/2, w1_c_y+wheel_width/2]
	w1_p4 = [w1_c_x-wheel_length/2, w1_c_y+wheel_width/2]
	draw_rect([w1_c_x, w1_c_y], [w1_p1, w1_p2, w1_p3, w1_p4], orientation, black)





	w2_c_x = car_x + car_length/2
	w2_c_y = car_y - car_width/3
	length = sqrt((w2_c_x - car_x)**2 + (car_y - w2_c_y)**2)
	angle = atan2(car_y - w2_c_y, w2_c_x - car_x)
	angle += orientation
	w2_c_x = car_x + length*cos(angle)
	w2_c_y = car_y - length*sin(angle)

	w2_p1 = [w2_c_x-wheel_length/2, w2_c_y-wheel_width/2]
	w2_p2 = [w2_c_x+wheel_length/2, w2_c_y-wheel_width/2]
	w2_p3 = [w2_c_x+wheel_length/2, w2_c_y+wheel_width/2]
	w2_p4 = [w2_c_x-wheel_length/2, w2_c_y+wheel_width/2]
	draw_rect([w2_c_x, w2_c_y], [w2_p1, w2_p2, w2_p3, w2_p4], steering_angle + orientation, black)
	# rect = pygame.draw.polygon(screen, black, (w2_p1,w2_p2,w2_p3,w2_p4))


	w3_c_x = car_x + car_length/2
	w3_c_y = car_y + car_width/3
	length = sqrt((w3_c_x - car_x)**2 + (car_y - w3_c_y)**2)
	angle = atan2(car_y - w3_c_y, w3_c_x - car_x)
	angle += orientation
	w3_c_x = car_x + length*cos(angle)
	w3_c_y = car_y - length*sin(angle)

	w3_p1 = [w3_c_x-wheel_length/2, w3_c_y-wheel_width/2]
	w3_p2 = [w3_c_x+wheel_length/2, w3_c_y-wheel_width/2]
	w3_p3 = [w3_c_x+wheel_length/2, w3_c_y+wheel_width/2]
	w3_p4 = [w3_c_x-wheel_length/2, w3_c_y+wheel_width/2]
	draw_rect([w3_c_x, w3_c_y], [w3_p1, w3_p2, w3_p3, w3_p4], steering_angle + orientation, black)
	# rect = pygame.draw.polygon(screen, black, (w3_p1,w3_p2,w3_p3,w3_p4))



	w4_c_x = car_x
	w4_c_y = car_y + car_width/3
	length = sqrt((w4_c_x - car_x)**2 + (car_y - w4_c_y)**2)
	angle = atan2(car_y - w4_c_y, w4_c_x - car_x)
	angle += orientation
	w4_c_x = car_x + length*cos(angle)
	w4_c_y = car_y - length*sin(angle)

	w4_p1 = [w4_c_x-wheel_length/2, w4_c_y-wheel_width/2]
	w4_p2 = [w4_c_x+wheel_length/2, w4_c_y-wheel_width/2]
	w4_p3 = [w4_c_x+wheel_length/2, w4_c_y+wheel_width/2]
	w4_p4 = [w4_c_x-wheel_length/2, w4_c_y+wheel_width/2]
	draw_rect([w4_c_x, w4_c_y], [w4_p1, w4_p2, w4_p3, w4_p4], orientation, black)

	# pygame.draw.line(screen, red, (h[0], h[1]),(int(car_x), int(car_y)), 1)

	# draw axle
	pygame.draw.line(screen, black, (w1_c_x, w1_c_y),(w4_c_x, w4_c_y), 1)

	# draw mid of axle
	pygame.draw.circle(screen, red, (int(car_x), int(car_y)), 3)


# def draw_dashed_line(surf, color, start_pos, end_pos, width=1, dash_length=10):
#     orig = Point(start_pos)
#     target = Point(end_pos)
#     displacement = target - orig
#     length = len(displacement)
#     slope = displacement/length
#
#     for index in range(0, length/dash_length, 2):
#         start = origin + (slope *    index    * dash_length)
#         end   = origin + (slope * (index + 1) * dash_length)
#         pygame.draw.line(surf, color, start.get(), end.get(), width)

def get_cam_depth():
    cam_cent = (robot.orientation - cam_fixed_theta) % (2*pi)
    sweep_start = cam_cent + cam_FOV_angle/2
    sweep_stop = cam_cent - cam_FOV_angle/2
    ray_angs = np.linspace(sweep_start,sweep_stop,cam_dist_points)
    ray_angs = ray_angs % (2*pi)
    x_dist = wall_x - robot.x

    truth_array = np.ones(ray_angs.shape).astype(bool)
    truth_array[np.where((ray_angs >pi/2-.001) & (ray_angs<pi/2+.001))] = 0
    truth_array[np.where((ray_angs < 0.001) & (ray_angs > -0.001))] = 0
    ys = np.tan(ray_angs, out=None, where=truth_array)
    # ys = ys[np.where(ys == pi/2)]
    dist_array = np.sqrt(x_dist ** 2 + (ys * x_dist) ** 2)
    dist_array[dist_array > cam_max_dist] = cam_max_dist

    i = 0
    tt = False
    for dit in dist_array:
        if dit == cam_max_dist:
            end_x = robot.x + np.cos(ray_angs[i], out=None, where=truth_array[i])*cam_max_dist
            end_y = robot.y - np.sin(ray_angs[i], out=None, where=truth_array[i])*cam_max_dist
        elif dit < cam_max_dist:
            end_x = wall_x
            end_y = robot.y - ys[i] * x_dist
            tt = True
        if tt: pygame.draw.line(screen, green, (robot.x, robot.y), (int(end_x), int(end_y)), 1)
        else: pygame.draw.line(screen, red, (robot.x, robot.y), (int(end_x), int(end_y)), 1)

        i += 1
    return(dist_array)

setpt_dist = int(display_width/2) + 50
wall_x = int(display_width*7/8)

pygame.init()

screen = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("Moving robot")
clock = pygame.time.Clock()

screen.fill(white)

robot = robot()
speed = 5
orientation = 90.0
steering_angle = 0.0

err_hist = []
err_hist_window = 25

x_positions = np.linspace(10, display_width, cam_dist_points)
x_positions = x_positions.astype(int)

robot.set(300, 400,radians(orientation), steering_angle)
delta_forward = 0.0
delta_steer = 0.0
exit = False
while exit == False:
    screen.fill(white)
    screen.fill(black,(wall_x,0,display_width/8,display_height))

    # screen.blit(track_img, (0, 100))
    # Uncomment following to see the exact racetrack
    #draw_track(250, 400, 200, grey)
    # draw_path(path, red)

    draw_robot(robot)

    # pygame.draw.line(screen, green, (display_width/2, 0), (display_width/2, display_height), 1)
    # pygame.draw.line(screen, black, (0, display_height/2), (display_width, display_height/2), 1)

    pygame.draw.line(screen,red,(setpt_dist,0), (setpt_dist,display_height),1)
    # draw_dashed_line(screen, red, (set_dist,0), (set_dist,display_height))



    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit = True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                delta_steer = radians(.5)
            elif event.key == pygame.K_RIGHT:
                delta_steer = -radians(.5)
            elif event.key == pygame.K_UP:
                delta_forward = .10
            elif event.key == pygame.K_DOWN:
                delta_forward = -.10
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT:
                delta_steer = 0.0
            if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                delta_forward = 0.0

    steering_angle += delta_steer
    speed += delta_forward

    if speed > max_speed: speed = max_speed
    elif speed < -max_speed: speed = -max_speed
    if steering_angle > max_steering_angle: steering_angle = max_steering_angle
    elif steering_angle < -max_steering_angle: steering_angle = -max_steering_angle

    # === CONTROLLER ===

    RS_dists = get_cam_depth()

    i = 0
    for meas in RS_dists:
        pygame.draw.circle(screen, red, (x_positions[i], display_height - int(RS_dists[i])), 3)
        i += 1
    # print('dist magnitude: ',np.mean(RS_dists))

    translation_err = (290-np.mean(RS_dists))
    err_hist.append(translation_err)
    if len(err_hist) >= err_hist_window:
        err_hist.pop(0)
        d_inp = -0.0003*np.mean(err_hist)
    else:
        d_inp = 0
    # d_inp = 0

    if speed > 0.1: i_inp = 0.00001*translation_err
    else: i_inp = 0
    # if abs(steering_angle) < 0.1 and abs(translation_err) < 50: i_inp = 0.0001*translation_err
    # else: i_inp = 0
    # i_inp = 0

    auto_steer = 0.0003*translation_err+d_inp + i_inp
    steering_angle += auto_steer
    # print('P gain: ', 0.0001*(290-np.mean(RS_dists)))

    print('translation error: ',translation_err, 'steer: ',steering_angle)
    robot.move(steering_angle, speed)
    
    # ___ CTRL END ___

    # print(max(RS_dists))
    # print(speed)

    pygame.display.update()
    clock.tick(100)

pygame.quit()


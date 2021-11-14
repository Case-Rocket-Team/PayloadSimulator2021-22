import matplotlib.pyplot as plt
from random import random
from math import pi, exp
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

'''
Kyler | 10/26/21 | Began writing fuzzifier
'''

# Todo: update as a geoid
def distance_to_degrees(distance):
	# note: 6371000m is the radius of the earth
	return (distance * 360) / (2 * pi * 6371000)


def altitude_to_pressure(altitude):
	# https://math24.net/barometric-formula.html
	return 101325 * exp(- (0.02896 * 9.8) / (8.3143 * 288.15))


def fuzzifier(pos, accel,
			  gps_noise=0.01, imu_noise=0.01, altimeter_noise=0.01):
	"""simulates sensor output by adding noise and formatting into string.

	Args:
		pos (Vec3): an array containing the absolute cartesian position of the craft
		accel ([type]): an array containing the cartesian acceleration vector of the craft
		gps_noise (float, optional): The gps noise to apply. Defaults to 0.01.
		imu_noise (float, optional): the imu noise to apply. Defaults to 0.01.
		altimeter_noise (float, optional): the altimeter noise to apply. Defaults to 0.01.
		pos (Vec3): an array containing the absolute cartesian position of the craft in meters
		accel (Vec3): an array containing the cartesian acceleration vector of the craft in meters / second^2
		gps_noise (float, optional): The gps noise to apply. Defaults to 0.01, or 1%.
		imu_noise (float, optional): the imu noise to apply. Defaults to 0.01, or 1%.
		altimeter_noise (float, optional): the altimeter noise to apply. Defaults to 0.01, or 1%.

	Returns:
		gps: the gps longitude/latitude
		imu: the imu acceleration
		pressure: the altimeter pressure
	"""

	"""calculating GPS"""
	# long, lat of Ben's house
	gps_reference = [40.897800, -81.713250]
	gps = gps_reference + distance_to_degrees(pos)

	gps_error = gps_noise * (random() * 2 - 1)
	gps = gps * (1 + gps_error)

	"""calculating imu"""
	# Creates error for a value -1 < error < 1
	imu_error = imu_noise * (random() * 2 - 1)
	imu = accel * (1 + imu_error)

	"""calculating pressure"""
	altitude_error = altimeter_noise * (random() * 2 - 1)
	altitude = pos[2] * (1 + altitude_error)

	pressure = altitude_to_pressure(altitude)

	return gps, imu, pressure

def graph_data(sim_kinetics, dt, noise_data, pred_kinetics, path, **graph_params):
	"""creates a series of graphs for important data
		#TODO: decide what graphs to show

	Args:
		sim_kinetics (Vec3[][]): the simulated kinetics (pos,vel,accel) for the course of the simulation.
		noise_data (Vec3[][]): the noisy data (gps,imu,pressure) for the course of the simulation.
		pred_kinetics (Vec3[][]): the simulated kinetics (pos,vel,accel) for the course of the simulation.
		path (Vec3): the set of waypoints used throughout the simulation.
	"""
	time = [0]
	x_pos = []
	y_pos = []
	z_pos = []
	for n in range(0, len(sim_kinetics[0])):
		x_pos.append(sim_kinetics[0][n][0])
		y_pos.append(sim_kinetics[0][n][1])
		z_pos.append(sim_kinetics[0][n][2])

	for i in range(0, len(sim_kinetics[0])):
		time.append(time[i] + dt)

	plt.plot(x_pos, z_pos)
	plt.xlabel("X Position (m)")
	plt.ylabel("Z Position (m)")
	plt.title("Z vs X Position")

	plt.figure()
	plt.plot(x_pos, y_pos)
	plt.xlabel("X Position (m)")
	plt.ylabel("Y Position (m)")
	plt.title("Y vs X Position")

	fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
	ax.plot(x_pos, y_pos, z_pos)
	ax.set_title("Parafoil Path with a P Controller")
	ax.set_xlabel("Downwind X (m)")
	ax.set_ylabel("Crosswind Y (m)")
	ax.set_zlabel("Altitude Z (m)")

	plt.show()


def servo_math(accel_hat,max_turn):
	"""calculates servo movement knowing desired acceleration direction.
		Note: for this sim, calculate the magnitude of acceleration the craft can take

	Args:
		accel_hat (Vec3): the unit acceleration
		max_turn (float): the maximum turning radius

	Returns:
		sigma_dot: the applied acceleration
	"""

	return sigma_dot
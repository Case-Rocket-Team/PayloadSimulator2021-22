import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

_gravity = 9.81

def fuzzifier(pos, accel,
	gps_noise=0.01,imu_noise=0.01,altimeter_noise=0.01):
	"""simulates sensor output by adding noise and formatting into string.

	Args:
		pos (Vec3): an array containing the absolute cartesian position of the craft
		accel ([type]): an array containing the cartesian acceleration vector of the craft
		gps_noise (float, optional): The gps noise to apply. Defaults to 0.01.
		imu_noise (float, optional): the imu noise to apply. Defaults to 0.01.
		altimeter_noise (float, optional): the altimeter noise to apply. Defaults to 0.01.

	Returns:
		gps: the gps longitude/latitude
		imu: the imu acceleration
		pressure: the altimeter pressure
	"""
	
	
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


def servo_math(heading, max_turn):
	"""calculates servo movement knowing desired acceleration direction.
		Note: for this sim, calculate the magnitude of acceleration the craft can take

	Args:
		heading (Vec3): an array containing the heading of the craft in the format [azimuth, bank_angle, glide_angle]
		max_turn (float): the maximum turning radius

	Returns:
		servo_angle: the applied acceleration
	"""

	return servo_angle


def convert_bank_to_deflect(heading, vel, span):
	"""
	Uses Steven Lingards Basic Analsyis of Ram Air Parachute

	Args:
		heading (Vec3): an array containing the heading of the craft in the format [azimuth, bank_angle, glide_angle]
		vel (Vec3): an array containing the caretesian velocity of the craft.
		span (Float): spanwise width of the wing

	Returns:
		deflect_angle: the deflection angle of the wing
	"""
	vel_mag = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
	# r=\frac{sin(\phi )*g}{V*cos(\gamma )}
	turn_rate = (math.sin(heading[1]) * _gravity) / (vel_mag * math.cos(heading[2]))
	# \delta _{a}=\frac{r*b}{0.625*V}
	deflect_angle = (turn_rate * span) / (0.625 * vel_mag)

	return deflect_angle

def convert_deflect_to_servo(heading, vel, span):
	deflect_angle = convert_bank_to_deflect(heading, vel, span)
	
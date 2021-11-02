import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

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
	for n in range(0, len(sim_kinetics)):
		x_pos.append(sim_kinetics[n][0])
		y_pos.append(sim_kinetics[n][1])
		z_pos.append(sim_kinetics[n][2])

	for i in range(0, len(sim_kinetics)):
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
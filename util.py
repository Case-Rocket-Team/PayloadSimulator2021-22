import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

# Geometric or environment constants
# TODO: Ben get values for these constants from the solidworks model
_gravity = 9.81
_effective_cord_length = 1.08 # l + L, overall cord length - the part inside the risers
_por_to_slider_distance = 0.13 # d sourced from 
_servo_arm_length = 0.02 # r
_parafoil_span = 1.0 # b in Steven Lingards Basic Analysis of Ram Air Parachute eq. 2.41
_yaw_based_constant = 0.625 # based on Steven Lingards Basic Analysis of Ram Air Parachute, page 46, eq. 2.41
_cos_glide_angle = math.cos(-0.35) # considered constant for purposes of angle calculation, cos(glide_angle)
_parafoil_to_slider_distance = 0.58 # vertical drop, H
_flap_length = 0.167 # approximated as constant to avoid nightmare math, A
_h_angle_servo_to_slider = math.radians(67.4) # alpha (67.4 deg.)

# Calculated constants for computational speed
_k1 = -_effective_cord_length**2 + _por_to_slider_distance**2 + _servo_arm_length**2
_k2 = 2 * _por_to_slider_distance * _servo_arm_length
_k3 = _gravity * _parafoil_span / (_yaw_based_constant * _cos_glide_angle)


def fuzzifier(pos, accel, gps_noise=0.01, imu_noise=0.01, altimeter_noise=0.01):
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
		List of Graphs:
			Z vs X over time
			Y vs X over time
			XYZ over time

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
	glide_angles = []
	bank_angles = []
	for n in range(0, len(sim_kinetics[0])):
		x_pos.append(sim_kinetics[0][n][0])
		y_pos.append(sim_kinetics[0][n][1])
		z_pos.append(sim_kinetics[0][n][2])
		glide_angles.append(math.degrees(sim_kinetics[1][n][2]))
		bank_angles.append(math.degrees(sim_kinetics[1][n][1]))

	for i in range(0, len(sim_kinetics[0])-1):
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

	plt.figure()
	plt.plot(time, glide_angles)
	plt.xlabel("Time (s)")
	plt.ylabel("Glide Angle (deg)")
	plt.title("Glide Angle vs Time")

	plt.figure()
	plt.plot(time, bank_angles)
	plt.xlabel("Time (s)")
	plt.ylabel("Bank Angle (deg)")
	plt.title("Bank Angle vs Time")

	plt.figure()
	plt.plot(glide_angles, bank_angles)
	plt.xlabel("Glide Angle (deg)")
	plt.ylabel("Bank Angle (deg)")
	plt.title("Bank Angle vs Glide Angle")

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

def convert_deflect_to_servo(heading, vel):
	"""This function takes the current bank angle, calcuates the desired deflection angle of the 
		parafoil, and converts to the servo angle needed to achieve that deflection angle.


	Args:
		heading ([Vec3]): an array containing the heading of the craft in the format [azimuth, bank_angle, glide_angle] 
		vel ([Vec3]): an array containing the velocity of the craft in the format [v_x, v_y, v_z]
	"""
	bank_angle = heading[1] # save bank angle for clarity
	deflect_angle = math.sin(bank_angle * _k3 / np.linalg.norm(vel) ** 2) # calculate deflect angle from bank_angle
	# print(deflect_angle)
	cosine_total_angle = ((_parafoil_to_slider_distance - _flap_length * math.sin(deflect_angle)) ** 2 + _k1) / _k2 # breaking up the function a bit for clarity
	# print(cosine_total_angle)
	# final calculation
	servo_angle = math.acos(cosine_total_angle) - _h_angle_servo_to_slider

	return servo_angle

# print("k1: " + str(_k1))
# print("k2: " + str(_k2))
# print("k3: " + str(_k3))



def exp_moving_avg(x_current, decay_rate, weight_sum, weight_count):
	weight_sum = x_current + (1 - decay_rate) * weight_sum
	weight_count = 1 + (1 - decay_rate) * weight_count
	return weight_sum, weight_count


def exp_run_avg(acc,decay_rate):
	ema = []
	sum_prev = 0
	count_prev = 0
	for i in range(0, len(acc)):
		sum_prev, count_prev = exp_moving_avg(acc[i], decay_rate, sum_prev, count_prev)
		ema.append(sum_prev/count_prev)
	return ema


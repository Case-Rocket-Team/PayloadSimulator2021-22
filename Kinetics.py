import numpy as np
import math

# heading(azimuth,bank angle, glide angle)
gravity = 9.81

def simulate_flight(
	mass, pos, vel, app_accel,
	timestep, air_density, wind_speed):
	"""calculates kinetics of the system, using Eulers method, for a given timestep with applied forces.

	Args:
		mass (float): the mass of the payload
		pos (Vec3): an array containing the absolute cartesian position of the craft.
		vel (Vec3): an array containing the caretesian velocity of the craft.
		app_accel (Vec3): the applied acceleration (sigmadot).
		timestep (float): the size of the timestep for eulers method.
		air_density (function): a function that returns the air density at a point.
		wind_speed (function): a function that returns the wind velocity at a point.

	Returns:
		pos: the position after eulers method is applied.
		vel: the vel after eulers method is applied.
		accel: the total acceleration after applying wind, gravity, lift and drag.
	"""
	
	
	return pos, vel, accel

# Calculates lift force on the vehicle
def calcLiftForce(lift_coefficient, air_density, velocity, vehicle_area):
	# L=\frac{1}{2}*C_{L}*\rho*V^{_{2}}*A
	return 0.5 * lift_coefficient * air_density * velocity ** 2 * vehicle_area

# Calculates drag force on the vehicle
def calcDragForce(drag_coefficient, air_density, velocity, vehicle_area):
	# L=\frac{1}{2}*C_{D}*\rho*V^{_{2}}*A
	return 0.5 * drag_coefficient * air_density * velocity ** 2 * vehicle_area

# Calculate rate of change of the glide angle
def calcRocGlideAngle(lift_force, heading, mass, velocity):
	# \dot{\gamma }=\frac{(L*cos\sigma -W*cos\gamma )}{mV}
	return ((lift_force * math.cos(heading[1]) - (mass * gravity * math.cos(heading[2])))) / (mass * velocity)

# Calculate rate of change of the azimuth angle
def calcRocAzimuth(lift_force, heading, mass, velocity):
	# \frac{L*sin\sigma }{mVcos\gamma }
	return lift_force * math.sin(heading[1]) / (mass * velocity * math.cos(heading[2]))
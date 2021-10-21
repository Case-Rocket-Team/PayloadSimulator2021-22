
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


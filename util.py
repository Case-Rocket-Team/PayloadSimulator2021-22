
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

def graph_data(sim_kinetics, noise_data, pred_kinetics, path, **graph_params):
	"""creates a series of graphs for important data
		#TODO: decide what graphs to show

	Args:
		sim_kinetics (Vec3[][]): the simulated kinetics (pos,vel,accel) for the course of the simulation.
		noise_data (Vec3[][]): the noisy data (gps,imu,pressure) for the course of the simulation.
		pred_kinetics (Vec3[][]): the simulated kinetics (pos,vel,accel) for the course of the simulation.
		path (Vec3): the set of waypoints used throughout the simulation.
	"""

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
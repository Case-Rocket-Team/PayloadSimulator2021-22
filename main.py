from Kinetics import simulate_flight, get_wind_speed, get_air_density
import numpy as np
from util import graph_data


pos = np.array([0, 0, 1000])
vel = np.array([0, 0, -20])
vel_mag = np.linalg.norm(vel)
heading = np.array([0, 0.174533, 0])
applied_acceleration = np.zeros(3)
_mass = 4.249
_timestep = .1
previous_wind_speeds = None

sim_kinematics = [[],[],[]]

while pos[2] > 0:
    pos, heading, vel, vel_mag, accel, previous_wind_speed = simulate_flight(_mass, pos, vel, vel_mag, heading, applied_acceleration, _timestep, get_air_density, get_wind_speed, previous_wind_speeds)
    
    sim_kinematics[0].append(pos)

graph_data(sim_kinematics, _timestep, 0, 0, 0)
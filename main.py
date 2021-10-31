from Kinetics import simulate_flight, get_wind_speed, get_air_density
from icecream import ic
import numpy as np


pos = np.array([0, 0, 1000])
vel = np.array([0, 0, -20])
vel_mag = np.linalg.norm(vel)
heading = np.zeros(3)
applied_acceleration = np.zeros(3)
_mass = 4.249
_timestep = .1


while pos[2] > 0:
    pos, heading, vel, vel_mag, accel = simulate_flight(_mass, pos, vel, vel_mag, heading, applied_acceleration, _timestep, get_air_density, get_wind_speed)
    print("\n\npos")
    print(pos)
    print("heading")
    print(heading)
    print("vel")
    print(vel)
    print("accel")
    print(accel)
    
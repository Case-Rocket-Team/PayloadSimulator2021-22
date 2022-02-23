from Kinetics import simulate_flight, get_wind_speed, get_air_density
import numpy as np
from ControlLoop import pure_pursuit, gen_path
from util import graph_data
import csv


def main():
    pos = np.array([0, 0, 1000])
    vel = np.array([0, 0, -20])
    vel_mag = np.linalg.norm(vel)
    target = [700, 700, 0]
    heading = np.array([0, 0.418879, 0])
    applied_acceleration = np.zeros(3)
    _mass = 4.249
    _timestep = .1
    look_ahead_distance = 20

    path = gen_path(pos, heading, target)
    curve, bank_angle = pure_pursuit(pos, look_ahead_distance, vel, path)

    sim_kinematics = [[],[],[],[]]

    with open('data.csv', mode='w') as data:
            data_write = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            data_write.writerow(['Bank Angle', 'Glide Angle', 'Turn Radius'])

    gen_path()

    while True:
        time = 0
        while pos[2] > 0:
            time += _timestep
            pos, heading, vel, vel_mag, accel, azimuth_angle_roc = simulate_flight(_mass, pos, vel, vel_mag, heading, applied_acceleration, _timestep, get_air_density, get_wind_speed)
            sim_kinematics[0].append(pos)
            sim_kinematics[1].append(heading)
            turn_radius = (vel[0]/azimuth_angle_roc)
            print("Heading", heading[2])
            print("Velocity", vel_mag)
            if time >= 100:
                with open('data.csv', mode='w') as data:
                    data_write = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    data_write.writerow([heading[1], heading[2], turn_radius])
                break

        heading[2] += 0.01

    graph_data(sim_kinematics, _timestep, 0, 0, 0)


if __name__ == "__main__":
    main()

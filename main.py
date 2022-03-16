from Kinetics import simulate_flight, get_wind_speed
import numpy as np
from ControlLoop import pure_pursuit, gen_path, plot_path
from util import graph_data
import csv


def main():
    pos = np.array([0, 0, 1000])
    vel = np.array([-10, 10, -2])
    vel_mag = np.linalg.norm(vel)
    target = [700, 700, 0]
    heading = [0, 0, 0]
    applied_acceleration = np.zeros(3)
    _mass = 4.249
    _time_step = 1
    look_ahead_distance = 50

    path = gen_path(pos, vel, target)
    plot_path([path])

    sim_kinematics = [[], [], [], []]

    with open('data.csv', mode='w') as data:
        data_write = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_write.writerow(['Bank Angle', 'Glide Angle', 'Turn Radius'])

    time = 0

    while pos[2] > 0:
        curve, bank_angle = pure_pursuit(pos, look_ahead_distance, np.ndarray.tolist(vel), path, heading[2])

        # if pure pursuit returned a blank curve
        if not curve:
            return True # TODO: Get rid of when program no longer loops

            path = gen_path(pos, vel, target)
            continue

        plot_path([path, curve])

        heading[1] = bank_angle

        time += _time_step
        pos, heading, vel, vel_mag, accel, azimuth_angle_roc = simulate_flight(_mass, pos, vel, vel_mag, heading,
                                                                               applied_acceleration, _time_step)
        sim_kinematics[0].append(pos)
        sim_kinematics[1].append(heading)
        turn_radius = abs((vel[0]/azimuth_angle_roc))

        print("Heading", heading[2])
        print("Velocity", vel_mag)

        if time >= 100:
            with open('data.csv', mode='w') as data:
                data_write = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_write.writerow([heading[1], heading[2], turn_radius])
            break

    graph_data(sim_kinematics, _time_step, 0, 0, 0)


if __name__ == "__main__":
    main()

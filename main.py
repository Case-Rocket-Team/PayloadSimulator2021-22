from Kinetics import simulate_flight, get_wind_speed
import numpy as np
from ControlLoop import pure_pursuit, gen_path, plot_path, generate_straight_path, normalize, dist_formula_2d
from util import graph_data
import csv
import math


def main():
    pos = np.array([0, 0, 5000])
    vel = np.array([-11, 11, -1.5])
    vel_mag = np.linalg.norm(vel)
    target = [700, 700, 0]
    heading = [2.00713, 0, 0.346]
    applied_acceleration = np.zeros(3)
    _mass = 4.249
    _time_step = 1/100
    look_ahead_distance = 500

    path = gen_path(pos, vel, target)
    path = path[:2000]

    velocity_trajectory = []
    generate_straight_path(pos, 1/2.7, vel*100, normalize(vel), velocity_trajectory)
    plot_path(path, velocity_trajectory)

    sim_kinematics = [[], [], [], []]

    with open('data.csv', mode='w') as data:
        data_write = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_write.writerow(['Bank Angle', 'Glide Angle', 'Turn Radius'])

    time = 0

    while pos[2] > 0:
        recalc_step = 1
        if round(time, 2) % recalc_step == 0:
            curve, bank_angle = pure_pursuit(pos, look_ahead_distance, np.ndarray.tolist(vel), path, heading[2])

        print(f"Bank Angle {bank_angle * 180 / math.pi}")

        # if pure pursuit returned a blank curve
        if not curve:
            print("!!!!!!!!!!borken!!!!!!!!!!!!!")
            return True # TODO: Get rid of when program no longer loops

            path = gen_path(pos, vel, target)
            continue


        velocity_trajectory = []
        generate_straight_path(pos, 1/2.7, vel*100, normalize(vel), velocity_trajectory)
        # plot_path(path, curve, velocity_trajectory)

        heading[1] = bank_angle

        time += _time_step

        print("pos: ", pos, " heading: ", heading, " vel: ", vel, " vel_mag: ", vel_mag)

        pos, heading, vel, vel_mag, accel, azimuth_angle_roc = simulate_flight(_mass, pos, vel, vel_mag, heading,
                                                                               applied_acceleration, _time_step)

        print("pos: ", pos, " heading: ", heading, " vel: ", vel, " vel_mag: ", vel_mag, " accel: ",
              accel, " azimuth angle roc: ", azimuth_angle_roc)

        velocity_trajectory = []
        generate_straight_path(pos, 1/2.7, vel*100, normalize(vel), velocity_trajectory)

        pure_pursuit_trajectory = []
        pure_pursuit_direction = [curve[-1][x] - pos[x] for x in range(3)]
        print(f"Curve: {curve[-1]}")
        print(f"Direction: {normalize(pure_pursuit_direction)}")
        generate_straight_path(pos, 1 / 2.7, pure_pursuit_direction,
                               normalize(pure_pursuit_direction), pure_pursuit_trajectory)

        print(f"Time :{time}")
        second_step = 2
        if round(time, 2) % second_step == 0:
            plot_path(path, curve, velocity_trajectory, pure_pursuit_trajectory)

        sim_kinematics[0].append(pos)
        sim_kinematics[1].append(heading)

        # calculated by:
        # https://app.knovel.com/web/view/khtml/show.v/rcid:kpPADSMDC1/cid:kt010RIPL3/viewerType:khtml//root_slug:precision-aerial-delivery/url_slug:gliding-parachute-performance?kpromoter=federation&page=46&view=collapsed&zoom=1
        turn_radius = abs((math.sqrt(vel[0]**2 + vel[1]**2)/azimuth_angle_roc))

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

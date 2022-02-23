import numpy as np
import math
import json


# TODO: pull these from JSON
_gravity = 9.81
_lift_coefficient = 0.449
_drag_coefficient = 0.162
_span = 1.016
_chord = 0.508
_area = _span * _chord
_wind_speed_x = 0.0
_wind_speed_y = 0.0
_air_density = 1.225
_ground_wind_speed_x = -5
_ground_wind_speed_y = 1


def simulate_flight( mass, pos, vel, vel_mag, heading, app_accel, timestep, air_density, wind_speed ):
    """calculates kinetics of the system, using Eulers method, for a given timestep with applied forces.

    Args:
        mass (float): the mass of the payload
        pos (Vec3): an array containing the absolute cartesian position of the craft.
        vel (Vec3): an array containing the caretesian velocity of the craft.
        vel_mag (float): the overall magitude of the velocity of the craft.
        heading (Vec3): an array containing the heading of the craft in the format [azimuth, bank_angle, glide_angle]
        app_accel (Vec3): the applied acceleration (sigmadot).
        timestep (float): the size of the timestep for eulers method.
        air_density (function): a function that returns the air density at a point.
        wind_speed_func (function): a function that returns the wind velocity at a point.
        previous_wind_speeds (Vec2): the wind velocity in x and y at the last position

    Returns:
        pos (Vec3): the position after eulers method is applied.
        heading (Vec3): the heading after eulers method is applied in the format [azimuth, bank_angle, glide_angle]
        vel (Vec3): the vel after eulers method is applied.
        vel_mag (float): the overall magitude of the velocity after eulers method is applied.
        accel (Vec3): the total acceleration after applying wind, gravity, lift and drag.
    """
    air_density = air_density(pos)  # TODO: get from jenny
    drag = calc_drag_force(_drag_coefficient, air_density, vel_mag, _area)
    lift = calc_lift_force(_lift_coefficient, air_density, vel_mag, _area)

    glide_angle_roc = calc_roc_glide_angle(lift, heading, mass, vel_mag)
    azimuth_angle_roc = calc_roc_azimuth(lift, heading, mass, vel_mag)

    previous_wind_speeds = wind_speed_func(
        pos, _ground_wind_speed_x, _ground_wind_speed_y, previous_wind_speeds
    )
    wind_speed_x, wind_speed_y = previous_wind_speeds
    heading = calc_heading(heading, glide_angle_roc, azimuth_angle_roc, timestep)
    new_vel, vel_mag = calc_velocity(
        vel_mag, heading[2], heading[0], drag, mass, timestep
    )
    pos = calc_position(pos, vel, wind_speed_x, wind_speed_y, timestep)

    accel = (new_vel - vel) / timestep

    return pos, heading, new_vel, vel_mag, accel, previous_wind_speeds


# Calculates lift force on the vehicle
def calc_lift_force(_lift_coefficient, air_density, velocity, vehicle_area):
    # L=\frac{1}{2}*C_{L}*\rho*V^{_{2}}*A
    return 0.5 * _lift_coefficient * air_density * velocity ** 2 * vehicle_area


# Calculates drag force on the vehicle
def calc_drag_force(_drag_coefficient, air_density, velocity, vehicle_area):
    # L=\frac{1}{2}*C_{D}*\rho*V^{_{2}}*A
    return 0.5 * _drag_coefficient * air_density * velocity ** 2 * vehicle_area


# Calculate rate of change of the glide angle
def calc_roc_glide_angle(lift_force, heading, mass, velocity):
    # \dot{\gamma }=\frac{(L*cos\sigma -W*cos\gamma )}{mV}
    return (
        (lift_force * math.cos(heading[1]) - (mass * _gravity * math.cos(heading[2])))
    ) / (mass * velocity)


# Calculate rate of change of the azimuth angle
def calc_roc_azimuth(lift_force, heading, mass, velocity):
    # \dot{\psi} = \frac{L*sin\sigma }{mVcos\gamma}
    return lift_force * math.sin(heading[1]) / (mass * velocity * math.cos(heading[2]))


# placeholder that just gives the same air density over and over
def get_air_density(pos):
    return _air_density


def calc_velocity(current_velocity, glide_angle, azimuth_angle, drag_force, mass, dt):
    """
    Calculates the velocity in the next timestep

    Args:
        current_velocity (float): the magnitude of current velocity
        glide_angle (float): in radians, the angle of attack of the craft
        azimuth_angle (float): in radians, the angle from north of the craft motion, CW+
        drag_force (float): the drag force on the craft
        mass (float): the mass of the craft in kg
        dt (float): the timestep

    Returns:
        vel (Vec3): the calculated next cartesian velocities of the craft
        velocity_mag (float): the magnitude of the next velocities
    """

    # Calculate velocity differential
    # - \frac{D + Wsin \gamma}{m} \cdot dt
    dv = (-(drag_force + mass * _gravity * math.sin(glide_angle)) / mass) * dt

    # Calculate new velocity
    velocity_mag = current_velocity + dv

    # Split velocity into cartesian components and store in array
    v_x = velocity_mag * math.cos(glide_angle) * math.cos(azimuth_angle)
    v_y = velocity_mag * math.cos(glide_angle) * math.sin(azimuth_angle)
    v_z = velocity_mag * math.sin(glide_angle)
    vel = np.array([v_x, v_y, v_z])

    return vel, velocity_mag


def calc_position(current_pos, current_velocity, wind_vel_x, wind_vel_y, dt):
    """
    Calculates the position in the next timestep based on current velocity and wind.

    Args:
        current_pos (Vec3): an array containing the current known absolute cartesian position of the craft [x, y, z]
        current_velocity (Vec3): an array containing the current known absolute cartesian velocities of the craft [vx, vy, vz]
        wind_vel_x (float): the wind speed in the x direction
        wind_vel_y (float): the wind speed in the y direction
        dt (float): timestep

    Output:
        pos (Vec3): an array containing the next absolute cartesian position of the craft [x, y, z]
    """

    # Calculate dx, dy, dz
    # dx = (V_x + w_x) \cdot dt
    dx = (current_velocity[0] + wind_vel_x) * dt
    # dy = (V_y + w_y) \cdot dt
    dy = (current_velocity[1] + wind_vel_y) * dt
    # dz = V_z \cdot dt
    dz = current_velocity[2] * dt
    pos_diff = np.array([dx, dy, dz])

    pos = current_pos + pos_diff

    return pos


def calc_heading(current_heading, glide_angle_roc, azimuth_roc, dt):
    """
    Calculate the new heading of the vehicle based on timestep, glide angle rate of change, and azimuth angle rate of change

    Args:
        current_heading (Vec3): array containing the current heading of the craft in the format [azimuth, bank_angle, glide_angle]
        glide_angle_roc (float): glide angle rate of change
        azimuth_roc (float): azimuth angle rate of change
        dt (float): time step

    Output:
        new_heading (Vec3): array containing the new heading of the craft in the format [azimuth, bank_angle, glide_angle]
    """

    # find the actual amount the angle changes by multiplying rate by time
    glide_angle_change = glide_angle_roc * dt
    azimuth_angle_change = azimuth_roc * dt
    # we don't update bank_angle, just the other two
    heading_change = np.array([azimuth_angle_change, 0.0, glide_angle_change])

    return current_heading + heading_change

def get_wind_speed(current_pos, ground_wind_speed_x, ground_wind_speed_y, previous_wind_speeds=None, alpha=0.143):
    """
    Calculates the wind speed at a given height using the formula found here: https://en.wikipedia.org/wiki/Wind_profile_power_law
    (same as here: https://websites.pmc.ucsc.edu/~jnoble/wind/extrap/)

    Args:
        current_pos (Vec3): array with current position of the craft
        ground_wind_speed (float): wind speed 5 meters off the ground
        alpha (float): the wind shear exponent
    
    Output:
        wind_given_height (float): wind speed for this position
    """

    # u = u_r (\frac{z}{z_r})^{\alpha}
    calculated_speed_x = ground_wind_speed_x * ((current_pos[2] / 5) ** alpha)
    calculated_speed_y = ground_wind_speed_y * ((current_pos[2] / 5) ** alpha)
    # on the first time we still need a previous wind speed that isn't exactly on the line
    if previous_wind_speeds == None:
        previous_wind_speed_x = calculated_speed_x * 1.001
        previous_wind_speed_y = calculated_speed_y * 1.001
    else:
        previous_wind_speed_x = previous_wind_speeds[0]
        previous_wind_speed_y = previous_wind_speeds[1]

    # sample from a normal distribution where...
    sigma_x = abs(calculated_speed_x - previous_wind_speed_x) # ... the stddev is the difference between the last two values and...
    mean_x = calculated_speed_x + previous_wind_speed_x / 2.0 # ...the mean is the average of the last two values
    gusted_wind_speed_x = np.random.normal(mean_x, sigma_x, 1)[0]

    sigma_y = abs(calculated_speed_y - previous_wind_speed_y) # ... the stddev is the difference between the last two values and...
    mean_y = (calculated_speed_y + previous_wind_speed_y) / 2.0 # ...the mean is the average of the last two values
    gusted_wind_speed_y = np.random.normal(mean_y, sigma_y, 1)[0]

    return (gusted_wind_speed_x, gusted_wind_speed_y)

def get_zero_wind(current_pos, ground_wind_speed, previous_wind_speeds=None):
    return(0, 0)
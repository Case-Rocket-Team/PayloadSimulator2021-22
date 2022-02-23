import math  
vel_mag = 16.5
_gravity = 9.81
bank_angle = 24 * math.pi / 180
glide_angle = -0.3511812642006587
span = 1.016
turn_rate = (math.sin(bank_angle) * _gravity) / (vel_mag * math.cos(glide_angle))
deflect_angle = (turn_rate * span) / (0.625 * vel_mag)

print(deflect_angle * 180 / math.pi)
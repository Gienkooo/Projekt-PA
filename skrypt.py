import math
import matplotlib.pyplot as plt

def e(v, sv):
    return v - sv

def perpendicular_component(v, alfa):
    return math.cos(alfa) * v

def parallel_component(v, alfa):
    return math.sin(alfa) * v

def slope(func, x, dx):
    return (func(x) - func(x - dx)) / dx

def acceleration(F, m):
    return F / m

def pressure(m):
    return m * g

def generated_force(torque):
    return torque / wheel_radius

def generated_torque(F):
    return F * wheel_radius

def generated_force_capped(torque):
    return max(-max_torque, min(max_torque, torque)) / wheel_radius

def perpendicular_pressure(alfa):
    return perpendicular_component(pressure(vehicle_mass), alfa)

def rolldown(alfa):
    return parallel_component(pressure(vehicle_mass), alfa)

def rolling_resistance(F):
    return F * rolling_res_coef

def air_drag(v):
    return 0.5 * wind_res_coef * frontal_area * air_density * v * v

def wind_force(v, wind_v):
    return air_drag(v + wind_v)


air_density = 1.293
frontal_area = 1.89
wind_res_coef = 0.36
rolling_res_coef = 0.012
g = 9.80665
wind_speed = 0
wind_angle = 0
vehicle_mass = 2000
wheel_radius = 0.2159
max_torque = 200
kp = 0.1
ki = 0.01
kd = 0.05
max_force = generated_force(max_torque)

def route_manip(x):
    return math.atan(x / 400) * 10

route_func = route_manip
t = 250
ts = 1
set_vel = 10
start_vel = 0
start_pos = -1000

time_array = [0]
vel_arr = [start_vel]
pos_arr = [start_pos]
h_arr = [route_func(pos_arr[-1])]
e_arr = [0]
es_arr = [0]
a_arr = [0]
F_arr = [0]
slope_arr = [slope(route_func, pos_arr[-1], 1)]
rolldown_arr = [rolldown(slope_arr[-1])]

for _ in range(int(t / ts)):
    e_arr.append(set_vel - vel_arr[-1])
    es_arr.append(es_arr[-1] + e_arr[-1])
    curr_slope = slope(route_func, pos_arr[-1], 0.1 + vel_arr[-1] * ts)
    slope_arr.append(curr_slope)
    F_motor = max(-max_force, min(max_force, (kp * e_arr[-1] + ki * es_arr[-1] + kd * (e_arr[-2] - e_arr[-1]) / ts) * vehicle_mass / ts))
    F_rolling = rolling_resistance(curr_slope)
    F_drag = air_drag(vel_arr[-1])
    F_wind = wind_force(vel_arr[-1], perpendicular_component(wind_speed, math.radians(wind_angle)))
    F_rolldown = rolldown(curr_slope)
    F_net = F_motor - F_rolling - F_drag - F_wind - F_rolldown
    a = acceleration(F_net, vehicle_mass)
    curr_vel = vel_arr[-1] + a * ts
    curr_pos = pos_arr[-1] + curr_vel * ts
    time_array.append(ts + time_array[-1])
    vel_arr.append(curr_vel)
    pos_arr.append(curr_pos)
    h_arr.append(route_func(curr_pos))
    a_arr.append(a)
    F_arr.append(F_net)
    rolldown_arr.append(F_rolldown)
    

plt.subplot(3, 2, 1)
plt.plot(time_array, F_arr, label="F_arr")
plt.legend()
plt.subplot(3, 2, 2)
plt.plot(time_array, vel_arr, label="vel_arr")
plt.legend()
plt.subplot(3, 2, 3)
plt.plot(time_array, slope_arr, label="slope_arr")
plt.legend()
plt.subplot(3, 2, 4)
plt.plot(time_array, h_arr, label="h_arr")
plt.legend()
plt.subplot(3, 2, 5)
plt.plot(time_array, [route_func(x) for x in pos_arr], label="route")
plt.legend()
plt.subplot(3, 2, 6)
plt.plot(time_array, e_arr, label="e")
plt.legend()

plt.show()

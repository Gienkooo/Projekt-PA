import math

air_density = 1.293
frontal_area = 1.89
wind_res_coef = 0.36
rolling_res_coef = 0.012
g = 9.80665
wind_speed = 1.75
wind_angle = 0
vehicle_mass = 2000
wheel_radius = 0.2159
max_torque = 300

def route_manip(x):
    return math.atan(x / 100) * 50

route_func = route_manip
t = 5
ts = 0.01
set_vel = 15
vel_arr = [0]

time_array = [0]

def e(v):
    return v - set_vel

def perpendicular_component(v, alfa):
    return math.cos(alfa) * v

def parallel_component(v, alfa):
    return math.sin(alfa) * v

def slope(func, x, dx):
    return math.atan((func(x) - func(x - dx)) / dx)

def acceleration(F, m):
    return F / m

def pressure(m):
    return m * g

def generated_force(torque):
    return torque / wheel_radius

def perpendicular_pressure(m, alfa):
    return perpendicular_component(pressure(m), alfa)

def rolldown(m, alfa):
    return parallel_component(pressure(m), alfa)

def rolling_resistance(F):
    return F * rolling_res_coef

def air_drag(v):
    return 0.5 * wind_res_coef * frontal_area * air_density * v * v

def wind_force(v, wind_v):
    return air_drag(v - wind_v)




for _ in range(t // ts):
    

import math
import plotly.graph_objects as go
import dash 
from dash import html
import dash_core_components as dcc
import dash_bootstrap_components as dbc

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


def function1(x, p1):
    return p1 * x

def function2(x, p1, p2):
    return p1 * x + p2

def function3(x, p1, p2, p3):
    return (p1 * x + p2) % p3
p1 = 3
p2 = 20
p3 = 40

#parametry sliderow
min1 = 1
max1 = 10
value1 = (min1 + max1) / 2
min2 = 1
max2 = 10
value2 = (min1 + max1) / 2
min3 = 1
max3 = 10
value3 = (min1 + max1) / 2

x = [i for i in range(20)]

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container(
    [
        html.H1("TEMPOMAT"),
        dbc.Row(
            [
                dbc.Col(
                    html.Div(
                        [
                            html.H2("parametry pojazdu"),
                            html.H3("wybór modelu:"),
                            dcc.Dropdown(
                                id='dropdown_menu',
                                options=
                                [
                                    {'label':'autko1', 'value' : 'opt1'},
                                    {'label':'autko2', 'value' : 'opt2'},
                                    {'label':'custom', 'value' : 'opt3'}
                                ],
                                value='opt3'
                            ),
                            html.H3("parametr1:"),
                            dcc.Slider(
                                id='slider1',
                                min=min1,
                                max=max1,
                                step=1,
                                value=value1,
                                marks={i: str(i) for i in range(min1, max1+1)}
                            ),
                            html.H3("parametr2:"),
                            dcc.Slider(
                                id='slider2',
                                min=min2,
                                max=max2,
                                step=1,
                                value=value2,
                                marks={i: str(i) for i in range(min2, max2+1)}
                            ),
                            html.H3("parametr3:"),
                            dcc.Slider(
                                id='slider3',
                                min=min3,
                                max=max3,
                                step=1,
                                value=value3,
                                marks={i: str(i) for i in range(min3, max3+1)}
                            ),
                            html.H2("parametry symulacji"),
                            html.Button("aktualizuj", id='button')
                        ]
                    ),
                    width=3
                ),
                dbc.Col(
                    html.Div(
                        [
                            dcc.Graph(id='wykres1'),
                            dcc.Graph(id='wykres2'),
                            dcc.Graph(id='wykres3')
                        ]
                    ),
                    width=9
                )
            ]
        )
    ],
    fluid=True, 
    style={'height': '100vh'}
)

@app.callback(
    [
        dash.dependencies.Output('wykres1', 'figure'),
        dash.dependencies.Output('wykres2', 'figure'),
        dash.dependencies.Output('wykres3', 'figure'),
        dash.dependencies.Output('slider1', 'value'),
        dash.dependencies.Output('slider2', 'value'),
        dash.dependencies.Output('slider3', 'value'),
    ],
    [
        dash.dependencies.Input('slider1', 'value'),
        dash.dependencies.Input('slider2', 'value'),
        dash.dependencies.Input('slider3', 'value'),
        dash.dependencies.Input('dropdown_menu', 'value')
    ]
)

def update_graphs(p1, p2, p3, selected_model):
    if selected_model == 'opt1':
        p1, p2, p3 = 1, 2, 3
    elif selected_model == 'opt2':
        p1, p2, p3 = 2, 4, 6
    
    y1 = [function1(x,p1) for x in x]
    y2 = [function2(x,p1,p2) for x in x]
    y3 = [function3(x,p1,p2,p3) for x in x]

    trace1 = go.Scatter(x=x, y=y1, mode='lines', name='wykres 1')
    trace2 = go.Scatter(x=x, y=y2, mode='lines', name='wykres 2')
    trace3 = go.Scatter(x=x, y=y3, mode='lines', name='wykres 3')
    
    layout1 = go.Layout(
        title='wykres 1', 
        xaxis=dict(title='x'), 
        yaxis=dict(title='y')
    )
    layout2 = go.Layout(
        title='wykres 2', 
        xaxis=dict(title='x'), 
        yaxis=dict(title='y')
    )
    layout3 = go.Layout(
        title='wykres 3', 
        xaxis=dict(title='x'), 
        yaxis=dict(title='y')
    )

    fig1 = go.Figure(data=[trace1], layout=layout1)
    fig2 = go.Figure(data=[trace2], layout=layout2)
    fig3 = go.Figure(data=[trace3], layout=layout3)

    return fig1, fig2, fig3, p1, p2, p3


if __name__ == '__main__':
    app.run_server(port = 4050)
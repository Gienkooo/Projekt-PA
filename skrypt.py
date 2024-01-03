import math
import plotly.graph_objects as go
import dash 
from dash import html, dcc, Input, Output
import dash_bootstrap_components as dbc

def e(v, sv):
    return v - sv

def perpendicular_component(v, alfa):
    return math.cos(math.radians(alfa)) * v

def parallel_component(v, alfa):
    return math.sin(math.radians(alfa)) * v

def slope(func, x, dx):
    return math.atan((func(x) - func(x - dx)) / dx)

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
    return air_drag(v - perpendicular_component(wind_v, wind_angle))

def route_arc_tan(x):
    return (math.atan(x / 400) + math.pi / 2) * 10

def route_sin(x):
    return (math.sin(x / 400) + 1) * 5

def route_sin_harmonic(x):
    return (math.sin(x / 60) + math.sin(x / 120) + math.sin(x / 420) + 3) * 5

def route_const(x):
    return 0

#passat vlkswagen
model_1_frontal_area = 2.1
model_1_wind_res_coef = 0.29
model_1_rolling_res_coef = 0.05
model_1_vehicle_mass = 1450
model_1_wheel_radius = 0.35
model_1_max_torque = 420
model_1_kp = 2
model_1_ti = 10
model_1_td = 0.06

#ford super duty
model_2_frontal_area = 2.8
model_2_wind_res_coef = 0.33
model_2_rolling_res_coef = 0.007
model_2_vehicle_mass = 2600
model_2_wheel_radius = 0.38
model_2_max_torque = 1600.0
model_2_kp = 2
model_2_ti = 14.4
model_2_td = 0.05


#volkswagen up
model_3_frontal_area = 1.7
model_3_wind_res_coef = 0.32
model_3_rolling_res_coef = 0.004
model_3_vehicle_mass = 1080
model_3_wheel_radius = 0.30
model_3_max_torque = 230.0

#vehicle parameters
frontal_area = 1.89
wind_res_coef = 0.36
rolling_res_coef = 0.012
vehicle_mass = 2000
wheel_radius = 0.2159
max_torque = 200

min_frontal_area = 1.0
min_wind_res_coef = 0.0
min_rolling_res_coef = 0.0
min_vehicle_mass = 1000.0
min_wheel_radius = 0.1
min_max_torque = 0.0

max_frontal_area = 3.0
max_wind_res_coef = 1.0
max_rolling_res_coef = 1.0
max_vehicle_mass = 5000.0
max_wheel_radius = 1.0
max_max_torque = 2500.0

#enviorment parameters
default_air_density = 1.293
default_g = 9.80665
default_wind_speed = 0.0
default_wind_angle = 0.0

route_func = route_arc_tan
air_density = 1.293
g = 9.80665
wind_speed = 0
wind_angle = 0

min_air_density = 0.0
min_g = 9.0
min_wind_speed = 0.0
min_wind_angle = 0.0

max_air_density = 5.0
max_g = 10.0
max_wind_speed = 10.0
max_wind_angle = 360.0

#simulation parameters
default_t = 250
default_ts = 1
default_set_vel = 10
default_start_vel = 0
default_start_pos = -1000
default_kp = 2
default_ti = 70
default_td = 0.5

t = 250 #s
ts = 1 #s
set_vel = 10 #
start_vel = 0
start_pos = -1000
kp = 2
ti = 0.01
td = 0.05

min_t = 100
min_ts = 0.1
min_set_vel = 0
min_start_vel = 0
min_start_pos = -1000
min_kp = 0.1
min_ti = 0
min_td = 0

max_t = 500
max_ts = 10
max_set_vel = 50
max_start_vel = 50
max_start_pos = 0
max_kp = 10
max_ti = 250
max_td = 10
max_force = generated_force(max_torque)

time_array = [0]
vel_arr = [start_vel]
pos_arr = [start_pos]
h_arr = [route_func(pos_arr[-1])]
e_arr = [0]
es_arr = [0]
a_arr = [0]
a_obj_arr = [0]
F_arr = [0]
F_motor_arr = [0]
slope_arr = [slope(route_func, pos_arr[-1], 1)]
rolldown_arr = [rolldown(slope_arr[-1])]

def set_global_variables(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route):
    global frontal_area
    global wind_res_coef
    global rolling_res_coef
    global vehicle_mass 
    global wheel_radius 
    global max_torque
    global max_force 
    global air_density 
    global g
    global wind_speed 
    global wind_angle
    global t
    global ts
    global set_vel
    global route_func
    global start_vel
    global start_pos
    global kp, ti, td
    frontal_area = v1
    wind_res_coef = v2
    rolling_res_coef = v3
    vehicle_mass = v4
    wheel_radius = v5
    max_torque = v6
    max_force = generated_force(max_torque)
    route_func = route
    air_density = e1
    g = e2
    wind_speed = e3
    wind_angle = e4
    t = s1
    ts = s2
    start_vel = s3
    set_vel = s4
    start_pos = s5
    kp = s6
    ti = s7
    td = s8

def compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route):
    set_global_variables(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)
    global time_array, vel_arr, pos_arr, h_arr, e_arr, es_arr, a_arr, a_obj_arr, F_arr, F_motor_arr, slope_arr, rolldown_arr, route_func, max_set_vel, max_force, vehicle_mass, rolling_res_coef, wind_res_coef, frontal_area, air_density
    time_array = [0]
    vel_arr = [start_vel]
    pos_arr = [start_pos]
    h_arr = [route_func(pos_arr[-1])]
    e_arr = [set_vel - start_vel]
    es_arr = [0]
    a_arr = [0]
    a_obj_arr = [0]
    F_arr = [0]
    F_motor_arr = [0]
    slope_arr = [slope(route_func, pos_arr[-1], 1)]
    rolldown_arr = [rolldown(slope_arr[-1])]
    v_max = math.sqrt(max((2 * max_force - 2 * vehicle_mass * g * rolling_res_coef) / (wind_res_coef * frontal_area * air_density), 0.0000001))
    for _ in range(int(t / ts)):
        e_arr.append(set_vel - vel_arr[-1])
        es_arr.append(es_arr[-1] + e_arr[-1])
        curr_slope = slope(route_func, pos_arr[-1], 0.1 + vel_arr[-1] * ts)
        slope_arr.append(curr_slope)
        F_motor = max(-max_force, min(max_force, max_force * kp * (e_arr[-1] + (ts / ti) * es_arr[-1] + td * (e_arr[-1] - e_arr[-2]) / ts) / (v_max)))
        F_motor_arr.append(F_motor)
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
        a_obj_arr.append(vel_arr[-1] - vel_arr[-2])
        pos_arr.append(curr_pos)
        h_arr.append(route_func(curr_pos))
        a_arr.append(a)
        F_arr.append(F_net)
        rolldown_arr.append(F_rolldown)

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

traces = [[], [], [], []]
last_clr_n_clicks = 0
last_add_n_clicks = 0
last_s_param_clicks = 0
last_e_param_clicks = 0
start = True

layout1 = go.Layout(title='kształt przebytej trasy h(t)',yaxis=dict(title='h [m]', tickformat='.2f'),xaxis=dict(title='t [s]', tickformat='.2f'))
layout2 = go.Layout(title='prędkość pojazdu v(t)',yaxis=dict(title='v [m/s]', tickformat='.2f'),xaxis=dict(title='t [s]', tickformat='.2f'))
layout3 = go.Layout(title='siła ciągu silnika F(t)',yaxis=dict(title=u'F [kg · m/s²]', tickformat='.2f'),xaxis=dict(title='t [s]', tickformat='.2f'))
layout4 = go.Layout(title='uchyb regulacji e(t)',yaxis=dict(title='e [m/s]', tickformat='.2f'),xaxis=dict(title='t [s]', tickformat='.2f'))

def add_new_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route):
    compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)
    x = [i for i in range(0, max_t + 1, s2)]
    new_trace1 = go.Scatter(
        x=x,
        y=h_arr,
        mode='lines',
    )
    new_trace2 = go.Scatter(
        x=x,
        y=vel_arr,
        mode='lines',
    )
    new_trace3 = go.Scatter(
        x=x,
        y=F_motor_arr,
        mode='lines',
    )
    new_trace4 = go.Scatter(
        x=x,
        y=e_arr,
        mode='lines',
    )
    traces[0].append(new_trace1)
    traces[1].append(new_trace2)
    traces[2].append(new_trace3)
    traces[3].append(new_trace4)

def modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route):
    compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)
    x = [i for i in range(0, max_t + 1, s2)]
    if traces[0] and traces[1] and traces[2] and traces[3]:
        traces[0][-1]['y'] = h_arr
        traces[1][-1]['y'] = vel_arr
        traces[2][-1]['y'] = F_motor_arr
        traces[3][-1]['y'] = e_arr
        traces[0][-1]['x'] = x
        traces[1][-1]['x'] = x
        traces[2][-1]['x'] = x
        traces[3][-1]['x'] = x

app.layout = dbc.Container(
    fluid=True,
    children=[
        html.H1(" TEMPOMAT"),
        dbc.Row([
            dbc.Col(
                [
                    html.Div([
                    html.H5("parametry pojazdu"),
                    html.Label('wybór modelu:'),
                    dcc.Dropdown(
                        id='dropdown_menu',
                        options=
                        [
                            {'label':'niestandardowy', 'value' : 'opt1'},
                            {'label':'Volkswagen Passat', 'value' : 'opt2'},
                            {'label':'Ford Super Duty', 'value' : 'opt3'},
                            {'label':'Volkswagen Up', 'value' : 'opt4'}
                        ],
                        value='opt1'
                    ),
                    html.Label(u'powierzchnia czołowa [m²]'),
                    dcc.Slider(
                        id='v_slider1',
                        min=min_frontal_area,
                        max=max_frontal_area,
                        step=0.01,
                        value=frontal_area,
                        marks={min_frontal_area : str(min_frontal_area), max_frontal_area : str(max_frontal_area)},
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('współczynnik oporu toczenia'),
                    dcc.Slider(
                        id='v_slider2',
                        min=min_rolling_res_coef,
                        max=max_rolling_res_coef,
                        step=0.01,
                        value=rolling_res_coef,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('współczynnik oporu aerodynamicznego'),
                    dcc.Slider(
                        id='v_slider3',
                        min=min_wind_res_coef,
                        max=max_wind_res_coef,
                        step=0.01,
                        value=wind_res_coef,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('masa pojazdu [kg]'),
                    dcc.Slider(
                        id='v_slider4',
                        min=min_vehicle_mass,
                        max=max_vehicle_mass,
                        step=0.01,
                        value=vehicle_mass,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('promień koła [m]'),
                    dcc.Slider(
                        id='v_slider5',
                        min=min_wheel_radius,
                        max=max_wheel_radius,
                        step=0.01,
                        value=wheel_radius,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('maksymalny moment siły silnika [Nm]'),
                    dcc.Slider(
                        id='v_slider6',
                        min=min_max_torque,
                        max=max_max_torque,
                        step=0.01,
                        value=max_torque,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label(' '),
                    html.H5(u'parametry środowiska'),
                    html.Label('kształt trasy:'),
                    dcc.Dropdown(
                        id='route_menu',
                        options=
                        [
                            {'label':'arcus tangens', 'value' : 'opt1'},
                            {'label':'sinus', 'value' : 'opt2'},
                            {'label':'sinus harmoniczny', 'value' : 'opt3'},
                            {'label':'stała', 'value' : 'opt4'}
                        ],
                        value='opt1'
                    ),
                    html.Label(u'gęstość powietrza [kg/m³]'),
                    dcc.Slider(
                        id='e_slider1',
                        min=min_air_density,
                        max=max_air_density,
                        step=0.01,
                        value=air_density,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label(u'przyspieszenie grawitacyjne [m/s²]'),
                    dcc.Slider(
                        id='e_slider2',
                        min=min_g,
                        max=max_g,
                        step=0.01,
                        value=g,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('prędkość wiatru [m/s]'),
                    dcc.Slider( 
                        id='e_slider3',
                        min=min_wind_speed,
                        max=max_wind_speed,
                        step=0.01,
                        value=wind_speed,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('kąt wiania wiatru [°]'),
                    dcc.Slider(
                        id='e_slider4',
                        min=min_wind_angle,
                        max=max_wind_angle,
                        step=0.01,
                        value=wind_angle,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    )
                    ], style={'width': '100%', 'margin': 10}),
                ],
                width=2
            ),
            dbc.Col(
                [
                    html.Div([
                    html.H5("parametry symulacji"),
                    html.Label('prędkość początkowa [m/s]'),
                    dcc.Slider(
                        id='s_slider3',
                        min=min_start_vel,
                        max=max_start_vel,
                        step=0.01,
                        value=start_vel,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('prędkość zadana [m/s]'),
                    dcc.Slider(
                        id='s_slider4',
                        min=min_set_vel,
                        max=max_set_vel,
                        step=0.01,
                        value=set_vel,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('położenie początkowe [m]'),
                    dcc.Slider(
                        id='s_slider5',
                        min=min_start_pos,
                        max=max_start_pos,
                        step=0.01,
                        value=start_pos,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('czas symulacji [s]'),
                    dcc.Slider(
                        id='s_slider1',
                        min=min_t,
                        max=max_t,
                        step=1,
                        value=t,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('krok symulacji [s]'),
                    dcc.Slider(
                        id='s_slider2',
                        min=min_ts,
                        max=max_ts,
                        step=None,
                        value=ts,
                        marks={ 0.1: '0.1', 1: '1', 10: '10'}
                    ),
                    html.Label('k_p'),
                    dcc.Slider(
                        id='s_slider6',
                        min=min_kp,
                        max=max_kp,
                        step=0.1,
                        value=kp,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('T_i'),
                    dcc.Slider(
                        id='s_slider7',
                        min=min_ti,
                        max=max_ti,
                        step=0.1,
                        value=ti,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label(u'T_d'),
                    dcc.Slider(
                        id='s_slider8',
                        min=min_td,
                        max=max_td,
                        step=0.001,
                        value=td,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.H1(" "),
                    dbc.Row(
                    html.Button('resetuj parametry środowiska', id='e_param-button', n_clicks=0)
                    ),
                    dbc.Row(
                        html.Button('resetuj parametry symulacji', id='s_param-button', n_clicks=0)
                    ),
                    dbc.Row([
                        html.Button('dodaj ślad', id='add-button', n_clicks=0),
                        html.Button('usuń poprzednie ślady', id='clr-button', n_clicks=0)
                    ]),
                    ], style={'width': '100%', 'margin': 10}),
                ],
                width=2
            ),
            dbc.Col(
                html.Div([
                dcc.Graph(id='graph-1', figure={'data': [], 'layout': layout1}),
                dcc.Graph(id='graph-4', figure={'data': [], 'layout': layout4})
                ], style={'width': '100%', 'margin': 10}),
                width=4
            ),
            dbc.Col(
                html.Div([
                dcc.Graph(id='graph-2', figure={'data': [], 'layout': layout2}),
                dcc.Graph(id='graph-3', figure={'data': [], 'layout': layout3})
                ], style={'width': '100%', 'margin': 10}),
                width=4
            )
        ], style={"margin-left": "0px"})
    ]
)

@app.callback(
    [
        Output('graph-1', 'figure'),
        Output('graph-2', 'figure'),
        Output('graph-3', 'figure'),
        Output('graph-4', 'figure'),
        Output('v_slider1', 'value'),
        Output('v_slider2', 'value'),
        Output('v_slider3', 'value'),
        Output('v_slider4', 'value'),
        Output('v_slider5', 'value'),
        Output('v_slider6', 'value'),
        Output('e_slider1', 'value'),
        Output('e_slider2', 'value'),
        Output('e_slider3', 'value'),
        Output('e_slider4', 'value'),
        Output('s_slider1', 'value'),
        Output('s_slider2', 'value'),
        Output('s_slider3', 'value'),
        Output('s_slider4', 'value'),
        Output('s_slider5', 'value'),
        Output('s_slider6', 'value'),
        Output('s_slider7', 'value'),
        Output('s_slider8', 'value'),
        Output('v_slider1', 'disabled'),
        Output('v_slider2', 'disabled'),
        Output('v_slider3', 'disabled'),
        Output('v_slider4', 'disabled'),
        Output('v_slider5', 'disabled'),
        Output('v_slider6', 'disabled'),
     ],
    [
        Input('dropdown_menu', 'value'),
        Input('route_menu', 'value'),
        Input('add-button', 'n_clicks'),
        Input('clr-button', 'n_clicks'),
        Input('e_param-button', 'n_clicks'),
        Input('s_param-button', 'n_clicks'),
        Input('v_slider1', 'value'),
        Input('v_slider2', 'value'),
        Input('v_slider3', 'value'),
        Input('v_slider4', 'value'),
        Input('v_slider5', 'value'),
        Input('v_slider6', 'value'),
        Input('e_slider1', 'value'),
        Input('e_slider2', 'value'),
        Input('e_slider3', 'value'),
        Input('e_slider4', 'value'),
        Input('s_slider1', 'value'),
        Input('s_slider2', 'value'),
        Input('s_slider3', 'value'),
        Input('s_slider4', 'value'),
        Input('s_slider5', 'value'),
        Input('s_slider6', 'value'),
        Input('s_slider7', 'value'),
        Input('s_slider8', 'value')
     ]
)
def update_graph(dropdown, route_dropdown, add_n_clicks, clr_n_clicks, e_param_clicks, s_param_clicks,
                 v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8):
    global last_clr_n_clicks, last_add_n_clicks, last_s_param_clicks, last_e_param_clicks, traces, start

    if route_dropdown == 'opt1':
        route = route_arc_tan
    elif route_dropdown == 'opt2':
        route = route_sin
    elif route_dropdown == 'opt3':
        route = route_sin_harmonic
    elif route_dropdown == 'opt4':
        route = route_const

    if add_n_clicks != last_add_n_clicks:
        add_new_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)
        last_add_n_clicks = add_n_clicks
    else:
        modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)

    if clr_n_clicks != last_clr_n_clicks:
        traces = [[], [], [], []]
        last_clr_n_clicks = clr_n_clicks

    if e_param_clicks != last_e_param_clicks: 
        e1 = default_air_density
        e2 = default_g
        e3 = default_wind_speed
        e4 = default_wind_angle
        last_e_param_clicks = e_param_clicks

    if s_param_clicks != last_s_param_clicks:
        s1 = default_t
        s2 = default_ts
        s3 = default_set_vel
        s4 = default_start_vel
        s5 = default_start_pos
        s6 = default_kp
        s7 = default_ti
        s8 = default_td
        last_s_param_clicks = s_param_clicks

    if dropdown == 'opt1':
        disabled = False
    else:
        disabled = True
        if dropdown == 'opt2':
            v1 = model_1_frontal_area
            v2 = model_1_wind_res_coef
            v3 = model_1_rolling_res_coef
            v4 = model_1_vehicle_mass
            v5 = model_1_wheel_radius
            v6 = model_1_max_torque
        elif dropdown == 'opt3':
            v1 = model_2_frontal_area
            v2 = model_2_wind_res_coef
            v3 = model_2_rolling_res_coef
            v4 = model_2_vehicle_mass
            v5 = model_2_wheel_radius
            v6 = model_2_max_torque
        elif dropdown == 'opt4':
            v1 = model_3_frontal_area
            v2 = model_3_wind_res_coef
            v3 = model_3_rolling_res_coef
            v4 = model_3_vehicle_mass
            v5 = model_3_wheel_radius
            v6 = model_3_max_torque
        modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, route)
    
    figures = [
        {'data': traces[0], 'layout': layout1},
        {'data': traces[1], 'layout': layout2},
        {'data': traces[2], 'layout': layout3},
        {'data': traces[3], 'layout': layout4}
    ]
    
    return figures[0], figures[1], figures[2], figures[3], v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5, s6, s7, s8, disabled, disabled, disabled, disabled, disabled, disabled

if __name__ == '__main__':
    app.run_server(debug=False)

import math
import plotly.graph_objects as go
import dash 
from dash import html, dcc, Input, Output
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

def route_manip(x):
    return math.atan(x / 400) * 10

#vehicle model 1 parameters
model_1_frontal_area = 1.89
model_1_wind_res_coef = 0.36
model_1_rolling_res_coef = 0.012
model_1_vehicle_mass = 2000
model_1_wheel_radius = 0.2159
model_1_max_torque = 200

#vehicle model 2 parameters
model_2_frontal_area = 1.89
model_2_wind_res_coef = 0.36
model_2_rolling_res_coef = 0.012
model_2_vehicle_mass = 2000
model_2_wheel_radius = 0.2159
model_2_max_torque = 200

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
min_vehicle_mass = 500.0
min_wheel_radius = 0.1
min_max_torque = 100.0

max_frontal_area = 3.0
max_wind_res_coef = 1.0
max_rolling_res_coef = 1.0
max_vehicle_mass = 3000.0
max_wheel_radius = 1.0
max_max_torque = 300.0

#enviorment parameters
route_func = route_manip
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
max_wind_angle = 10.0

#simulation parameters
t = 250
ts = 1
set_vel = 10
start_vel = 0
start_pos = -1000

min_t = 200
min_ts = 1
min_set_vel = 0
min_start_vel = 0
min_start_pos = -1000

max_t = 300
max_ts = 5
max_set_vel = 50
max_start_vel = 50
max_start_pos = 0

kp = 0.1
ki = 0.01
kd = 0.05
max_force = generated_force(max_torque)

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

def set_global_variables(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5):
    global frontal_area
    global wind_res_coef
    global rolling_res_coef
    global vehicle_mass 
    global wheel_radius 
    global max_torque 
    global route_func 
    global air_density 
    global g
    global wind_speed 
    global wind_angle
    global t
    global ts
    global set_vel
    global start_vel
    global start_pos
    frontal_area = v1
    wind_res_coef = v2
    rolling_res_coef = v3
    vehicle_mass = v4
    wheel_radius = v5
    max_torque = v6
    route_func = route_manip
    air_density = e1
    g = e2
    wind_speed = e3
    wind_angle = e4
    t = s1
    ts = s2
    set_vel = s3
    start_vel = s4
    start_pos = s5

def compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5):
    set_global_variables(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)
    global time_array 
    global vel_arr
    global pos_arr
    global h_arr
    global e_arr
    global es_arr
    global a_arr
    global F_arr
    global slope_arr
    global rolldown_arr
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

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

traces = [[], [], []]
last_clr_n_clicks = 0
last_add_n_clicks = 0

def add_new_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5):
    compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)
    new_trace1 = go.Scatter(
        x=time_array,
        y=vel_arr,
        mode='lines',
    )
    new_trace2 = go.Scatter(
        x=time_array,
        y=F_arr,
        mode='lines',
    )
    new_trace3 = go.Scatter(
        x=time_array,
        y=slope_arr,
        mode='lines',
    )
    traces[0].append(new_trace1)
    traces[1].append(new_trace2)
    traces[2].append(new_trace3)

def modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5):
    compute_values(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)
    if traces[0] and traces[1] and traces[2]:
        traces[0][-1]['y'] = vel_arr
        traces[1][-1]['y'] = F_arr
        traces[2][-1]['y'] = slope_arr

        
app.layout = dbc.Container(
    [
        html.H1("TEMPOMAT"),
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
                            {'label':'custom', 'value' : 'opt1'},
                            {'label':'model 1', 'value' : 'opt2'},
                            {'label':'model 2', 'value' : 'opt3'}
                        ],
                        value='opt1'
                    ),
                    html.Label('powierzchnia czołowa:'),
                    dcc.Slider(
                        id='v_slider1',
                        min=min_frontal_area,
                        max=max_frontal_area,
                        step=0.01,
                        value=frontal_area,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('współczynnik oporu toczenia:'),
                    dcc.Slider(
                        id='v_slider2',
                        min=min_rolling_res_coef,
                        max=max_rolling_res_coef,
                        step=0.01,
                        value=rolling_res_coef,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('współczynnik oporu aerodynamicznego:'),
                    dcc.Slider(
                        id='v_slider3',
                        min=min_wind_res_coef,
                        max=max_wind_res_coef,
                        step=0.01,
                        value=wind_res_coef,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('masa pojazdu:'),
                    dcc.Slider(
                        id='v_slider4',
                        min=min_vehicle_mass,
                        max=max_vehicle_mass,
                        step=0.01,
                        value=vehicle_mass,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('promień koła:'),
                    dcc.Slider(
                        id='v_slider5',
                        min=min_wheel_radius,
                        max=max_wheel_radius,
                        step=0.01,
                        value=wheel_radius,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('maksymalny moment siły silnika:'),
                    dcc.Slider(
                        id='v_slider6',
                        min=min_max_torque,
                        max=max_max_torque,
                        step=0.01,
                        value=max_torque,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.H5("parametry środowiska"),
                    html.Label('gęstość powietrza:'),
                    dcc.Slider(
                        id='e_slider1',
                        min=min_air_density,
                        max=max_air_density,
                        step=0.01,
                        value=air_density,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('przyspieszenie grawitacyjne:'),
                    dcc.Slider(
                        id='e_slider2',
                        min=min_g,
                        max=max_g,
                        step=0.01,
                        value=g,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('prędkość wiatru:'),
                    dcc.Slider( 
                        id='e_slider3',
                        min=min_wind_speed,
                        max=max_wind_speed,
                        step=0.01,
                        value=wind_speed,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('kąt wiania wiatru:'),
                    dcc.Slider(
                        id='e_slider4',
                        min=min_wind_angle,
                        max=max_wind_angle,
                        step=0.01,
                        value=wind_angle,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.H5("parametry symulacji"),
                    html.Label('czas symulacji:'),
                    dcc.Slider(
                        id='s_slider1',
                        min=min_t,
                        max=max_t,
                        step=1,
                        value=t,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('krok symulacji:'),
                    dcc.Slider(
                        id='s_slider2',
                        min=min_ts,
                        max=max_ts,
                        step=1,
                        value=ts,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('prędkość początkowa:'),
                    dcc.Slider(
                        id='s_slider3',
                        min=min_start_vel,
                        max=max_start_vel,
                        step=0.01,
                        value=start_vel,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('prędkość zadana:'),
                    dcc.Slider(
                        id='s_slider4',
                        min=min_set_vel,
                        max=max_set_vel,
                        step=0.01,
                        value=set_vel,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Label('położenie początkowe:'),
                    dcc.Slider(
                        id='s_slider5',
                        min=min_start_pos,
                        max=max_start_pos,
                        step=0.01,
                        value=start_pos,
                        marks=None,
                        tooltip={"placement" : "bottom", "always_visible" : True}
                    ),
                    html.Button('Add Trace', id='add-button', n_clicks=0),
                    html.Button('Clear', id='clr-button', n_clicks=0)
                    ], style={'width': '50%', 'margin': 'auto'}),
                ],
                width=6
            ),
            dbc.Col(
                html.Div([
                dcc.Graph(id='graph-1', figure={'data': [], 'layout': go.Layout(title='y = ax')}),
                dcc.Graph(id='graph-2', figure={'data': [], 'layout': go.Layout(title='y = a*-x')}),
                dcc.Graph(id='graph-3', figure={'data': [], 'layout': go.Layout(title='y = x*x + a')}),
                ]),
                width=6
            )
        ])
    ]
)

@app.callback(
    [
        Output('graph-1', 'figure'),
        Output('graph-2', 'figure'),
        Output('graph-3', 'figure'),
        Output('v_slider1', 'value'),
        Output('v_slider2', 'value'),
        Output('v_slider3', 'value'),
        Output('v_slider4', 'value'),
        Output('v_slider5', 'value'),
        Output('v_slider6', 'value'),
        Output('v_slider1', 'disabled'),
        Output('v_slider2', 'disabled'),
        Output('v_slider3', 'disabled'),
        Output('v_slider4', 'disabled'),
        Output('v_slider5', 'disabled'),
        Output('v_slider6', 'disabled'),
     ],
    [
        Input('dropdown_menu', 'value'),
        Input('add-button', 'n_clicks'),
        Input('clr-button', 'n_clicks'),
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
        Input('s_slider5', 'value')
     ]
)
def update_graph(dropdown, add_n_clicks, clr_n_clicks, v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5):
    global last_clr_n_clicks, last_add_n_clicks, traces

    if add_n_clicks != last_add_n_clicks:
        add_new_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)
        last_add_n_clicks = add_n_clicks
    else:
        modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)

    if clr_n_clicks != last_clr_n_clicks:
        traces = [[], [], []]
        last_clr_n_clicks = clr_n_clicks

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
        modify_last_trace(v1, v2, v3, v4, v5, v6, e1, e2, e3, e4, s1, s2, s3, s4, s5)
    
    figures = [
        {'data': traces[0], 'layout': go.Layout(title='prędkość')},
        {'data': traces[1], 'layout': go.Layout(title='siła')},
        {'data': traces[2], 'layout': go.Layout(title='nachylenie')},
    ]
    
    return figures[0], figures[1], figures[2], v1, v2, v3, v4, v5, v6, disabled, disabled, disabled, disabled, disabled, disabled

if __name__ == '__main__':
    app.run_server(debug=True)

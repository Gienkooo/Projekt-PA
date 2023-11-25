import math
import plotly.graph_objects as go
import dash 
from dash import html
import dash_core_components as dcc
import dash_bootstrap_components as dbc


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




#for _ in range(t // ts):

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
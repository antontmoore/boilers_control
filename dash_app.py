from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go
import numpy as np
import dash_bootstrap_components as dbc
from dash_bootstrap_templates import load_figure_template
from simulation.simulator import Simulator
from controller.naiive_controller import NaiiveController
from controller.dynamic_programing import DynamicProgramingController
from controller.always_on import AlwaysOnController
from simulation.simulator import SimulationResults
from constants import J_in_kWh
from constants import SETPOINT__degC
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import DEFAULT_CONTROL_HORIZON__sec
from constants import DEFAULT_SIMULATION_PERIOD__sec
from constants import DEFAULT_START_TEMPERATURE__degC
from constants import DEFAULT_TIME_STEP__sec
import datetime

# stylesheet with the .dbc class to style  dcc, DataTable and AG Grid components with a Bootstrap theme
dbc_css = "https://cdn.jsdelivr.net/gh/AnnMarieW/dash-bootstrap-templates/dbc.min.css"
template = "slate_dark"
load_figure_template(template)

app = Dash(__name__, external_stylesheets=[dbc.themes.DARKLY, dbc.icons.FONT_AWESOME, dbc_css])


header = html.H4(
    "Microgrid Controller Test Environment", className="bg-success text-white p-2 mb-2 text-center"
)

buttons = html.Div(
    [
        dbc.Button("   ▶   ", size="lg", className="me-1"),
        dbc.Button("↺", size="sm", color="secondary"),
    ]
)


def get_results_table(sim_result: SimulationResults,
                      always_on_result: SimulationResults):
    """
        Function creating the table with integral results, given two Results data.

        :param   sim_result:          result of current simulation
        :param   always_on_result:    result of simulation with boilers always switched on

        :return: results_table        Table object for app
    """

    delta_costs = round(always_on_result.total_costs__usd - sim_result.total_costs__usd, 2)
    delta_energy = round((always_on_result.total_energy__J - sim_result.total_energy__J) / J_in_kWh, 2)

    row1 = html.Tr([html.Td("Total energy, kWh"), html.Td(round(sim_result.total_energy__J / J_in_kWh, 2))])
    row2 = html.Tr([html.Td("Energy saved, kWh"), html.Td(delta_costs)])
    row3 = html.Tr([html.Td("Total costs, $"), html.Td(round(sim_result.total_costs__usd, 2))])
    row4 = html.Tr([html.Td("Money saved, $"), html.Td(delta_energy)])

    table_body = [html.Tbody([row1, row2, row3, row4])]
    results_table = dbc.Table(
        table_body,
        bordered=True,
        dark=True,
        hover=True,
        responsive=True,
        striped=True,
        id="results_table"
    )

    return results_table


def get_figures(sim_result):
    """
        Function creating the figures given the Results data

        :param sim_result:  result of current simulation
        :return:            figures of temperature, power, avg power and costs
    """

    timestamps__sec = np.linspace(0, 24 * 3600, int(24 * 3600 / step_size__sec))
    timestamps__hm = [str(datetime.timedelta(seconds=ts)).split(':') for ts in timestamps__sec]
    timestamps__hm = [ts[0] + ':' + ts[1] for ts in timestamps__hm]

    fig_temp_dict = dict(zip(["data", "layout", "frames"], [[], {}, []]))
    fig_temp_dict["data"] = [go.Scatter(x=timestamps__hm, y=sim_result.temperature_trends[:, hn], name="house "+str(hn+1)) for hn in
                             range(5)]  # [4, 7, 5, 3, 2])
    fig_temp_dict["layout"]["margin"] = dict(l=0, r=0, b=1, t=0)
    figure_temperature = go.Figure(fig_temp_dict)
    figure_temperature.update_yaxes(title_text="Temperature, °C")
    figure_temperature.update_layout(template=template, legend=dict(x=1, y=0, xanchor='right', yanchor='bottom'))

    figure_temperature.add_vrect(x0=6 * 3600 / step_size__sec, x1=9 * 3600 / step_size__sec,
                                 fillcolor="darkslateblue", opacity=0.2, layer="below", line_width=0,
                                 )
    figure_temperature.add_vrect(x0=7 * 3600 / step_size__sec, x1=8 * 3600 / step_size__sec,
                                 fillcolor="darkslateblue", opacity=0.4, layer="below", line_width=0,
                                 )
    figure_temperature.add_vrect(x0=16 * 3600 / step_size__sec, x1=21 * 3600 / step_size__sec,
                                 fillcolor="darkslateblue", opacity=0.2, layer="below", line_width=0,
                                 )
    figure_temperature.add_vrect(x0=17 * 3600 / step_size__sec, x1=20 * 3600 / step_size__sec,
                                 fillcolor="darkslateblue", opacity=0.4, layer="below", line_width=0,
                                 )

    fig_power_dict = dict(zip(["data", "layout", "frames"], [[], {}, []]))
    fig_power_dict["data"] = [
        go.Scatter(
            x=timestamps__hm, y=sim_result.power_trend, name="power", line={'color': 'royalblue'}
        ),
        go.Scatter(
            x=timestamps__hm, y=sim_result.avg_power_trend, name="avg power", line={'color': 'darkseagreen'})]
    fig_power_dict["layout"]["margin"] = dict(l=0, r=0, b=0, t=1)
    figure_power = go.Figure(fig_power_dict)
    figure_power.update_yaxes(title_text="Power, W")
    figure_power.update_layout(template=template, legend=dict(x=1, y=0, xanchor='right', yanchor='bottom'))
    figure_power.add_vrect(x0=8 * 3600 / step_size__sec, x1=9 * 3600 / step_size__sec,
                           fillcolor="tomato", opacity=0.4, layer="below", line_width=0,
                           )
    figure_power.add_vrect(x0=18 * 3600 / step_size__sec, x1=20 * 3600 / step_size__sec,
                           fillcolor="tomato", opacity=0.4, layer="below", line_width=0,
                           )

    fig_cost_dict = dict(zip(["data", "layout", "frames"], [[], {}, []]))
    fig_cost_dict["data"] = [go.Scatter(x=timestamps__hm, y=sim_result.cost_trend, line={'color': 'peru'})]
    fig_cost_dict["layout"]["margin"] = dict(l=0, r=0, b=0, t=1)
    figure_cost = go.Figure(fig_cost_dict)
    figure_cost.update_yaxes(title_text="Costs, $")
    figure_cost.update_layout(template=template)
    figure_cost.add_vrect(x0=8 * 3600 / step_size__sec, x1=9 * 3600 / step_size__sec,
                          fillcolor="tomato", opacity=0.4, layer="below", line_width=0)
    figure_cost.add_vrect(x0=18 * 3600 / step_size__sec, x1=20 * 3600 / step_size__sec,
                          fillcolor="tomato", opacity=0.4, layer="below", line_width=0)

    return figure_temperature, figure_power, figure_cost


def get_settings_panel():
    """
        Function creating settings panel
    """

    settings_inputs = html.Div(
        [
            dbc.Label("Temperature setpoint", color="light"),
            dcc.Slider(40, 60, step=1,
                       marks={
                                40: {'label': '40°C'},
                                45: {'label': '45°C'},
                                50: {'label': '50°C'},
                                55: {'label': '55°C'},
                                60: {'label': '60°C'},
                        }, value=SETPOINT__degC, id="setpoint_slider"),
            html.P(),
            dbc.Label("Max total instantaneous power", color="light"),
            dcc.Slider(2000, 10000, step=2000,
                       marks={
                           2000: {'label': '2kW'},
                           4000: {'label': '4kW'},
                           6000: {'label': '6kW'},
                           8000: {'label': '8kW'},
                           10000: {'label': '10kW'},
                       }, value=MAXIMUM_INSTANTANEOUS_POWER__W, className="dbc", id="max_power_slider"),
            html.P(),
            dbc.Label("Max total avg 15min power", color="light"),
            dcc.Slider(800, 1600, step=100,
                       marks={
                           800: {'label': '0.8kW'},
                           1000: {'label': '1kW'},
                           1200: {'label': '1.2kW'},
                           1400: {'label': '1.4kW'},
                           1600: {'label': '1.6kW'},
                       }, value=MAXIMUM_15MIN_POWER__W, id="max_avg_power_slider"),
            html.P(),
            html.Div(
                dbc.Select(
                    ["naiive controller", "dynamic programming"],
                    "naiive controller",
                    id="controller-select",
                    class_name="bg-dark text-white",
                    ),
            ),
            html.P(),
            dbc.Label("Control horizon, hours", color="light"),

            dcc.Slider(1, 12, step=1,
                       marks={
                           1: {'label': '1'},
                           4: {'label': '4'},
                           8: {'label': '8'},
                           12: {'label': '12'},
                       }, value=int(DEFAULT_CONTROL_HORIZON__sec / 3600),
                       id="horizon_slider"),
            # dbc.Input(placeholder="Horizon in hours...", value=int(DEFAULT_CONTROL_HORIZON__sec/3600), valid=None,
            #           id="horizon_setting"),
            html.P(),
            # dbc.Label("Time step, sec", color="light"),
            # dbc.Input(placeholder="Timestep in sec...", value=DEFAULT_TIME_STEP__sec, valid=None),
            # html.P(),
            dbc.Label("Start temperatures, °C", color="light"),
            dbc.InputGroup(
                [dbc.InputGroupText("house 1"),
                 dbc.Input(placeholder="Temperature in °C...", value=DEFAULT_START_TEMPERATURE__degC, id="house1_temperature")], className="mb-3",
            ),
            dbc.InputGroup(
                [dbc.InputGroupText("house 2"),
                 dbc.Input(placeholder="Temperature in °C...", value=DEFAULT_START_TEMPERATURE__degC, id="house2_temperature")], className="mb-3",
            ),
            dbc.InputGroup(
                [dbc.InputGroupText("house 3"),
                 dbc.Input(placeholder="Temperature in °C...", value=DEFAULT_START_TEMPERATURE__degC, id="house3_temperature")], className="mb-3",
            ),
            dbc.InputGroup(
                [dbc.InputGroupText("house 4"),
                 dbc.Input(placeholder="Temperature in °C...", value=DEFAULT_START_TEMPERATURE__degC, id="house4_temperature")], className="mb-3",
            ),
            dbc.InputGroup(
                [dbc.InputGroupText("house 5"),
                 dbc.Input(placeholder="Temperature in °C...", value=DEFAULT_START_TEMPERATURE__degC, id="house5_temperature")], className="mb-3",
            ),
        ]
    )

    settings_panel = html.Div(
        [
            dbc.Button(
                "⚙ Settings", id="settings_button", outline=True, color="secondary", className="me-1", n_clicks=0
            ),
            dbc.Collapse(
                dbc.Card(
                    settings_inputs
                ),
                id="settings_collapse",
                is_open=False,
            ),
        ]
    )
    return settings_panel

# first simulation is done with deafult settings
step_size__sec = DEFAULT_TIME_STEP__sec
horizon__sec = DEFAULT_CONTROL_HORIZON__sec
setpoint__degC = SETPOINT__degC
max_instantaneous_power__W = MAXIMUM_INSTANTANEOUS_POWER__W
max_15min_power__W = MAXIMUM_15MIN_POWER__W
simulation_period__sec = DEFAULT_SIMULATION_PERIOD__sec
start_temperatures__degC = DEFAULT_START_TEMPERATURE__degC * np.ones((5,))

naiive_controller = NaiiveController(
    horizon_time__sec=horizon__sec,
    setpoint__degC=setpoint__degC,
    max_instantaneous_power__W=max_instantaneous_power__W,
    max_15min_power__W=max_15min_power__W
)

always_on_controller = AlwaysOnController(
    horizon_time__sec=horizon__sec,
    setpoint__degC=setpoint__degC
)

simulator = Simulator()
result = simulator.simulate_day(controller=naiive_controller,
                                setpoint__degC=setpoint__degC,
                                timestep__sec=step_size__sec,
                                period__sec=simulation_period__sec,
                                start_temperatures__degC=start_temperatures__degC)
always_on_result = simulator.simulate_day(controller=always_on_controller,
                                          setpoint__degC=setpoint__degC,
                                          timestep__sec=step_size__sec,
                                          period__sec=simulation_period__sec,
                                          start_temperatures__degC=start_temperatures__degC)

figure_temperature, figure_power, figure_cost = get_figures(result)
results_table = get_results_table(result, always_on_result)
settings_panel = get_settings_panel()


results_table_fade = dbc.Fade(results_table, is_in=True, id="results_table_fade")

# creating app layout of all components
app.layout = dbc.Container(
    [
        header,
        dbc.Row([
            dbc.Col([
                buttons,
                html.Br(),
                results_table_fade,
                settings_panel,
            ],  width=2),
            dbc.Col([
                dcc.Graph(figure=figure_temperature, id="temperature_graph"),
                dbc.Tabs(
                    [
                        dbc.Tab(dcc.Graph(figure=figure_power, id="power_graph"), label="Power"),
                        dbc.Tab(dcc.Graph(figure=figure_cost, id="cost_graph"), label="Costs")
                    ]
                )
            ], width=8),
        ]),
    ],
    fluid=True,
    className="dbc",
)


@app.callback(
    Output("settings_collapse", "is_open"),
    [Input("settings_button", "n_clicks")],
    [State("settings_collapse", "is_open")],
)
def toggle_collapse(n, is_open):
    if n:
        return not is_open
    return is_open


@app.callback(
    Output("results_table_fade", "children"),
    Output("temperature_graph", "figure"),
    Output("power_graph", "figure"),
    Output("cost_graph", "figure"),
    Output("house1_temperature", "value"),
    Output("house2_temperature", "value"),
    Output("house3_temperature", "value"),
    Output("house4_temperature", "value"),
    Output("house5_temperature", "value"),
    Input("setpoint_slider", "value"),
    Input("max_power_slider", "value"),
    Input("max_avg_power_slider", "value"),
    Input("controller-select", "value"),
    Input("horizon_slider", "value"),
    Input("house1_temperature", "value"),
    Input("house2_temperature", "value"),
    Input("house3_temperature", "value"),
    Input("house4_temperature", "value"),
    Input("house5_temperature", "value"),
)
def change_general_settings(
        setpoint__degC,
        max_instantaneous_power__W,
        max_15min_power__W,
        controller_name,
        horizon__h,
        house1_t, house2_t, house3_t, house4_t, house5_t
):
    def bound_temperature(x):
        if x < 10:
            x = 10
        if x > 55:
            x = 55
        return x

    t_list = list(map(float, [house1_t, house2_t, house3_t, house4_t, house5_t]))
    t_list = list(map(bound_temperature, t_list))

    start_temperatures__degC = np.array(t_list, dtype=float)
    horizon__sec = horizon__h * 3600
    simulator = Simulator()
    if controller_name == "naiive controller":
        controller = NaiiveController(
            horizon_time__sec=horizon__sec,
            setpoint__degC=setpoint__degC,
            max_instantaneous_power__W=max_instantaneous_power__W,
            max_15min_power__W=max_15min_power__W
        )
    else:
        controller = DynamicProgramingController(
            horizon_time__sec=horizon__sec,
            setpoint__degC=setpoint__degC,
            max_instantaneous_power__W=max_instantaneous_power__W,
            max_15min_power__W=max_15min_power__W
        )

    result = simulator.simulate_day(
        controller=controller,
        setpoint__degC=setpoint__degC,
        timestep__sec=step_size__sec,
        period__sec=simulation_period__sec,
        start_temperatures__degC=start_temperatures__degC
    )

    figure_temperature, figure_power, figure_cost = get_figures(result)

    return get_results_table(result, always_on_result), figure_temperature, figure_power, figure_cost, *t_list


if __name__ == "__main__":
    app.run_server(debug=True)

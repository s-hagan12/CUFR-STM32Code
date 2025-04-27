from dash import Dash, html, dcc, callback, Output, Input
import plotly.express as px
import pandas as pd
import dash_daq as daq
import random

df = pd.read_csv('https://raw.githubusercontent.com/plotly/datasets/master/gapminder_unfiltered.csv')

app = Dash()

app.layout = [
    html.H1(children='CUFR Test App', style={'textAlign':'center'}),
    daq.Gauge(
        id='my-gauge-1',
        label="Default",
        value=6
    ),
    dcc.Slider(
        id='my-gauge-slider-1',
        min=0,
        max=60,
        step=5,
        value=0
    ),
    dcc.Interval(
            id='interval-component',
            interval=1*1000, # in milliseconds
            n_intervals=0
        )
]

@callback(Output('my-gauge-1', 'value'), Input('interval-component', 'n_intervals'))
def update_output(value):
    return gauge_val

gauge_val = 0

def rand_val():
    while(True):
        gauge_val = random.random()*60
        


if __name__ == '__main__':
    app.run(debug=True)
    rand_val()
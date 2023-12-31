import numpy as np

# power of boiler heater default value
DEFAULT_HEATER_POWER__W = 2000.

# boiler volume capacity default value
DEFAULT_BOILER_CAPACITY__m3 = 80 * 0.001

# temperature setpoint of boiler
SETPOINT__degC = 55.

# maximum instantaneous power consumed by microgrid
MAXIMUM_INSTANTANEOUS_POWER__W = 4_000.

# maximum average power consumed by microgrid in 15 minutes
MAXIMUM_15MIN_POWER__W = 900.

# ambient temperature outside the boiler
AMBIENT_TEMPERATURE__degC = 20.

# default value for start water temperatures
DEFAULT_START_TEMPERATURE__degC = 48.

# default horizon of modelling while generating control
DEFAULT_CONTROL_HORIZON__sec = 12 * 3600

# default period to split the day of simulation
DEFAULT_SIMULATION_PERIOD__sec = 3600

# full time for boiler to heat from incoming water temperature to set point
DEFAULT_FULL_HEATING_TIME__sec = 2 * 3600 + 2 * 60

# water temperature coming inside boiler to be heated
INCOMING_WATER_TEMPERATURE__degC = 20.

# table value of specific heat capacity for water
SPECIFIC_HEAT_CAPACITY__J_per_kg_degC = 4_184.

# table value for water density
WATER_DENSITY__kg_per_m3 = 1000.

# time step for controller
DEFAULT_TIME_STEP__sec = 60

# coefficients to calculate J-functional (metrics) in dp controller
CONTROLLER_COEFFICIENTS_DP = {"temp_diff": 3., "consumption": 1.2, "cost": 1.}

# coefficients to calculate J-functional (metrics) in mpc controller
CONTROLLER_COEFFICIENTS_MPC = {"temp_diff": 0.4, "consumption": 1., "cost": 15.}

# 15 minute period in seconds
PERIOD_15MIN__sec = 15 * 60

# given consumption of water by one house [time in sec from 00:00, consumption in m3/sec]
water_consumption__m3_per_sec = np.array([[0,         0.],
                                          [6 * 3600,  0.],
                                          [7 * 3600,  0.0115 * 0.001],
                                          [8 * 3600,  0.0115 * 0.001],
                                          [9 * 3600,  0.],
                                          [16 * 3600, 0.],
                                          [17 * 3600, 0.0115 * 0.001],
                                          [20 * 3600, 0.0115 * 0.001],
                                          [21 * 3600, 0.],
                                          [24 * 3600, 0.],
                                          ]
                                         )

# given price of energy [time in sec from 00:00, price in usd/kWh]
energy_price__usd_per_J = np.array([[0,         0.001],
                                    [8 * 3600,  0.001],
                                    [8 * 3600,  0.02],
                                    [9 * 3600,  0.02],
                                    [9 * 3600,  0.001],
                                    [18 * 3600, 0.001],
                                    [18 * 3600, 0.02],
                                    [20 * 3600, 0.02],
                                    [20 * 3600, 0.001],
                                    [24 * 3600, 0.001],
                                    ]
                                   )
J_in_kWh = 1000 * 3600
energy_price__usd_per_J[:, 1] *= 100 / J_in_kWh

from controller.dynamic_programing import DynamicProgramingController
import numpy as np


dpc = DynamicProgramingController(horizon_time__sec=7200)
dpc.generate_control(start_temperatures__degC=52 * np.ones((5,)),
                     current_time__sec=18 * 3600,
                     step_size__sec=60)
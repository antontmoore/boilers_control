import numpy as np
from .controller import Controller
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import SETPOINT__degC
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3
from constants import SURGE_CONSUMPTION_THRESHOLD__M_PER_SEC
from constants import DEFAULT_TIME_STEP__sec
from constants import DEFAULT_CONTROL_HORIZON__sec
from constants import WATER_DENSITY__kg_per_m3
from constants import INCOMING_WATER_TEMPERATURE__degC
from constants import water_consumption__m3_per_sec
from numpy.typing import NDArray
from .utils import CircularBuffer


class NaiiveController(Controller):
    """
        Naiive controller.
        Calculate aprroximate time needed to go to setpoint.
        Splits work periods into pieces for each house.
        It boils only before consumption starts.
    """

    def __init__(self,
                 number_of_houses: int = 5,
                 horizon_time__sec: int = DEFAULT_CONTROL_HORIZON__sec,
                 setpoint__degC: float = SETPOINT__degC,
                 max_instantaneous_power__W: float = MAXIMUM_INSTANTANEOUS_POWER__W,
                 max_15min_power__W: float = MAXIMUM_15MIN_POWER__W,
                 heater_power__W: float = DEFAULT_HEATER_POWER__W,
                 boiler_capacity__m3: float = DEFAULT_BOILER_CAPACITY__m3,
                 ):

        super().__init__(number_of_houses,
                         horizon_time__sec,
                         setpoint__degC,
                         heater_power__W,
                         boiler_capacity__m3,
                         max_instantaneous_power__W,
                         max_15min_power__W)
        self.horizon__steps = int(self.horizon_time__sec / DEFAULT_TIME_STEP__sec)

    def generate_control(self,
                         start_temperatures__degC: NDArray,
                         current_time__sec: int,
                         step_size__sec: int,
                         ) -> NDArray:
        """
            Function generating control for the future time steps, starting from given temperatures.

            :param   start_temperatures__degC:    temperature at every boiler          [number_of_houses]
            :param   current_time__sec:           current time from 00:00 in seconds
            :param   step_size__sec:              step size for control generation

            :return: control                      control actions for all houses       [horizon_steps, number_of_houses]
        """

        self.horizon__steps = int(self.horizon_time__sec / step_size__sec)
        time_to_surge__sec = self.calc_time_left_to_flow_growth(current_time__sec)
        steps_to_surge = int(time_to_surge__sec / step_size__sec)

        max_power__W = min(self.max_15min_power__W, self.max_instantaneous_power__W)
        max_energy_left__J = max_power__W * time_to_surge__sec

        max_final_temperature__degC = (
                (1 / self.number_of_houses) *
                (sum(start_temperatures__degC) +
                 max_energy_left__J / SPECIFIC_HEAT_CAPACITY__J_per_kg_degC / self.total_water_mass__kg
                 )
        )

        # if we start to heat right now
        heat_schedule, steps_to_setpoint = self.calc_heat_schedule(
            start_temperatures__degC,
            step_size__sec
        )

        return schedule

    @staticmethod
    def calc_time_left_to_flow_growth(current_time__sec: int) -> int:
        """
            Calculates time left before water consumption.
            If water is consumed now, returns 0.

            :param current_time__sec: current time from 00:00 in seconds

            :return: time_before_water_consumption
        """

        current_flow__m3_per_sec = np.interp(
            current_time__sec,
            water_consumption__m3_per_sec[:, 0],
            water_consumption__m3_per_sec[:, 1]
        )

        if current_flow__m3_per_sec > SURGE_CONSUMPTION_THRESHOLD__M_PER_SEC:
            return 0

        min_time_before_growth = 24 * 3600
        for t_idx in range(water_consumption__m3_per_sec.shape[0]-1):
            time__sec, value__m3_per_sec = water_consumption__m3_per_sec[t_idx]
            next_value__m3_per_sec = water_consumption__m3_per_sec[t_idx + 1, 1]

            if value__m3_per_sec < SURGE_CONSUMPTION_THRESHOLD__M_PER_SEC < next_value__m3_per_sec:
                # growth in flow

                if current_time__sec < time__sec:
                    time_before_this_growth = time__sec - current_time__sec
                else:
                    time_before_this_growth = time__sec + (24 * 3600 - current_time__sec)
                min_time_before_growth = min(min_time_before_growth, time_before_this_growth)

        return int(min_time_before_growth)

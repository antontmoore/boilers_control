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
from constants import water_consumption__m3_per_sec
from numpy.typing import NDArray


class NaiiveController(Controller):
    """
        Naiive controller.
        Calculate aprroximate time needed to got to setpoint.
        Splits work periods into pieces for each house.
        It boils only before consumption starts.
    """

    def __init__(self,
                 number_of_houses: int = 5,
                 horizon_time__sec: int = 3 * 3600,
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
                          step_size__sec: int
                          ):
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
        heat_schedule_size__steps = heat_schedule.shape[0]

        if steps_to_setpoint < steps_to_surge:
            schedule = np.vstack((
                np.zeros((steps_to_surge - steps_to_setpoint, self.number_of_houses)),
                heat_schedule[:self.horizon__steps - (steps_to_surge - steps_to_setpoint)]
            ))
        else:
            schedule = heat_schedule

        trends = self.simulate_schedule(start_temperatures__degC, schedule, step_size__sec)

        self.plot_trends(trends)

    @staticmethod
    def calc_time_left_to_flow_growth(current_time__sec):
        """
            Calculates time left before water consumption.
            If water is consumed now, returns 0.

            :param current_time__sec: current time from 00:00 in seconds

            :return: time_before_water_consumption
        """

        current_flow__m3_per_sec = np.interp(
            current_time__sec,
            water_consumption__m3_per_sec[:, 0],
                                             water_consumption__m3_per_sec[:, 1],
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

    def calc_heat_schedule(self,
                           current_temperatures__degC: float,
                           step_size__sec: int,
                           ):

        n_simulteneous = int(self.max_instantaneous_power__W / self.heater_power__W)
        heating_time_part = self.max_15min_power__W / (n_simulteneous * self.heater_power__W)
        if heating_time_part > 1.:      # no constraints
            heating_time_part = 1.

        heating_inside_15min__sec = int(15 * 60 * heating_time_part)
        nonheating_inside_15min__sec = 15 * 60 - heating_inside_15min__sec

        heating_inside_15min__steps = int(heating_inside_15min__sec / step_size__sec)
        nonheating_inside_15min__steps = int(15 * 60 / step_size__sec) - heating_inside_15min__steps
        heat_to_all_relation = heating_inside_15min__steps / \
                               (heating_inside_15min__steps + nonheating_inside_15min__steps)

        # max_current_temperature__degC = np.max(current_temperatures__degC)
        # temperature_differences__degC = max_current_temperature__degC - current_temperatures__degC
        temperature_differences__degC = self.setpoint__degC - current_temperatures__degC

        time_to_setpoint__sec = self.temp_to_time__sec_per_degC * temperature_differences__degC
        steps_to_setpoint_by_house = np.ceil(time_to_setpoint__sec / step_size__sec).astype(int)

        # calculate schedule to move to the max of current temperatures
        schedule_length = int(np.sum(steps_to_setpoint_by_house) / heat_to_all_relation / n_simulteneous)
        schedule = np.zeros((schedule_length, self.number_of_houses), dtype=bool)
        schedule = np.zeros((self.horizon__steps, self.number_of_houses), dtype=bool)

        ids_and_steps = [[idx, steps_to_setpoint_by_house[idx]] for idx in range(self.number_of_houses)]
        ids_and_steps.sort(key=lambda pair: -pair[1])
        steps_to_setpoint = self.horizon__steps
        step = 0

        while step < self.horizon__steps:
            for j in range(n_simulteneous):
                idx, steps_needed = ids_and_steps[j]
                schedule[step: step + heating_inside_15min__steps, idx] = True
                ids_and_steps[j][1] -= heating_inside_15min__steps
                # if ids_and_steps[j][1] < 0:
                #     ids_and_steps[j][1] = 0

            step += heating_inside_15min__steps
            ids_and_steps.sort(key=lambda pair: -pair[1])
            if ids_and_steps[0][1] < 0:
                # all temperatures are at setpoint
                steps_to_setpoint = min(step, steps_to_setpoint)
            step += nonheating_inside_15min__steps

        return schedule, steps_to_setpoint

import numpy as np
from controller.controller import Controller
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import SETPOINT__degC
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3
from constants import INCOMING_WATER_TEMPERATURE__degC
from constants import WATER_DENSITY__kg_per_m3
from constants import DEFAULT_TIME_STEP__sec
from constants import water_consumption__m3_per_sec
from constants import energy_price__usd_per_J
from numpy.typing import NDArray
from math import ceil


class DynamicProgramingController(Controller):
    """
        Dynamic Programing controller. Controller has the horizon and split it into 15min intervals.
        Then it calculate the dp-table, which has dimensions [number_of_temperature_steps, number_of_15min_timesteps].
        After dynamic programming calculation of dp-table controller restores the optimal path and
        make the schedule for every single house.
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
        self.horizon_steps = int(self.horizon_time__sec / DEFAULT_TIME_STEP__sec)
        self.horizon_periods = int(self.horizon_time__sec / 15 / 60)

    def generate_control(self,
                         start_temperatures__degC: NDArray,
                         current_time__sec: int,
                         step_size__sec: int
                         ) -> NDArray:
        """
            Function generating control for the future time steps, starting from given temperatures.

            :param   start_temperatures__degC:    temperature at every boiler          [number_of_houses]
            :param   current_time__sec:           current time from 00:00 in seconds
            :param   step_size__sec:              step size for control generation

            :return: control                      control actions for all houses       [horizon_steps, number_of_houses]
        """

        period_15min__sec = 15 * 60
        self.horizon_steps = int(self.horizon_time__sec / step_size__sec)
        self.horizon_periods = int(self.horizon_time__sec / period_15min__sec)
        steps_inside_period = int(period_15min__sec / step_size__sec)

        one_step_heater_energy__J = self.heater_power__W * self.model[0].efficiency * step_size__sec

        def get_allowable_control():
            """
                Function calculating max control steps inside 15 min period.
                Needed to calculate how many step_sizes and how many boilers we can switch on inside 15 min period.
            """

            max_heaters_in_step = int(self.max_instantaneous_power__W / self.heater_power__W)

            max_control_steps = min(
                int(self.max_15min_power__W * steps_inside_period / self.heater_power__W),
                max_heaters_in_step * steps_inside_period
            )
            return max_control_steps

        def interpolate_to_horizon(x, y):
            """
                Function to interpolate water consumption and energy price for control horizon.

                :param x: array of timestamps, corresponding to values
                :param y: array of values
                :return: array of values corresponfing to every 15 min period inside horizon
            """

            x = np.hstack((x, x + 24 * 3600))
            y = np.hstack((y, y))
            timestamps__sec = np.linspace(0, self.horizon_time__sec, self.horizon_time__sec + 1) + current_time__sec
            detailed_values = np.interp(timestamps__sec, x, y)
            result = np.zeros((int(self.horizon_time__sec / period_15min__sec),))
            for j in range(result.shape[0]):
                result[j] = np.sum(detailed_values[j * period_15min__sec: (j + 1) * period_15min__sec])

            return result

        # allowable control actions are 0 and max power due to constraints
        max_control_steps_inside_15min = get_allowable_control()
        allowable_heating_steps = [0, max_control_steps_inside_15min]

        # interpolate external data of energy price and water consumption
        horizon_price__usd_per_J = interpolate_to_horizon(energy_price__usd_per_J[:, 0],
                                                          energy_price__usd_per_J[:, 1])

        horizon_consumption__m3 = interpolate_to_horizon(water_consumption__m3_per_sec[:, 0],
                                                         water_consumption__m3_per_sec[:, 1])

        # dp-table characteristics (steps and boundaries)
        current_mean_temperature__degC = np.mean(start_temperatures__degC)

        delta_temp_mean__degC = (one_step_heater_energy__J * max_control_steps_inside_15min /
                                 SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
                                 self.total_water_mass__kg)

        max_possible_consumption__m3_per_sec = np.max(water_consumption__m3_per_sec[:, 1]) * self.number_of_houses

        minimum_temperature__degC = (
            current_mean_temperature__degC -
            WATER_DENSITY__kg_per_m3 * (max_possible_consumption__m3_per_sec / self.total_water_mass__kg) *
            (current_mean_temperature__degC - INCOMING_WATER_TEMPERATURE__degC) * (5 * 3600)
        )

        number_of_temp_steps = ceil((self.setpoint__degC - minimum_temperature__degC) / delta_temp_mean__degC)
        temperature_step__degC = (self.setpoint__degC - minimum_temperature__degC) / (number_of_temp_steps - 1)

        # functions to set correspondings between temperature and row inside dp-table
        def idx_to_temp(idx):
            return minimum_temperature__degC + idx * temperature_step__degC

        def temp_to_idx(t):
            return int(round((t - minimum_temperature__degC) / temperature_step__degC))

        # intialize dp-table and auxilary tables
        dp_value = 1e9 * np.ones((number_of_temp_steps, self.horizon_periods + 1))
        dp_value[temp_to_idx(current_mean_temperature__degC), 0] = 0

        came_from = -1 * np.ones((number_of_temp_steps, self.horizon_periods + 1), dtype=int)
        boilers_switched_on = np.zeros((number_of_temp_steps, self.horizon_periods + 1), dtype=int)

        # coefficeints to calc functional J
        j_coeff_delta_t = 3.
        j_coeff_cons = 1.2
        j_coeff_price = 1.

        # filling the dp-value
        for col in range(self.horizon_periods):
            price = horizon_price__usd_per_J[col]
            consumption__kg = horizon_consumption__m3[col] * WATER_DENSITY__kg_per_m3 * self.number_of_houses

            for row in range(number_of_temp_steps):
                this_temperature__degC = idx_to_temp(row)
                for control_steps in allowable_heating_steps:
                    heating__J = control_steps * one_step_heater_energy__J

                    # calculate finctional J value
                    j_new = (
                        dp_value[row, col] +
                        j_coeff_price * price * heating__J +
                        j_coeff_delta_t * (self.setpoint__degC - this_temperature__degC) +
                        j_coeff_cons * consumption__kg * (self.setpoint__degC - this_temperature__degC)
                    )

                    # new temperature and new row inside table
                    mean_temperature_new = (
                        this_temperature__degC +
                        heating__J / SPECIFIC_HEAT_CAPACITY__J_per_kg_degC / self.total_water_mass__kg +
                        consumption__kg / self.total_water_mass__kg *
                        (INCOMING_WATER_TEMPERATURE__degC - this_temperature__degC)
                    )
                    mean_temperature_new = min(self.setpoint__degC, mean_temperature_new)
                    new_idx = temp_to_idx(mean_temperature_new)

                    # compare and decide if we need to renew dp-value
                    if j_new < dp_value[new_idx, col + 1]:
                        dp_value[new_idx, col + 1] = j_new
                        came_from[new_idx, col + 1] = row
                        boilers_switched_on[new_idx, col + 1] = control_steps

        # making the heat schedule for 15min periods
        heating_mean_schedule = np.zeros((self.horizon_periods + 1,), dtype=int)

        idx = np.argmin(dp_value[:, -1])
        for col in range(dp_value.shape[1] - 1, -1, -1):
            heating_mean_schedule[col] = boilers_switched_on[idx, col]
            idx = came_from[idx, col]

        # refining heat schedule from 15min period to step_size scale
        schedule = self.calc_heat_schedule(heating_mean_schedule[1:], start_temperatures__degC, step_size__sec)

        return schedule

    def calc_heat_schedule(self,
                           mean_schedule: NDArray,
                           temperatures__degC: NDArray,
                           step_size__sec: int,
                           ):
        """
            Function calculate refined heat schedule from general 15min periods' schedule.
            It receives mean_schedule - the number of boilers switched on during every 15min period
            and calculates current temperatures for every timestep.
        """

        # we can simulteneously turn on n_simulteneous boilers
        n_simulteneous = int(self.max_instantaneous_power__W / self.heater_power__W)
        steps_inside_period = int(15 * 60 / step_size__sec)

        # the difference of temperatures added by one switched on
        add_temperature_in_one_step__degC = (
            self.model[0].heater_power__W * self.model[0].efficiency * step_size__sec /
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
            (self.model[0].water_mass__kg)
        )

        schedule = np.zeros((0, self.number_of_houses), dtype=bool)
        for period in range(self.horizon_periods):
            steps_to_heat = mean_schedule[period]

            period_schedule = np.zeros((steps_inside_period, self.number_of_houses), dtype=bool)
            timestep_inside_period = 0
            while steps_to_heat > 0:
                for houses_switched_on in range(n_simulteneous):
                    min_temp_idx = np.argmin(temperatures__degC)
                    temperatures__degC[min_temp_idx] += add_temperature_in_one_step__degC
                    period_schedule[timestep_inside_period, min_temp_idx] = True
                    steps_to_heat -= 1

                timestep_inside_period += 1

            schedule = np.vstack((schedule, period_schedule))

        return schedule

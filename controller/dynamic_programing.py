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
        Dynamic Programing controller.

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
                         ) -> NDArray:
        """
            Function generating control for the future time steps, starting from given temperatures.

            :param   start_temperatures__degC:    temperature at every boiler          [number_of_houses]
            :param   current_time__sec:           current time from 00:00 in seconds
            :param   step_size__sec:              step size for control generation

            :return: control                      control actions for all houses       [horizon_steps, number_of_houses]
        """

        self.horizon__steps = int(self.horizon_time__sec / step_size__sec)

        one_heater_power__W = self.model[0].heater_power__W
        allowable_energy_adding__J = [p * one_heater_power__W * step_size__sec for p in range(int(self.max_instantaneous_power__W / one_heater_power__W))]
        max_allowable_energy_15min__J = 15 * 60 * self.max_15min_power__W

        current_mean_temperature__degC = np.mean(start_temperatures__degC)

        boilers_mass__kg = self.number_of_houses * self.model[0].boiler_capacity__m3 * WATER_DENSITY__kg_per_m3

        inst_power_efficient__W = self.max_instantaneous_power__W * self.model[0].efficiency
        heating_energy_one_step__J = inst_power_efficient__W * step_size__sec
        delta_temp_mean__degC = (heating_energy_one_step__J /
                                 SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
                                 boilers_mass__kg)
        max_possible_consumtion__m3_per_sec = np.max(water_consumption__m3_per_sec[:, 1])
        minimum_temperature__degC = (
            current_mean_temperature__degC -
            WATER_DENSITY__kg_per_m3 * (max_possible_consumtion__m3_per_sec / boilers_mass__kg) *
            (current_mean_temperature__degC - INCOMING_WATER_TEMPERATURE__degC) * self.horizon_time__sec
        )

        number_of_temp_steps = ceil((self.setpoint__degC - minimum_temperature__degC) / delta_temp_mean__degC)

        timestamps__sec = np.linspace(0, self.horizon_time__sec, self.horizon__steps) + current_time__sec
        horizon_price = np.interp(timestamps__sec, energy_price__usd_per_J[:, 0], energy_price__usd_per_J[:, 1])
        horizon_consumption__m3_per_sec = np.interp(timestamps__sec,
                                                    water_consumption__m3_per_sec[:, 0],
                                                    water_consumption__m3_per_sec[:, 1])
        horizon_consumption__m3_per_sec[:60] *= 0.

        idx_to_temp = lambda idx: minimum_temperature__degC + idx * delta_temp_mean__degC
        temp_to_idx = lambda t: int(round((t - minimum_temperature__degC) / delta_temp_mean__degC))

        dp_value = 1e9 * np.ones((number_of_temp_steps + 1, self.horizon__steps + 1))
        dp_value[temp_to_idx(current_mean_temperature__degC), 0] = 0


        came_from = -1 * np.ones((number_of_temp_steps + 1, self.horizon__steps + 1), dtype=int)
        boilers_switched_on = np.zeros((number_of_temp_steps + 1, self.horizon__steps + 1), dtype=int)
        last_15min_energy__J = \
            np.zeros((number_of_temp_steps + 1, self.horizon__steps + 1, int(15 * 60 / step_size__sec)))

        j_coeff_deltaT = 5000.

        for col in range(self.horizon__steps):
            price = horizon_price[col]
            consumption__m3 = horizon_consumption__m3_per_sec[col] * step_size__sec
            consumption__kg = consumption__m3 * WATER_DENSITY__kg_per_m3

            for row in range(number_of_temp_steps):
                energy_already_consumed_in_last_15min__J = np.sum(last_15min_energy__J[row, col, 1:])
                this_temperature__degC = idx_to_temp(row)
                for heating__J in allowable_energy_adding__J:

                    if energy_already_consumed_in_last_15min__J + heating__J > max_allowable_energy_15min__J:
                        continue

                    j_new = (
                        dp_value[row, col] + price * heating__J +
                        j_coeff_deltaT * consumption__m3 * (self.setpoint__degC - this_temperature__degC)
                    )

                    mean_temperature_new = (
                        this_temperature__degC +
                        heating__J / SPECIFIC_HEAT_CAPACITY__J_per_kg_degC / boilers_mass__kg +
                        consumption__kg / boilers_mass__kg * (INCOMING_WATER_TEMPERATURE__degC - this_temperature__degC)
                    )

                    mean_temperature_new = min(self.setpoint__degC, mean_temperature_new)
                    new_idx = temp_to_idx(mean_temperature_new)

                    if j_new < dp_value[new_idx, col + 1]:
                        dp_value[new_idx, col + 1] = j_new
                        came_from[new_idx, col + 1] = row
                        boilers_switched_on[new_idx, col + 1] = int(heating__J / one_heater_power__W / step_size__sec)
                        last_15min_energy__J[new_idx, col + 1, :-1] = last_15min_energy__J[new_idx, col + 1, 1:]
                        last_15min_energy__J[new_idx, col + 1, -1] = heating__J


        heating_mean_schedule = np.zeros((self.horizon__steps + 1,), dtype=int)

        path = np.zeros((self.horizon__steps + 1,), dtype=int)
        idx = np.argmin(dp_value[:, -1])
        for col in range(dp_value.shape[1]-1, -1, -1):
            heating_mean_schedule[col] = boilers_switched_on[idx, col]
            path[col] = idx
            idx = came_from[idx, col]

        schedule = self.calc_heat_schedule(heating_mean_schedule, start_temperatures__degC, step_size__sec)

        print("dp control generated")
        return schedule


    def calc_heat_schedule(self,
                           mean_schedule,
                           current_temperatures__degC: NDArray,
                           step_size__sec: int,
                           ):
        """
            Function calculate the heat schedule for microgrid. It receives current temperatures and every timestep
            heats the boiler with the lowest temperature.

            :param current_temperatures__degC:
            :param step_size__sec:
            :return:
        """

        # # we can simulteneously turn on n_simulteneous boilers
        # n_simulteneous = int(self.max_instantaneous_power__W / self.heater_power__W)
        #
        # # inside 15min period we can turn on boilers for this part of period
        # heating_time_part = self.max_15min_power__W / (n_simulteneous * self.heater_power__W)
        # if heating_time_part > 1.:      # no constraints
        #     heating_time_part = 1.
        #
        # # time when n_simulteneous boilers are turned on
        # heating_inside_15min__sec = int(15 * 60 * heating_time_part)
        #
        # # convert time from sec to steps
        # heating_inside_15min__steps = int(heating_inside_15min__sec / step_size__sec)
        # nonheating_inside_15min__steps = int(15 * 60 / step_size__sec) - heating_inside_15min__steps
        #
        # # part of heating time corresponding to all time (inside 15 min interval)
        # heat_to_all_relation = (
        #         heating_inside_15min__steps /
        #         (heating_inside_15min__steps + nonheating_inside_15min__steps)
        # )
        #
        # # calc deltaT = current_temperature - setpoint for every boiler
        # temperature_differences__degC = self.setpoint__degC - current_temperatures__degC
        #
        # # convert deltaT to time needed to warm up (in sec and steps)
        # time_to_setpoint__sec = self.temp_to_time__sec_per_degC * temperature_differences__degC
        # steps_to_setpoint_by_house = np.ceil(time_to_setpoint__sec / step_size__sec).astype(int)

        # calculate schedule to go to setpoint and beyond
        schedule = np.zeros((self.horizon__steps, self.number_of_houses), dtype=bool)

        # ids_and_steps = [[idx, steps_to_setpoint_by_house[idx]] for idx in range(self.number_of_houses)]
        # ids_and_steps.sort(key=lambda pair: -pair[1])
        # steps_to_setpoint = self.horizon__steps


        adding_temperature_in_one_step__degC = (
                self.model[0].heater_power__W * self.model[0].efficiency * step_size__sec /
                SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
                (self.model[0].water_mass__kg)
        )

        ids_and_temps = [(idx, current_temperatures__degC[idx]) for idx in range(self.number_of_houses)]
        temperatures__degC = current_temperatures__degC
        for timestep in range(mean_schedule.shape[0]):
            for j in range(mean_schedule[timestep]):

                min_temp_idx = np.argmin(temperatures__degC)
                min_temp = temperatures__degC[min_temp_idx]

                if min_temp < self.setpoint__degC:
                    temperatures__degC[min_temp_idx] += adding_temperature_in_one_step__degC
                    schedule[timestep, min_temp_idx] = True


        return schedule






        # step = 0
        # while step < self.horizon__steps:
        #     for j in range(n_simulteneous):
        #         idx, steps_needed = ids_and_steps[j]
        #         heat_steps = min(steps_needed, heating_inside_15min__steps)
        #         schedule[step: step + heat_steps, idx] = True
        #         ids_and_steps[j][1] -= heat_steps
        #
        #     step += heating_inside_15min__steps
        #     ids_and_steps.sort(key=lambda pair: -pair[1])
        #     if ids_and_steps[0][1] < 0:
        #         # all temperatures are at setpoint
        #         steps_to_setpoint = min(step, steps_to_setpoint)
        #     step += nonheating_inside_15min__steps
        #
        # return schedule, steps_to_setpoint















if __name__ == "__main__":
    dpc = DynamicProgramingController()
    dpc.generate_control(start_temperatures__degC=40*np.ones((5,)),
                         current_time__sec=3*3600,
                         step_size__sec=60)

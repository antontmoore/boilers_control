import numpy as np
from .controller import Controller
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import SETPOINT__degC
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3
from constants import DEFAULT_TIME_STEP__sec
from constants import DEFAULT_CONTROL_HORIZON__sec
from constants import WATER_DENSITY__kg_per_m3
from constants import CONTROLLER_COEFFICIENTS_MPC
from constants import INCOMING_WATER_TEMPERATURE__degC
from constants import water_consumption__m3_per_sec
from constants import energy_price__usd_per_J
from numpy.typing import NDArray
from math import floor, ceil
from scipy.optimize import minimize
from scipy.optimize import Bounds


class ModelPredictiveController(Controller):
    """
        Model Predictive Controller.
        Use the MPC-strategy to choose the period for heating.
        The variational parameters are start and end time of heating inside the horizon.
        The metrics is sum of:
         - difference between mean temperature inside all boilers and setpoint
         - the coldness of consumed water
         - total cost of energy used to heat up water
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
        self.one_step_one_heater_energy__J = self.heater_power__W * self.model[0].efficiency * DEFAULT_TIME_STEP__sec
        self.j_coefficients = CONTROLLER_COEFFICIENTS_MPC

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

        def x_to_time(x):
            """ Function calculate timestamp from x = part of horizon) """
            return x * self.horizon_time__sec + current_time__sec

        # parameters according to given step_size
        self.one_step_one_heater_energy__J = self.heater_power__W * self.model[0].efficiency * step_size__sec
        self.horizon__steps = int(self.horizon_time__sec / step_size__sec)
        timestamps__sec = np.linspace(0, self.horizon_time__sec, self.horizon__steps + 1) + current_time__sec

        # interpolate external data of energy price and water consumption
        horizon_price__usd_per_J = self.interpolate_to_horizon(energy_price__usd_per_J[:, 0],
                                                               energy_price__usd_per_J[:, 1],
                                                               timestamps__sec) * self.number_of_houses

        horizon_consumption__m3 = self.interpolate_to_horizon(water_consumption__m3_per_sec[:, 0],
                                                              water_consumption__m3_per_sec[:, 1],
                                                              timestamps__sec) * self.number_of_houses

        horizon_consumption__kg = horizon_consumption__m3 * WATER_DENSITY__kg_per_m3

        def calc_functional(x: NDArray) -> float:
            """
                Function to calculate optimization metrics and to be used inside scipy optimization.
                :param x: consists of two values: start and end time of heating in terms of horizon parts

                :return: optimization metrics
            """

            time_start = x_to_time(x[0]) + 1
            time_end = x_to_time(x[1])

            if time_end < time_start:
                # we don't need inverted timestamps
                return 1e9

            # calculate array of mean heating energy during the horizon period
            max_allowable_energy_in_one_step__J = self.get_max_allowable_energy(step_size__sec)
            mask = (time_start <= timestamps__sec) & (timestamps__sec <= time_end)
            heating__J = np.zeros((self.horizon__steps + 1,))
            heating__J[mask] = max_allowable_energy_in_one_step__J

            # calculate the mean temperature trend and metric's parts
            mean_temperature__degC = np.mean(start_temperatures__degC) * np.ones((self.horizon__steps + 1,))
            total_consumption_penalty = 0.
            for j in range(mean_temperature__degC.shape[0] - 1):
                mean_temperature__degC[j + 1] = (
                    mean_temperature__degC[j] +
                    heating__J[j] / SPECIFIC_HEAT_CAPACITY__J_per_kg_degC / self.total_water_mass__kg +
                    horizon_consumption__kg[j] / self.total_water_mass__kg *
                    (INCOMING_WATER_TEMPERATURE__degC - mean_temperature__degC[j])
                )
                if mean_temperature__degC[j + 1] > self.setpoint__degC:
                    mean_temperature__degC[j + 1] = self.setpoint__degC

                total_consumption_penalty += horizon_consumption__kg[j] * \
                    (self.setpoint__degC - mean_temperature__degC[j])

            # calculate other metric's parts
            total_cost__usd = np.sum(horizon_price__usd_per_J[mask] * max_allowable_energy_in_one_step__J)
            total_temp_diff_penalty = np.sum(self.setpoint__degC - mean_temperature__degC)

            # finally calculate wheighted optimization metrics
            opimization_metrics = (
                self.j_coefficients["cost"] * total_cost__usd +
                self.j_coefficients["temp_diff"] * total_temp_diff_penalty +
                self.j_coefficients["consumption"] * total_consumption_penalty
            )

            return opimization_metrics

        # optimization itself
        opt_bounds = Bounds(lb=-0.001, ub=1., keep_feasible=True)
        init_value = self.get_initial_values(int(current_time__sec / 3600), self.horizon__steps)
        opt_result = minimize(fun=calc_functional,
                              x0=init_value,
                              bounds=opt_bounds,
                              method='L-BFGS-B')

        # having the time boundaries we calculate the mean schedule
        mean_schedule = self.create_mean_schedule(opt_result.x[0], opt_result.x[1], step_size__sec)

        # and refined schedule (for every house)
        schedule = self.calc_heat_schedule_from_mean_schedule(mean_schedule=mean_schedule,
                                                              current_temperatures__degC=start_temperatures__degC,
                                                              step_size__sec=step_size__sec)
        return schedule

    def create_mean_schedule(self,
                             part_start: float,
                             part_end: float,
                             step_size__sec: int) -> NDArray:
        """
            Function calculates mean schedule - how many boilers we should switch at every timestep.

            :param part_start:      time when the heating starts (in terms of horizon parts)
            :param part_end:        time when the heating ends (in terms of horizon parts)
            :param step_size__sec:  the size of step in seconds

            :return: mean_schedule  schedule of switching on boilers     [horizon_steps,]
        """

        steps_inside_15min = int(15 * 60 / step_size__sec)
        max_heaters_in_one_step = int(self.max_instantaneous_power__W / self.heater_power__W)

        max_control_actions_in_15min = min(
            int(self.max_15min_power__W * steps_inside_15min / self.heater_power__W),
            max_heaters_in_one_step * steps_inside_15min
        )

        # create 15min schedule with current constraints
        period_15min = np.zeros((int(15 * 60 / step_size__sec),), dtype=int)
        j, control_actions_used = 0, 0
        while j < period_15min.shape[0] and control_actions_used < max_control_actions_in_15min:
            period_15min[j] = max_heaters_in_one_step
            control_actions_used += max_heaters_in_one_step
            j += 1

        # create mean schedule
        mean_schedule = np.zeros((self.horizon__steps + 1,), dtype=int)
        step_start = floor(self.horizon__steps * part_start)
        step_end = ceil(self.horizon__steps * part_end)
        step = max(0, step_start)
        while step < step_end:
            steps_to_fill = min(step_end, step + steps_inside_15min) - step
            mean_schedule[step: step + steps_to_fill] = period_15min[:steps_to_fill]
            step += steps_to_fill

        return mean_schedule

    def get_max_allowable_energy(self, step_size__sec: int) -> float:
        """
            Function calculating max allowable power.
        """
        steps_inside_15min = (15 * 60 / step_size__sec)
        max_heaters_in_one_step = int(self.max_instantaneous_power__W / self.heater_power__W)

        max_control_actions_in_15min = min(
            int(self.max_15min_power__W * steps_inside_15min / self.heater_power__W),
            max_heaters_in_one_step * steps_inside_15min
        )

        max_one_step_energy__J = (
            self.one_step_one_heater_energy__J *
            (max_control_actions_in_15min / steps_inside_15min)
        )

        return max_one_step_energy__J

    @staticmethod
    def get_initial_values(hour: int, horizon__sec: int) -> NDArray:
        """
            Function giving the initial estimation for optimization.
        """
        if hour in [8, 19]:
            t_start = 3600 / horizon__sec
        elif hour == 18:
            t_start = 2 * 3600 / horizon__sec
        else:
            t_start = 0

        if hour < 8:
            t_end = (8 - hour) * 3600 / horizon__sec
        elif hour < 18:
            t_end = (18 - hour) * 3600 / horizon__sec
        else:
            t_end = (24 + 8 - hour) * 3600 / horizon__sec

        t_start = max(0., t_start)
        t_end = min(1., t_end)

        return np.array([t_start, t_end])

    @staticmethod
    def interpolate_to_horizon(x: NDArray, y: NDArray, timestamps__sec: NDArray) -> NDArray:
        """
            Function to interpolate water consumption and energy price for control horizon.

            :param x:                   array of timestamps, corresponding to values
            :param y:                   array of values
            :param timestamps__sec:    array of timestamps, where we need the interpolated values

            :return: array of interpolated values
        """

        x = np.hstack((x, x + 24 * 3600))
        y = np.hstack((y, y))

        return np.interp(timestamps__sec, x, y)

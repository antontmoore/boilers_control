from abc import ABC
import numpy as np
from numpy.typing import NDArray
from simulation.house_model import HouseWithBoiler
from .utils import CircularBuffer
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import SETPOINT__degC
from constants import DEFAULT_TIME_STEP__sec
from constants import DEFAULT_CONTROL_HORIZON__sec
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3


class Controller(ABC):
    """ Abstract class for controller. """

    def __init__(self,
                 number_of_houses: int = 5,
                 horizon_time__sec: int = DEFAULT_CONTROL_HORIZON__sec,
                 setpoint__degC: float = SETPOINT__degC,
                 heater_power__W: float = DEFAULT_HEATER_POWER__W,
                 boiler_capacity__m3: float = DEFAULT_BOILER_CAPACITY__m3,
                 max_instantaneous_power__W: float = MAXIMUM_INSTANTANEOUS_POWER__W,
                 max_15min_power__W: float = MAXIMUM_15MIN_POWER__W,
                 ):
        """
            :param number_of_houses:                number of houses in the grid
            :param horizon_time__sec:               the horizon of modelling while generating control
            :param setpoint__degC:                  setpoint for water temperature in boiler
            :param heater_power__W:                 heater power value
            :param boiler_capacity__m3:             boiler volume capacity in cubic meters
            :param max_instantaneous_power__W:      maximum instantaneous power for all microgrid
            :param max_15min_power__W:              maximum average power during 15 minutes for all microgrid
        """

        self.number_of_houses = number_of_houses
        self.horizon_time__sec = horizon_time__sec
        self.setpoint__degC = setpoint__degC
        self.heater_power__W = heater_power__W
        self.boiler_capacity__m3 = boiler_capacity__m3
        self.max_instantaneous_power__W = max_instantaneous_power__W
        self.max_15min_power__W = max_15min_power__W

        # creating the internal controller model for houses with boiler
        self.model = [HouseWithBoiler(setpoint__degC=setpoint__degC,
                                      heater_power__W=heater_power__W,
                                      boiler_capacity__m3=boiler_capacity__m3,
                                      )
                      for _ in range(number_of_houses)
                      ]

        # water mass in all boilers
        self.total_water_mass__kg = sum([m.water_mass__kg for m in self.model])

        self.last_15min_energy__J = CircularBuffer(int(15 * 60 / DEFAULT_TIME_STEP__sec))

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
        pass

    def calc_heat_schedule_from_mean_schedule(
            self,
            mean_schedule: NDArray,
            current_temperatures__degC: NDArray,
            step_size__sec: int,
    ) -> NDArray:

        """
            Function calculate the heat schedule for all the houses, given the mean_schedule and start temperatures.
            Every timestep it heats the boiler(s) with the lowest temperature.

            :param mean_schedule:               amount of boilers to heat at the step k       [number_of_time_steps,]
            :param current_temperatures__degC:  temperarutures we start with in every boiler  [number_of_houses,]
            :param step_size__sec:              size of time step in seconds
            :return:
        """

        schedule = np.zeros((self.horizon__steps, self.number_of_houses), dtype=bool)

        adding_temperature_in_one_step__degC = (
            self.model[0].heater_power__W * self.model[0].efficiency * step_size__sec /
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
            (self.model[0].water_mass__kg)
        )

        temperatures__degC = current_temperatures__degC
        for timestep in range(mean_schedule.shape[0]):
            for j in range(mean_schedule[timestep]):

                min_temp_idx = np.argmin(temperatures__degC)
                min_temp = temperatures__degC[min_temp_idx]

                if min_temp < self.setpoint__degC:
                    temperatures__degC[min_temp_idx] += adding_temperature_in_one_step__degC
                    schedule[timestep, min_temp_idx] = True

        return schedule

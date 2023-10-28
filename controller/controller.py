from abc import ABC
import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
from house_model import HouseWithBoiler
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import SETPOINT__degC
from constants import DEFAULT_CONTROL_HORIZON__sec
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3
from constants import WATER_DENSITY__kg_per_m3


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

        # water mass in one boiler
        water_mass_in_boiler__kg = self.boiler_capacity__m3 * WATER_DENSITY__kg_per_m3

        # coefficient to calculate time needed to increase temperature in one boiler by one degree
        self.temp_to_time__sec_per_degC = (
                SPECIFIC_HEAT_CAPACITY__J_per_kg_degC * water_mass_in_boiler__kg /
                (self.heater_power__W * self.model[0].efficiency)
        )

    def simulate_schedule(self,
                          start_temperatures__degC: NDArray,
                          schedule: NDArray,
                          step_size__sec: int):
        """
            Function simulate the given schedule, starting with given start_temperatures

            :param start_temperatures__degC:  water temperature in boilers          [number_of_houses]
            :param schedule:                  flags to switch boiler on or not      [number_of_steps, number_of_houses]
            :param step_size__sec:            duration of one time step

            :return:                          simulated temperature trends          [number_of_steps, number_of_houses]
        """

        steps_to_model = schedule.shape[0]

        [self.model[house_idx].set_current_temperature(start_temperatures__degC[house_idx])
            for house_idx in range(self.number_of_houses)]

        temperature_trends__degC = np.zeros((steps_to_model+1, self.number_of_houses))
        temperature_trends__degC[0, :] = start_temperatures__degC
        for step in range(steps_to_model):
            commands = schedule[step, :]
            # commands = np.ones((5,), dtype=bool)

            for house_idx, house in enumerate(self.model):
                house.make_step(0, commands[house_idx], step_size__sec)
                temperature_value = house.get_current_temperature()
                temperature_trends__degC[step + 1, house_idx] = temperature_value

        return temperature_trends__degC

    @staticmethod
    def plot_trends(trends: NDArray):
        """
            Auxilary method for quick plotting the trends. Needed at debugging.

            :param trends:      values for plotting         [number_of_points, number_of_trends]
        """

        for idx in range(trends.shape[1]):
            plt.plot(trends[:, idx])
        plt.show()

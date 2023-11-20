from controller.controller import Controller
from numpy.typing import NDArray
import numpy as np
from .house_model import HouseWithBoiler
from constants import water_consumption__m3_per_sec
from constants import energy_price__usd_per_J
from controller.utils import CircularBuffer
from math import ceil


class SimulationResults:
    """
        Class of simulation results.
        This class is returned by Simulator.simulate_day() and used for plotting inside test environment.
    """
    temperature_trends: NDArray
    power_trend: NDArray
    avg_power_trend: NDArray
    cost_trend: NDArray
    total_costs__usd: float
    total_energy__J: float
    money_saved: float

    def __init__(self, periods):
        self.temperature_trends = np.zeros((periods, 5))
        self.power_trend = np.zeros((periods,))
        self.avg_power_trend = np.zeros((periods,))
        self.cost_trend = np.zeros((periods,))
        self.total_costs__usd = 0.
        self.total_energy__J = 0.


class Simulator:
    """
        Class for quick simulation.
        Recieves controller, create models of houses with boilers and simulate one full day, given starting conditions.
    """
    def __init__(self,
                 number_of_houses: int = 5):
        self.number_of_houses = number_of_houses
        self.real_houses = []

    def simulate_day(self,
                     controller: Controller,
                     setpoint__degC: float,
                     timestep__sec: int,
                     period__sec: int,
                     start_temperatures__degC: NDArray) -> SimulationResults:

        # creating the model of real houses
        self.real_houses = [HouseWithBoiler(setpoint__degC=setpoint__degC)
                            for _ in range(self.number_of_houses)]

        # setting current temperatures
        [self.real_houses[h].set_current_temperature(start_temperatures__degC[h])
         for h in range(self.number_of_houses)]

        periods_to_model = ceil(24 * 3600 / period__sec)
        steps_inside_period = int(period__sec / timestep__sec)
        last_15min_power__W = CircularBuffer(int(15 * 60 / timestep__sec))

        result = SimulationResults(periods_to_model * period__sec)

        current_time__sec = 0
        for period in range(periods_to_model):
            temperatures__degC = np.array([house.get_current_temperature() for house in self.real_houses])

            controller.last_15min_energy__J.put_data(last_15min_power__W.data * timestep__sec)
            schedule = controller.generate_control(
                start_temperatures__degC=temperatures__degC,
                current_time__sec=current_time__sec,
                step_size__sec=timestep__sec,
            )
            for j in range(steps_inside_period):

                index_to_save = period * steps_inside_period + j
                temperatures__degC = np.array([house.get_current_temperature() for house in self.real_houses])

                current_consumption__m3_per_sec = np.interp(current_time__sec,
                                                            water_consumption__m3_per_sec[:, 0],
                                                            water_consumption__m3_per_sec[:, 1])
                current_price__usd_per_J = np.interp(current_time__sec,
                                                     energy_price__usd_per_J[:, 0],
                                                     energy_price__usd_per_J[:, 1])

                for hn in range(self.number_of_houses):
                    power_used__W = self.real_houses[hn].make_step(current_consumption__m3_per_sec,
                                                                   schedule[j, hn],
                                                                   timestep__sec)

                    # save to results
                    result.temperature_trends[index_to_save, hn] = self.real_houses[hn].get_current_temperature()
                    result.power_trend[index_to_save] += power_used__W

                # calculate average
                last_15min_power__W.add(result.power_trend[index_to_save])
                result.avg_power_trend[index_to_save] = last_15min_power__W.current_mean()

                result.cost_trend[index_to_save] = \
                    current_price__usd_per_J * result.power_trend[index_to_save] * timestep__sec
                current_time__sec += timestep__sec

        result.total_costs__usd = np.sum(result.cost_trend)
        result.total_energy__J = np.sum(result.power_trend) * timestep__sec
        return result

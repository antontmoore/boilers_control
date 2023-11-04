from controller.controller import Controller
from numpy.typing import NDArray
import numpy as np
from .house_model import HouseWithBoiler
from constants import water_consumption__m3_per_sec
from constants import energy_price__usd_per_J


class SimulationResults:
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
    def __init__(self,
                 number_of_houses: int = 5):
        self.number_of_houses = number_of_houses
        self.real_houses = []

    def simulate_day(self,
                     controller: Controller,
                     setpoint__degC: float,
                     timestep__sec: int,
                     start_temperatures__degC: NDArray):

        # creating the model of real houses
        self.real_houses = [HouseWithBoiler(setpoint__degC=setpoint__degC)
                            for _ in range(self.number_of_houses)]

        # setting current temperatures
        [self.real_houses[h].set_current_temperature(start_temperatures__degC[h])
         for h in range(self.number_of_houses)]

        periods_to_model = int(24 * 3600 / timestep__sec)
        array_for_avg = np.zeros((int(15 * 60 / timestep__sec),))
        avg_pointer = 0
        result = SimulationResults(periods_to_model)

        current_time__sec = 0
        for period in range(periods_to_model):
            temperatures__degC = np.array([house.get_current_temperature() for house in self.real_houses])
            schedule = controller.generate_control(
                start_temperatures__degC=temperatures__degC,
                current_time__sec=current_time__sec,
                step_size__sec=timestep__sec
            )

            current_consumption__m3_per_sec = np.interp(current_time__sec,
                                                        water_consumption__m3_per_sec[:, 0],
                                                        water_consumption__m3_per_sec[:, 1])
            current_price__usd_per_J = np.interp(current_time__sec,
                                                 energy_price__usd_per_J[:, 0],
                                                 energy_price__usd_per_J[:, 1])

            for hn in range(self.number_of_houses):
                power_used__W = self.real_houses[hn].make_step(current_consumption__m3_per_sec,
                                                               schedule[0, hn],
                                                               timestep__sec)

                # save to results
                result.temperature_trends[period, hn] = self.real_houses[hn].get_current_temperature()
                result.power_trend[period] += power_used__W

            # calculate average
            array_for_avg[avg_pointer] = result.power_trend[period]
            result.avg_power_trend[period] = np.mean(array_for_avg)
            avg_pointer = (avg_pointer + 1) % array_for_avg.shape[0]

            result.cost_trend[period] = current_price__usd_per_J * result.power_trend[period] * timestep__sec
            current_time__sec += timestep__sec

        result.total_costs__usd = np.sum(result.cost_trend)
        result.total_energy__J = np.sum(result.power_trend) * timestep__sec

        return result

import numpy as np
import matplotlib.pyplot as plt
from typing import Union
from numpy.typing import NDArray
from constants import AMBIENT_TEMPERATURE__degC
from constants import INCOMING_WATER_TEMPERATURE__degC
from constants import SPECIFIC_HEAT_CAPACITY__J_per_kg_degC
from constants import WATER_DENSITY__kg_per_m3
from constants import DEFAULT_FULL_HEATING_TIME__sec
from constants import DEFAULT_TIME_STEP__sec
from constants import SETPOINT__degC


class HouseWithBoiler:
    def __init__(self,
                 setpoint__degC: float = SETPOINT__degC,
                 upper_limit__degC: float = 60.,
                 heater_power__W: float = 2_000.,
                 boiler_capacity__m3: float = 80. * 0.001,
                 full_heating_time__sec: float = DEFAULT_FULL_HEATING_TIME__sec,
                 heat_loss_coefficient__W_per_degC: float = 0.,
                 boiler_control_tolerance__degC: float = 0.2
                 ):
        self.setpoint__degC = setpoint__degC
        self.upper_limit__degC = upper_limit__degC
        self.heater_power__W = heater_power__W
        self.boiler_capacity__m3 = boiler_capacity__m3
        self.water_mass__kg = WATER_DENSITY__kg_per_m3 * boiler_capacity__m3
        self.heat_loss_coefficient__W_per_degC = heat_loss_coefficient__W_per_degC
        self.boiler_control_tolerance__degC = boiler_control_tolerance__degC

        # Estimating the boiler efficiency
        full_heating_energy__J = (
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC * self.water_mass__kg *
            (upper_limit__degC - INCOMING_WATER_TEMPERATURE__degC)
        )
        energy_consumed_full_heat__J = heater_power__W * full_heating_time__sec

        self.efficiency = full_heating_energy__J / energy_consumed_full_heat__J

        self.current_temperature__C = AMBIENT_TEMPERATURE__degC
        self.time_boiling = 0

    def make_step(self,
                  out_consumption__m3_per_sec: Union[float, NDArray] = 0.,
                  boiler_is_on: bool = False,
                  time_step__sec: int = DEFAULT_TIME_STEP__sec):
        """
            Function to make one step in time and calculate new current temperature.

            :param out_consumption__m3_per_sec:   volume flow of water used
            :param boiler_is_on:                  flag, is the boiler on
            :return:                              None
        """

        power_used_in_step__W = 0
        heat_loss_by_cooling__J = (
            self.heat_loss_coefficient__W_per_degC * time_step__sec *
            (self.current_temperature__C - AMBIENT_TEMPERATURE__degC)
        )

        heat_loss_by_flow__J = (
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC *
            WATER_DENSITY__kg_per_m3 * out_consumption__m3_per_sec * time_step__sec *
            (self.current_temperature__C - INCOMING_WATER_TEMPERATURE__degC)
        )

        heat_added_by_boiler__J = 0.
        if boiler_is_on:
            if self.current_temperature__C < self.setpoint__degC - self.boiler_control_tolerance__degC:
                heat_added_by_boiler__J = self.heater_power__W * time_step__sec * self.efficiency
                power_used_in_step__W = self.heater_power__W
                self.time_boiling += 1

        self.current_temperature__C += (
            (heat_added_by_boiler__J - heat_loss_by_cooling__J - heat_loss_by_flow__J) /
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
            self.water_mass__kg
        )

        return power_used_in_step__W

    def simulate_day(self,
                     timetable,
                     daily_consumption__m3_per_sec):
        """
            Function for one day simulation, calculating the temperature inside boiler step-by-step.

            :param   timetable:                         control action for every time step during the day
            :param   daily_consumption__m3_per_sec:     water consumption for every time step during the day
            :return: temperature_ts                     time series of temperature calculated with given action
        """

        temperature_ts = []
        current_time = 0
        for j in range(daily_consumption__m3_per_sec.shape[0]):
            boil = bool(timetable[j])
            water_consumption__m3_per_sec = daily_consumption__m3_per_sec[j]
            self.make_step(water_consumption__m3_per_sec, boil, DEFAULT_TIME_STEP__sec)
            temperature_ts.append(self.current_temperature__C)
            current_time += DEFAULT_TIME_STEP__sec

        return temperature_ts

    def set_current_temperature(self,
                                temperature_value__degC):
        self.current_temperature__C = temperature_value__degC

    def get_current_temperature(self):
        return self.current_temperature__C


def calc_avg_consumption_and_price(timestep__sec):

    number_of_timestamps = int(86400 / timestep__sec + 1)
    avg_consumption__m3_per_sec = np.zeros((number_of_timestamps,))
    current_time_sec = 0
    idx = 0
    while current_time_sec <= 86400:
        if (
                7 * 3600 <= current_time_sec <= 8 * 3600 or
                17 * 3600 <= current_time_sec <= 20 * 3600
        ):
            avg_consumption__m3_per_sec[idx] = 0.0115 * 0.001

        current_time_sec += timestep__sec
        idx += 1

    return avg_consumption__m3_per_sec


def generate_timetables(timestep__sec):
    number_of_timestamps = int(86400 / timestep__sec + 1)
    timetable = np.zeros((number_of_timestamps,))
    current_time_sec = 0
    idx = 0
    while current_time_sec <= 86400:
        if (
                4 * 3600 <= current_time_sec <= 9 * 3600 or
                15 * 3600 <= current_time_sec <= 20.1 * 3600
        ):
            timetable[idx] = 1
        current_time_sec += timestep__sec
        idx += 1

    return timetable


avg_consumption__m3_per_sec = calc_avg_consumption_and_price(DEFAULT_TIME_STEP__sec)
timetable = generate_timetables(DEFAULT_TIME_STEP__sec)


if __name__ == "__main__":
    hwb = HouseWithBoiler()
    temperature_ts = hwb.simulate_day(timetable,
                                      avg_consumption__m3_per_sec)

    fig, ax = plt.subplots()
    x_plot = np.linspace(0, 24., len(temperature_ts))

    x_timetable = x_plot[timetable == 1]
    plt.plot(x_timetable, 50 * np.ones_like(x_timetable), 'oc')

    x_consumption = x_plot[avg_consumption__m3_per_sec > 0]
    plt.plot(x_consumption, 45 * np.ones_like(x_consumption), 'og')

    plt.plot(x_plot, temperature_ts, '-')
    plt.grid(True)
    plt.show()

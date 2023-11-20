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
    """
        Class for simulation model of house with boiler.
    """
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

    def make_step(self,
                  out_consumption__m3_per_sec: Union[float, NDArray] = 0.,
                  boiler_is_on: bool = False,
                  time_step__sec: int = DEFAULT_TIME_STEP__sec) -> float:
        """
            Function to make one step in time and calculate new current temperature.

            :param    out_consumption__m3_per_sec     volume flow of water used
            :param    boiler_is_on                    flag, is the boiler on
            :param    time_step__sec                  timestep durztion in sec

            :return:  power_used_in_step__W           power used during the simulated step
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

        self.current_temperature__C += (
            (heat_added_by_boiler__J - heat_loss_by_cooling__J - heat_loss_by_flow__J) /
            SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
            self.water_mass__kg
        )

        return power_used_in_step__W

    def set_current_temperature(self,
                                temperature_value__degC: float):
        self.current_temperature__C = temperature_value__degC

    def get_current_temperature(self) -> float:
        return self.current_temperature__C

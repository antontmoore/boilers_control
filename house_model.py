import numpy as np
import matplotlib.pyplot as plt

MAXIMUM_INSTANTANEOUS_POWER__W = 4_000.
MAXIMUM_15MIN_POWER__W = 900.

AMBIENT_TEMPERATURE__degC = 20.
INCOMING_WATER_TEMPERATURE__degC = 10.
SPECIFIC_HEAT_CAPACITY__J_per_kg_degC = 4_184.
WATER_DENSITY__kg_per_m3 = 1000.
TIME_STEP__sec = 60


class HouseWithBoiler:
    def __init__(self,
                 setpoint__degC: float = 55.,
                 upper_limit__degC: float = 60.,
                 heater_power__W: float = 2_000.,
                 boiler_capacity__m3: float = 80. * 0.001,
                 full_heating_time__sec: float = 2 * 3600 + 21 * 60,
                 heat_loss_coefficient__W_per_degC: float = 5.,
                 boiler_control_tolerance__degC: float = 0.5
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
                  out_consumption__m3_per_sec: float = 0.,
                  boiler_is_on: bool = False):
        """
            Function to make one step in time and calculate new current temperature.

            :param out_consumption__m3_per_sec:   volume flow of water used
            :param boiler_is_on:                  flag, is the boiler on
            :return:                              None
        """

        heat_loss_by_cooling__J = (
                self.heat_loss_coefficient__W_per_degC * TIME_STEP__sec *
                (self.current_temperature__C - AMBIENT_TEMPERATURE__degC)
        )

        heat_loss_by_flow__J = (
                SPECIFIC_HEAT_CAPACITY__J_per_kg_degC *
                WATER_DENSITY__kg_per_m3 * out_consumption__m3_per_sec * TIME_STEP__sec *
                (self.current_temperature__C - INCOMING_WATER_TEMPERATURE__degC)
        )

        heat_added_by_boiler__J = 0.
        if boiler_is_on:
            if self.current_temperature__C < self.setpoint__degC - self.boiler_control_tolerance__degC:
                heat_added_by_boiler__J = self.heater_power__W * TIME_STEP__sec * self.efficiency
                self.time_boiling += 1

        self.current_temperature__C += (
                (heat_added_by_boiler__J - heat_loss_by_cooling__J - heat_loss_by_flow__J) /
                SPECIFIC_HEAT_CAPACITY__J_per_kg_degC /
                self.water_mass__kg
        )

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
            self.make_step(water_consumption__m3_per_sec, boil)
            temperature_ts.append(self.current_temperature__C)
            current_time += TIME_STEP__sec

        return temperature_ts


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
                15 * 3600 <= current_time_sec <= 20 * 3600
        ):
            timetable[idx] = 1
        current_time_sec += timestep__sec
        idx += 1

    return timetable


avg_consumption__m3_per_sec = calc_avg_consumption_and_price(TIME_STEP__sec)
timetable = generate_timetables(TIME_STEP__sec)


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

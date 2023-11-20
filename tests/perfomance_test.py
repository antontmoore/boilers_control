from controller.always_on import AlwaysOnController
from controller.naiive_controller import NaiiveController
from controller.dynamic_programing import DynamicProgramingController
from controller.model_predictive_controller import ModelPredictiveController
from simulation.simulator import Simulator
from constants import SETPOINT__degC
import numpy as np
from time import time as time_now
import matplotlib.pyplot as plt

plt.style.use('dark_background')


def perfomance_test():
    """
        This is test co compare perfomance (average time to generate control)
        and efficiency (total cost in USD) for all controllers.
    """
    controllers = {
        "always_on": AlwaysOnController(),
        "naiive   ": NaiiveController(),
        "dp       ": DynamicProgramingController(),
        "mpc      ": ModelPredictiveController()
    }

    calc_time_by_controller = {controller_name: [] for controller_name in controllers.keys()}
    cost_by_controller = {controller_name: [] for controller_name in controllers.keys()}

    start_temperature_array = [15., 20., 25., 30., 35., 40., 45.]
    days_to_simulate = 1

    for start_temperature in start_temperature_array:
        print(f"start_temperature = {start_temperature}")
        start_temperatures__degC = start_temperature * np.ones((5,))
        simulator = Simulator()

        # mark the time
        for controller_name in controllers.keys():
            start_time = time_now()
            for day in range(days_to_simulate):
                result = simulator.simulate_day(controllers[controller_name],
                                                setpoint__degC=SETPOINT__degC,
                                                timestep__sec=60,
                                                period__sec=3600,
                                                start_temperatures__degC=start_temperatures__degC)
            eval_time = time_now() - start_time

            calc_time_by_controller[controller_name].append(eval_time / days_to_simulate / 24)

        # calculate once again to receive costs
        for controller_name in controllers.keys():
            result = simulator.simulate_day(controllers[controller_name],
                                            setpoint__degC=SETPOINT__degC,
                                            timestep__sec=60,
                                            period__sec=3600,
                                            start_temperatures__degC=start_temperatures__degC)
            cost_by_controller[controller_name].append(result.total_costs__usd)

    # plot graph and print table
    print("controller	: avg time, sec	        : total_cost")
    print("-------------------------------------------------")
    for controller_name in controllers.keys():
        plt.plot(calc_time_by_controller[controller_name],
                 cost_by_controller[controller_name],
                 'o')

        meanx = np.mean(calc_time_by_controller[controller_name])
        meany = np.mean(cost_by_controller[controller_name])
        plt.text(meanx + 0.002, meany + 2, controller_name, color='#BBBBBB',
                 horizontalalignment='center',
                 verticalalignment='center',)

        plt.grid(True, color='#555555')
        plt.xlabel("Time of control generation, sec")
        plt.ylabel("Money spent in one day, usd")
        print(f"{controller_name}\t: {meanx}\t: {meany}")

    plt.show()


if __name__ == "__main__":
    perfomance_test()

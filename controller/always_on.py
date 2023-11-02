import numpy as np
from .controller import Controller
from constants import MAXIMUM_INSTANTANEOUS_POWER__W
from constants import MAXIMUM_15MIN_POWER__W
from constants import SETPOINT__degC
from constants import DEFAULT_HEATER_POWER__W
from constants import DEFAULT_BOILER_CAPACITY__m3
from constants import DEFAULT_TIME_STEP__sec
from numpy.typing import NDArray


class AlwaysOnController(Controller):
    """
        AlwaysOn controller.
        The dumbiest controller that always switches on all the boilers.
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
                         step_size__sec: int
                         ) -> NDArray:
        """
            Function generating control for the future time steps, with all boilers switched on.

            :param   step_size__sec:              step size for control generation

            :return: control                      control actions for all houses       [horizon_steps, number_of_houses]
        """

        self.horizon__steps = int(self.horizon_time__sec / step_size__sec)
        return np.ones((self.horizon__steps, self.number_of_houses), dtype=bool)

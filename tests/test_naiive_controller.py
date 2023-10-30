import numpy as np
from controller.naiive_controller import NaiiveController
import pytest

test_data = [
    (24 * 3600, 22 * 3600, 60, np.array([4.1, 42., 46.5, 44.8, 52.2]), np.array([153, 39, 26, 31, 9])),
    (2 * 3600,   8 * 3600, 60, np.array([41., 42., 43., 44., 45.]),    np.array([18, 12,  9,  6,  3])),
    (10 * 3600,  6 * 3600, 30, np.array([54.4, 55., 55., 55., 55.]),   np.array([4, 0, 0, 0, 0]))
]


@pytest.mark.parametrize('horizon_time__sec,current_time__sec,step_size__sec,start_temperatures__degC,schedule_sum',
                         test_data)
def test_naiive_controller(
        horizon_time__sec,
        current_time__sec,
        step_size__sec,
        start_temperatures__degC,
        schedule_sum
):

    nc = NaiiveController(horizon_time__sec=horizon_time__sec)
    schedule = nc.generate_control(
        start_temperatures__degC=start_temperatures__degC,
        current_time__sec=current_time__sec,
        step_size__sec=step_size__sec
    )

    # assert the shape of generated control actions' schedule
    assert schedule.shape == (int(horizon_time__sec / step_size__sec), 5)

    # assert the values of generated control
    assert np.all(np.sum(schedule, axis=0) == schedule_sum)

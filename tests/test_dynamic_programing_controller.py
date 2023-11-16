from controller.dynamic_programing import DynamicProgramingController
import numpy as np
import pytest

test_data = [
    (8 * 3600, 22 * 3600, 60,  np.array([47., 42., 46., 44., 52.]),    np.array([19, 33, 22, 30,  0])),
    (12 * 3600, 8 * 3600, 60,  np.array([41., 42., 43., 44., 45.]),    np.array([49, 47, 43, 39, 36])),
    (14 * 3600, 6 * 3600, 300, np.array([54., 55., 55., 55., 55.]),   np.array([18, 18, 18, 18, 18]))
]

@pytest.mark.parametrize('horizon_time__sec,current_time__sec,step_size__sec,start_temperatures__degC,schedule_sum',
                         test_data)
def test_dynamic_programming_controller(
        horizon_time__sec,
        current_time__sec,
        step_size__sec,
        start_temperatures__degC,
        schedule_sum
):

    dpc = DynamicProgramingController(horizon_time__sec=horizon_time__sec)
    schedule = dpc.generate_control(
        start_temperatures__degC=start_temperatures__degC,
        current_time__sec=current_time__sec,
        step_size__sec=step_size__sec
    )

    # assert the shape of generated control actions' schedule
    assert schedule.shape == (int(horizon_time__sec / step_size__sec), 5)

    # assert the values of generated control
    assert np.all(np.sum(schedule, axis=0) == schedule_sum)

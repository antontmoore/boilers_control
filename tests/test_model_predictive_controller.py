from controller.model_predictive_controller import ModelPredictiveController
import numpy as np
import pytest

test_data = [
    (8 * 3600, 22 * 3600, 60,  np.array([47., 42., 46., 44., 52.]),    np.array([25, 37, 28, 34, 10])),
    (12 * 3600, 9 * 3600, 60,  np.array([41., 42., 43., 44., 45.]),    np.array([41, 40, 37, 34, 31])),
    (14 * 3600, 6 * 3600, 300, np.array([54., 55., 55., 55., 55.]),   np.array([1, 0, 0, 0, 0]))
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

    mpc = ModelPredictiveController(horizon_time__sec=horizon_time__sec)
    schedule = mpc.generate_control(
        start_temperatures__degC=start_temperatures__degC,
        current_time__sec=current_time__sec,
        step_size__sec=step_size__sec
    )

    # assert the shape of generated control actions' schedule
    assert schedule.shape == (int(horizon_time__sec / step_size__sec), 5)

    # assert the values of generated control
    assert np.all(np.sum(schedule, axis=0) == schedule_sum)

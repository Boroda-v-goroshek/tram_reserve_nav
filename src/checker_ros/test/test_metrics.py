import math

from hackathon_solution_checker.metrics import ErrorAccumulator


def test_error_accumulator_calculates_rmse_and_maximum():
    metric = ErrorAccumulator()

    assert metric.add(-3.0)
    assert metric.add(4.0)

    assert metric.count == 2
    assert metric.rmse == math.sqrt(12.5)
    assert metric.maximum_error == 4.0


def test_error_accumulator_ignores_non_finite_values():
    metric = ErrorAccumulator()

    assert not metric.add(math.nan)
    assert not metric.add(math.inf)
    assert metric.count == 0
    assert math.isnan(metric.rmse)

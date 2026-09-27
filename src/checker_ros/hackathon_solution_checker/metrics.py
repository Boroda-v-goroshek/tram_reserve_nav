#!/usr/bin/env python3
"""Compare position and velocity solution topics with reference odometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence

import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tram_vehicle_msgs.msg import VelocitySensor


@dataclass
class ErrorAccumulator:
    """Accumulate RMSE and maximum absolute error without storing samples."""

    count: int = 0
    squared_error_sum: float = 0.0
    maximum_error: float = 0.0

    def add(self, error: float) -> bool:
        error = abs(float(error))
        if not math.isfinite(error):
            return False
        self.count += 1
        self.squared_error_sum += error * error
        self.maximum_error = max(self.maximum_error, error)
        return True

    @property
    def rmse(self) -> float:
        if not self.count:
            return math.nan
        return math.sqrt(self.squared_error_sum / self.count)


def get_numeric_field(message: Any, field_path: str) -> float:
    """Read a dot-separated numeric field from a ROS message."""
    value = message
    for part in field_path.split("."):
        if not hasattr(value, part):
            raise AttributeError(
                f"Message {type(message).__name__} has no field path '{field_path}'"
            )
        value = getattr(value, part)
    return float(value)


def position_errors(reference: Odometry, solution: Odometry) -> tuple[float, ...]:
    """Return absolute XYZ errors followed by their Euclidean norm."""
    ref = reference.pose.pose.position
    result = solution.pose.pose.position
    components = (
        abs(result.x - ref.x),
        abs(result.y - ref.y),
        abs(result.z - ref.z),
    )
    return (*components, math.sqrt(sum(value * value for value in components)))


class MetricsNode(Node):
    """Collect metrics for the two result streams independently."""

    def __init__(self) -> None:
        super().__init__("hackathon_solution_checker")

        self.declare_parameter("reference_topic", "/localization/kinematic_state")
        self.declare_parameter("velocity_topic", "/result/velocity")
        self.declare_parameter("position_topic", "/result/position")
        self.declare_parameter("reference_velocity_field", "twist.twist.linear.x")
        self.declare_parameter("result_velocity_field", "velocity")
        self.declare_parameter("sync_tolerance_sec", 0.05)
        self.declare_parameter("sync_queue_size", 100)
        self.declare_parameter("report_period_sec", 5.0)

        reference_topic = self._parameter("reference_topic", str)
        velocity_topic = self._parameter("velocity_topic", str)
        position_topic = self._parameter("position_topic", str)
        self.reference_velocity_field = self._parameter(
            "reference_velocity_field", str
        )
        self.result_velocity_field = self._parameter("result_velocity_field", str)
        tolerance = self._parameter("sync_tolerance_sec", float)
        queue_size = self._parameter("sync_queue_size", int)
        report_period = self._parameter("report_period_sec", float)

        if tolerance < 0.0:
            raise ValueError("sync_tolerance_sec must be non-negative")
        if queue_size < 1:
            raise ValueError("sync_queue_size must be positive")

        self.velocity_metric = ErrorAccumulator()
        self.position_metrics = {
            name: ErrorAccumulator() for name in ("x", "y", "z", "distance")
        }
        self._field_error_reported = False

        # Separate reference subscribers keep the two evaluations independent:
        # missing velocity messages cannot consume or delay position samples.
        self._velocity_reference = message_filters.Subscriber(
            self, Odometry, reference_topic, qos_profile=qos_profile_sensor_data
        )
        self._velocity_result = message_filters.Subscriber(
            self, VelocitySensor, velocity_topic, qos_profile=qos_profile_sensor_data
        )
        self._position_reference = message_filters.Subscriber(
            self, Odometry, reference_topic, qos_profile=qos_profile_sensor_data
        )
        self._position_result = message_filters.Subscriber(
            self, Odometry, position_topic, qos_profile=qos_profile_sensor_data
        )

        self._velocity_sync = message_filters.ApproximateTimeSynchronizer(
            [self._velocity_reference, self._velocity_result],
            queue_size,
            tolerance,
        )
        self._velocity_sync.registerCallback(self._on_velocity_pair)
        self._position_sync = message_filters.ApproximateTimeSynchronizer(
            [self._position_reference, self._position_result],
            queue_size,
            tolerance,
        )
        self._position_sync.registerCallback(self._on_position_pair)

        if report_period > 0.0:
            self._report_timer = self.create_timer(report_period, self.report)

        self.get_logger().info(
            "Comparing %s with velocity=%s and position=%s (sync tolerance %.3f s)"
            % (reference_topic, velocity_topic, position_topic, tolerance)
        )

    def _parameter(self, name: str, expected_type: type) -> Any:
        value = self.get_parameter(name).value
        if not isinstance(value, expected_type):
            raise TypeError(f"Parameter '{name}' must be {expected_type.__name__}")
        return value

    def _on_velocity_pair(
        self, reference: Odometry, solution: VelocitySensor
    ) -> None:
        try:
            reference_velocity = get_numeric_field(
                reference, self.reference_velocity_field
            )
            result_velocity = get_numeric_field(solution, self.result_velocity_field)
        except (AttributeError, TypeError, ValueError) as error:
            if not self._field_error_reported:
                self.get_logger().error(str(error))
                self._field_error_reported = True
            return

        self.velocity_metric.add(result_velocity - reference_velocity)

    def _on_position_pair(self, reference: Odometry, solution: Odometry) -> None:
        errors = position_errors(reference, solution)
        for accumulator, error in zip(self.position_metrics.values(), errors):
            accumulator.add(error)

    @staticmethod
    def _format(metrics: Iterable[tuple[str, ErrorAccumulator]]) -> str:
        return ", ".join(
            f"{name}: RMSE={metric.rmse:.6f}, max={metric.maximum_error:.6f}, "
            f"n={metric.count}"
            for name, metric in metrics
        )

    def report(self) -> None:
        """Write the current independent results to the ROS log."""
        velocity = self._format((("velocity", self.velocity_metric),))
        position = self._format(self.position_metrics.items())
        self.get_logger().info(f"Velocity metrics [m/s]: {velocity}")
        self.get_logger().info(f"Position metrics [m]: {position}")


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MetricsNode | None = None
    try:
        node = MetricsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.report()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Demo actions exposed via the x402 REST integration.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String


_GAIT_MANAGER: Optional[GaitManager] = None
_MOTION_MANAGER: Optional[MotionManager] = None
_SHOOT_PUBLISHER: Optional[rospy.Publisher] = None


def _get_gait_manager() -> GaitManager:
    global _GAIT_MANAGER  # noqa: PLW0603
    if _GAIT_MANAGER is None:
        _GAIT_MANAGER = GaitManager()
    return _GAIT_MANAGER


def _get_motion_manager() -> MotionManager:
    global _MOTION_MANAGER  # noqa: PLW0603
    if _MOTION_MANAGER is None:
        _MOTION_MANAGER = MotionManager()
    return _MOTION_MANAGER


def _get_shoot_publisher() -> rospy.Publisher:
    global _SHOOT_PUBLISHER  # noqa: PLW0603
    if _SHOOT_PUBLISHER is None:
        _SHOOT_PUBLISHER = rospy.Publisher("firecontroll", String, queue_size=1, latch=True)
        rospy.sleep(0.05)
    return _SHOOT_PUBLISHER


@dataclass
class Coordinates:
    lat: float
    lng: float

    def as_dict(self) -> Dict[str, float]:
        return {"lat": self.lat, "lng": self.lng}


def _parse_coordinates(payload: Dict[str, Any], key: str) -> Coordinates:
    if key not in payload:
        raise ValueError(f"'{key}' is required")

    value = payload[key]

    if isinstance(value, dict):
        lat = value.get("lat")
        lng = value.get("lng")
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        lat, lng = value
    else:
        raise ValueError(
            f"'{key}' must be an object with 'lat'/'lng' keys or a [lat, lng] list"
        )

    try:
        return Coordinates(lat=float(lat), lng=float(lng))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"'{key}' contains invalid latitude/longitude values") from exc


def buy_cola_demo(*, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Process a demo cola purchase request by walking the robot straight for 5 seconds.
    """

    payload: Dict[str, Any] = body or {}

    if "quantity" not in payload and "cans" not in payload:
        raise ValueError("'quantity' field is required")

    try:
        quantity = int(payload.get("quantity", payload.get("cans", 0)))
    except (TypeError, ValueError) as exc:
        raise ValueError("'quantity' must be an integer") from exc

    if quantity <= 0:
        raise ValueError("'quantity' must be greater than zero")

    buyer = _parse_coordinates(payload, "buyer")
    seller = _parse_coordinates(payload, "seller")

    gait_manager = _get_gait_manager()

    rospy.loginfo(
        "Executing cola delivery demo: quantity=%s buyer=%s seller=%s",
        quantity,
        buyer.as_dict(),
        seller.as_dict(),
    )

    gait_manager.enable()
    gait_manager.move(step_velocity=2, x_amplitude=0.02, y_amplitude=0.0, rotation_angle=0.0)

    try:
        rospy.sleep(5.0)
    finally:
        gait_manager.stop()

    return {
        "order": {
            "item": "cola_can",
            "quantity": quantity,
        },
        "buyer": buyer.as_dict(),
        "seller": seller.as_dict(),
        "movement": {
            "type": "forward",
            "durationSec": 5,
            "velocityPreset": 2,
            "stepAmplitude": 0.02,
        },
        "status": "completed",
    }


def move_demo(*, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Execute a predefined action group using the MotionManager.
    """

    payload = body or {}
    action_name = payload.get("demo_name") or "wave"

    motion_manager = _get_motion_manager()
    rospy.loginfo("Executing motion demo action=%s", action_name)
    motion_manager.run_action(action_name)

    return {
        "status": "completed",
        "action": action_name,
    }


def shoot_demo(*, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Publish a fire command to the firecontroll topic.
    """

    publisher = _get_shoot_publisher()
    publisher.publish(String(data="fire"))
    rospy.loginfo("Published shoot demo command to firecontroll topic.")

    return {
        "status": "fired",
        "topic": "firecontroll",
        "payload": "fire",
    }


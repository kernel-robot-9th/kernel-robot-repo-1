from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@dataclass
class BiOmxArmConfig:
    """Plain (non-registered) per-arm config so CLI nesting works like bi_so_follower."""

    port: str
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = False


@RobotConfig.register_subclass("bi_omx_follower")
@dataclass
class BiOmxFollowerConfig(RobotConfig):
    left_arm_config: BiOmxArmConfig
    right_arm_config: BiOmxArmConfig

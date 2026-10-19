from dataclasses import dataclass

from ..config import TeleoperatorConfig


@dataclass
class BiOmxLeaderArmConfig:
    """Plain (non-registered) per-arm config so CLI nesting works like bi_so_leader."""

    port: str
    gripper_open_pos: float = 60.0


@TeleoperatorConfig.register_subclass("bi_omx_leader")
@dataclass
class BiOmxLeaderConfig(TeleoperatorConfig):
    left_arm_config: BiOmxLeaderArmConfig
    right_arm_config: BiOmxLeaderArmConfig

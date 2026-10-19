import logging
from functools import cached_property

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..omx_leader import OmxLeader, OmxLeaderConfig
from ..teleoperator import Teleoperator
from .config_bi_omx_leader import BiOmxLeaderConfig

logger = logging.getLogger(__name__)


class BiOmxLeader(Teleoperator):
    """Bimanual OMX leader arms (ROBOTIS OpenMANIPULATOR-X)."""

    config_class = BiOmxLeaderConfig
    name = "bi_omx_leader"

    def __init__(self, config: BiOmxLeaderConfig):
        super().__init__(config)
        self.config = config

        def _arm_cfg(side: str, arm_cfg) -> OmxLeaderConfig:
            return OmxLeaderConfig(
                id=f"{config.id}_{side}" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=arm_cfg.port,
                gripper_open_pos=arm_cfg.gripper_open_pos,
            )

        self.left_arm = OmxLeader(_arm_cfg("left", config.left_arm_config))
        self.right_arm = OmxLeader(_arm_cfg("right", config.right_arm_config))

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"left_{k}": v for k, v in self.left_arm.action_features.items()},
            **{f"right_{k}": v for k, v in self.right_arm.action_features.items()},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        action_dict = {}
        action_dict.update({f"left_{k}": v for k, v in self.left_arm.get_action().items()})
        action_dict.update({f"right_{k}": v for k, v in self.right_arm.get_action().items()})
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    @check_if_not_connected
    def disconnect(self) -> None:
        self.left_arm.disconnect()
        self.right_arm.disconnect()

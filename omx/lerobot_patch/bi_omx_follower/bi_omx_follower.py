import logging
from functools import cached_property

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..omx_follower import OmxFollower, OmxFollowerConfig
from ..robot import Robot
from .config_bi_omx_follower import BiOmxFollowerConfig
from .sim_publisher import SimPublisher

logger = logging.getLogger(__name__)


class BiOmxFollower(Robot):
    """Bimanual OMX follower arms (ROBOTIS OpenMANIPULATOR-X)."""

    config_class = BiOmxFollowerConfig
    name = "bi_omx_follower"

    def __init__(self, config: BiOmxFollowerConfig):
        super().__init__(config)
        self.config = config

        def _arm_cfg(side: str, arm_cfg) -> OmxFollowerConfig:
            return OmxFollowerConfig(
                id=f"{config.id}_{side}" if config.id else None,
                calibration_dir=config.calibration_dir,
                port=arm_cfg.port,
                disable_torque_on_disconnect=arm_cfg.disable_torque_on_disconnect,
                max_relative_target=arm_cfg.max_relative_target,
                use_degrees=arm_cfg.use_degrees,
                cameras=arm_cfg.cameras,
            )

        self.left_arm = OmxFollower(_arm_cfg("left", config.left_arm_config))
        self.right_arm = OmxFollower(_arm_cfg("right", config.right_arm_config))
        self._sim_pub = SimPublisher.from_env()  # OMX_SIM_UDP 가 없으면 아무 동작 안 함

        # For compatibility with code that expects `robot.cameras`
        # Extra cameras not attached to an arm (e.g. head camera)
        self.head_cameras = make_cameras_from_configs(config.cameras)
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras, **self.head_cameras}

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"left_{k}": v for k, v in self.left_arm._motors_ft.items()},
            **{f"right_{k}": v for k, v in self.right_arm._motors_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            **{f"left_{k}": v for k, v in self.left_arm._cameras_ft.items()},
            **{f"right_{k}": v for k, v in self.right_arm._cameras_ft.items()},
            **{name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()},
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.left_arm.is_connected
            and self.right_arm.is_connected
            and all(cam.is_connected for cam in self.head_cameras.values())
        )

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)
        for cam in self.head_cameras.values():
            cam.connect()

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
    def get_observation(self) -> RobotObservation:
        obs_dict = {}
        obs_dict.update({f"left_{k}": v for k, v in self.left_arm.get_observation().items()})
        obs_dict.update({f"right_{k}": v for k, v in self.right_arm.get_observation().items()})
        for name, cam in self.head_cameras.items():
            obs_dict[name] = cam.read_latest() if hasattr(cam, "read_latest") else cam.async_read()
        self._sim_pub.publish(obs_dict)
        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {k.removeprefix("left_"): v for k, v in action.items() if k.startswith("left_")}
        right_action = {k.removeprefix("right_"): v for k, v in action.items() if k.startswith("right_")}

        sent_left = self.left_arm.send_action(left_action)
        sent_right = self.right_arm.send_action(right_action)

        return {
            **{f"left_{k}": v for k, v in sent_left.items()},
            **{f"right_{k}": v for k, v in sent_right.items()},
        }

    @check_if_not_connected
    def disconnect(self):
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        for cam in self.head_cameras.values():
            cam.disconnect()

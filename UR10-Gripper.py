from omni.isaac.core.objects.cuboid import DynamicCuboid
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.controllers import BaseGripperController
from omni.isaac.universal_robots.ur10 import UR10
from omni.isaac.universal_robots.controllers import RMPFlowController
from typing import List, Optional
from omni.isaac.contact_sensor import _contact_sensor
import numpy as np
import carb

class RobotiqGripperController(BaseGripperController):
    """ Define Gripper Controller
    Position Definition :
        Open position
        Close position 
    """
    def __init__(self, gripper_dof_indices: List[int], deltas: Optional[np.ndarray] = None) -> None:
        self._grippers_dof_indices = gripper_dof_indices
        super().__init__(name="robotiq_gripper_controller")
        if deltas is None:
            deltas = np.array([np.pi/90,np.pi/90,-np.pi/90,-np.pi/90,np.pi/90,-np.pi/90])
        self._deltas = deltas
        return
    
    def open(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        current_gripper_position_1 = current_joint_positions[self._grippers_dof_indices[0]]
        current_gripper_position_2 = current_joint_positions[self._grippers_dof_indices[1]]
        current_gripper_position_3 = current_joint_positions[self._grippers_dof_indices[2]]
        current_gripper_position_4 = current_joint_positions[self._grippers_dof_indices[3]]
        current_gripper_position_5 = current_joint_positions[self._grippers_dof_indices[4]]
        current_gripper_position_6 = current_joint_positions[self._grippers_dof_indices[5]]
        target_joint_positions = [None] * current_joint_positions.shape[0]
        target_joint_positions[self._grippers_dof_indices[0]] = current_gripper_position_1 - self._deltas[0]
        target_joint_positions[self._grippers_dof_indices[1]] = current_gripper_position_2 - self._deltas[1]
        target_joint_positions[self._grippers_dof_indices[2]] = current_gripper_position_3 - self._deltas[2]
        target_joint_positions[self._grippers_dof_indices[3]] = current_gripper_position_4 - self._deltas[3]
        target_joint_positions[self._grippers_dof_indices[4]] = current_gripper_position_5 - self._deltas[4]
        target_joint_positions[self._grippers_dof_indices[5]] = current_gripper_position_6 - self._deltas[5]
        return ArticulationAction(joint_positions=target_joint_positions)

    def close(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        current_gripper_position_1 = current_joint_positions[self._grippers_dof_indices[0]]
        current_gripper_position_2 = current_joint_positions[self._grippers_dof_indices[1]]
        current_gripper_position_3 = current_joint_positions[self._grippers_dof_indices[2]]
        current_gripper_position_4 = current_joint_positions[self._grippers_dof_indices[3]]
        current_gripper_position_5 = current_joint_positions[self._grippers_dof_indices[4]]
        current_gripper_position_6 = current_joint_positions[self._grippers_dof_indices[5]]
        target_joint_positions = [None] * current_joint_positions.shape[0]
        target_joint_positions[self._grippers_dof_indices[0]] = current_gripper_position_1 + self._deltas[0]
        target_joint_positions[self._grippers_dof_indices[1]] = current_gripper_position_2 + self._deltas[1]
        target_joint_positions[self._grippers_dof_indices[2]] = current_gripper_position_3 + self._deltas[2]
        target_joint_positions[self._grippers_dof_indices[3]] = current_gripper_position_4 + self._deltas[3]
        target_joint_positions[self._grippers_dof_indices[4]] = current_gripper_position_5 + self._deltas[4]
        target_joint_positions[self._grippers_dof_indices[5]] = current_gripper_position_6 + self._deltas[5]
        return ArticulationAction(joint_positions=target_joint_positions)

class Grip(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_event = 0
        self._gripper_offset = 25
        self._grip = False

        self._cs = _contact_sensor.acquire_contact_sensor_interface()
        self.props = _contact_sensor.SensorProperties()
        self.props.radius = 2 # Cover the body tip
        self.props.minThreshold = 0
        self.props.maxThreshold = 1000000000000
        self.props.sensorPeriod = 1 / 100.0
        self.props.position = carb.Float3(0.5,0,2) # Offset sensor in X,Y,Z direction from rigid body center

        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._ur10 = scene.add(UR10(prim_path="/World/UR10",name="ur10"))
        self._cube = scene.add(DynamicCuboid(prim_path="/World/Cube",name="cube",
                                position=np.array([40,50,5.5]),color=np.array([100/255,50/255,70/255]),
                                size=np.array([5,5,11])))
        self._sensor_handle = self._cs.add_sensor_on_body("/World/UR10/right_inner_finger",self.props)                        
        
        return

    def get_observations(self):
        cube_position,_ = self._cube.get_world_pose()
        current_joint_positions = self._ur10.get_joint_positions()
        readings = self._cs.get_sensor_readings(self._sensor_handle) # readings are in kg⋅m⋅s−2
        observations = {
            "sensor_value":readings,
            "task_event":self._task_event,
            self._ur10.name: {
                "joint_positions": current_joint_positions,
            },
            self._cube.name: {
                "position": cube_position
            },
        }
        return observations
    
    def pre_step(self, control_idx, simulation_time):
        """ Check parameters before simulation step"""
        if self._task_event == 0:
            current_joint_position = self._ur10.get_joint_positions()
            if np.mean(np.abs(current_joint_position[6] - np.pi/6)) > 1:
                self._task_event += 1
        elif self._task_event == 1:
            cube_position,_ = self._cube.get_world_pose()
            current_gripper_joint_position,_=self._ur10.end_effector.get_world_pose()
            cube_position[2] = self._gripper_offset
            if np.mean(np.abs(current_gripper_joint_position - cube_position)) < 0.01:
                self._task_event += 1
        elif self._task_event == 2:
            current_joint_position = self._ur10.get_joint_positions()
            if current_joint_position[6] > 0.5:
                self._task_event += 1
            
class GripperUR(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return
    
    def setup_scene(self):
        world = self.get_world() # Define World

        world.add_task(Grip(name="grip_task"))

        self.gripper_dof_indices = [6,7,8,9,10,11]

        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._ur10 = self._world.scene.get_object("ur10")

        self._controller = RobotiqGripperController(
            gripper_dof_indices=self.gripper_dof_indices
        )

        self._ur_controller = RMPFlowController(name="UR_Controller",robot_prim_path="/World/UR10")

        self._world.add_physics_callback("sim_step",callback_fn=self.physics_step)

        await self._world.play_async()

        return

    async def setup_pre_reset(self):
        self._controller.reset()
        self._ur_controller.reset()
        await self._world.play_async()

        return

    def world_cleanup(self):
        return
    
    def physics_step(self, step_size):
        self._offset = 25
        current_observations = self._world.get_observations()
        
        print(current_observations["sensor_value"])
        # readings are in kg⋅m⋅s−2

        if current_observations["task_event"] == 0:
            self._ur10.apply_action(
                self._controller.forward(
                    action = "open",
                    current_joint_positions=current_observations["ur10"]["joint_positions"]
                )
            )
        elif current_observations["task_event"] == 1:
            self._ur10.apply_action(
                self._ur_controller.forward(
                    target_end_effector_position=np.array([40,50,self._offset]),
                    target_end_effector_orientation=euler_angles_to_quat(np.array([0,np.pi/2,0]))
                )
            )
        elif current_observations["task_event"] == 2:
            self._ur10.apply_action(
                self._controller.forward(
                    action="close",
                    current_joint_positions=current_observations["ur10"]["joint_positions"]
                )
            )
        elif current_observations["task_event"] == 3:
            self._ur10.apply_action(
                self._ur_controller.forward(
                    target_end_effector_position=np.array([30,50,50]),
                    target_end_effector_orientation=euler_angles_to_quat(np.array([0,np.pi/2,0]))
                )
            )

        return

import numpy as np
import carb
import omni.kit.window.property
from pxr import Gf, Sdf, UsdGeom, Usd, UsdPhysics
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from isaacsim.replicator.behavior.base_behavior import BaseBehavior
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_exposed_variables,
)
from omni.kit.scripting import BehaviorScript
from omni.isaac.universal_robots import KinematicsSolver #FollowTarget,
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

class MoveRobot(BehaviorScript):

    BEHAVIOR_NS = "exampleBehavior"

    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "targetVelocity",
            "attr_type": Sdf.ValueTypeNames.Vector3d,
            "default_value": Gf.Vec3d(0.0, 0.0, 0.0),
            "doc": "The 3D vector specifying the velocities.",
        },
        {
            "attr_name": "targetPrimPath",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": "",
            "doc": "The path of the target prim to look at. If specified, it has priority over the target location.",
        },
        # Additional variables...
    ]

    _wheel_fl_joint = "wheels/wheel_drive/wheel_fl_joint"
    _wheel_fr_joint = "wheels/wheel_drive/wheel_fr_joint"
    _wheel_rl_joint = "wheels/wheel_drive/wheel_rl_joint"
    _wheel_rr_joint = "wheels/wheel_drive/wheel_rr_joint"

    _wheel_radius = 0.13735/2 # radius of the wheel (in meters)
    _wheel_track = 0.54
    _wheel_base = 0.63

    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")

        """Called when the script is assigned to a prim."""
        self._interval = 0
        self._update_counter = 0
        self._valid_prims = []

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

        self._ur5 = Robot(prim_path=str(self.prim_path), name="ur5")
        print("Self : ", self._ur5)
        

        # Refresh the property windows to show the exposed variables
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")
        """Called when the script is unassigned from a prim."""
        #self._reset()
        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)
            omni.kit.window.property.get_window().request_rebuild()

    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")
        #self._ur5.post_reset()

    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")

        #function with parameters - linear velocity x (forward and backward), angular velocity y (left and right), angular velocity z (rotation around the z axis) to move the robot
        x,y,z = get_exposed_variable(self.prim,"exposedVar:exampleBehavior:targetVelocity")

        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = (x/self._wheel_radius - y/self._wheel_radius - (self._wheel_base + self._wheel_track)*z)*180/np.pi
        joint_velocities[1] = (x/self._wheel_radius + y/self._wheel_radius + (self._wheel_base + self._wheel_track)*z)*180/np.pi
        joint_velocities[2] = (x/self._wheel_radius + y/self._wheel_radius - (self._wheel_base + self._wheel_track)*z)*180/np.pi
        joint_velocities[3] = (x/self._wheel_radius - y/self._wheel_radius + (self._wheel_base + self._wheel_track)*z)*180/np.pi

        stage = omni.usd.get_context().get_stage()
        # prim = stage.GetPrimAtPath(self.prim_path)

        # carb.log_info(prim)
        # carb.log_info(self.wheel_fl_joint)

        wheel_fl = stage.GetPrimAtPath(str(self.prim_path) + '/' + self._wheel_fl_joint)
        wheel_fr = stage.GetPrimAtPath(str(self.prim_path) + '/' + self._wheel_fr_joint)
        wheel_rl = stage.GetPrimAtPath(str(self.prim_path) + '/' + self._wheel_rl_joint)
        wheel_rr = stage.GetPrimAtPath(str(self.prim_path) + '/' + self._wheel_rr_joint)

        UsdPhysics.DriveAPI.Get(wheel_fl, "angular").GetTargetVelocityAttr().Set(joint_velocities[0])
        UsdPhysics.DriveAPI.Get(wheel_fr, "angular").GetTargetVelocityAttr().Set(joint_velocities[1])
        UsdPhysics.DriveAPI.Get(wheel_rl, "angular").GetTargetVelocityAttr().Set(joint_velocities[2])
        UsdPhysics.DriveAPI.Get(wheel_rr, "angular").GetTargetVelocityAttr().Set(joint_velocities[3])
        #carb.log_info("Angular: " + str(wheel_fl_joint1.Get()))

        self._ur5.initialize()
        #ur5_KinSolver = KinematicsSolver(self._ur5)
        # move all the robot joints to the indicated position
        #print("Robot joint positions: ", self._ur5.get_applied_action())
        
        action = ArticulationAction(joint_positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4]), joint_indices=np.array([2,0,1,3,8,37]))
        self._ur5.apply_action(action)

        #action = ArticulationAction(joint_positions=np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]), joint_indices=np.array([0,1,2,3,4,5,6,7,8]))

        #print("ur_arm_wrist_3_joint :", self._ur5.get_dof_index("ur_arm_wrist_3_joint"))
        print("ur_arm_wrist_3_joint :", self._ur5.dof_names)






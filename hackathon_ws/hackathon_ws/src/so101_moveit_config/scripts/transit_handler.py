import rclpy
from moveit_msgs.msg import Constraints, OrientationConstraint
# ... (standard ROS 2 imports)

def plan_constrained_path(move_group, target_pose):
    # 1. Define the Bi-RRT (RRT-Connect) Solver
    move_group.set_planner_id("RRTConnectkConfigDefault")
    
    # 2. Set the 'Orientation_Lock' for the bottle (Upright Constraint)
    constraints = Constraints()
    oc = OrientationConstraint()
    oc.header.frame_id = "base_link"
    oc.link_name = "gripper_link" # From your URDF [cite: 17]
    oc.orientation.w = 1.0  # Upright (Level with ground)
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14 # Allow rotation around the bottle's center
    oc.weight = 1.0
    
    constraints.orientation_constraints.append(oc)
    move_group.set_path_constraints(constraints)
    
    # 3. Plan Phase 3 (Transit to Cup)
    move_group.set_pose_target(target_pose)
    return move_group.plan()
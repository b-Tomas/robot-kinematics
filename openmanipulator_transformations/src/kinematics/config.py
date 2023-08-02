from numpy import arctan2, array
from numpy.linalg import norm

INV_TRANSFORM_SERVICE_NAME = "/kinematics/inverse_transformation"
INV_TRANSFORM_NODE_NAME = "inv_transform_srv"
FWD_TRANSFORM_SERVICE_NAME = "/kinematics/forward_transformation"
FWD_TRANSFORM_NODE_NAME = "fwd_transform_srv"

# OpenManipulator control service
ROBOT_CONTROL_SERVICE_NAME = "/goal_joint_space_path"

# Distances between joints
# Obtained from https://github.com/ROBOTIS-GIT/open_manipulator/blob/07625015a0088a069575f7e6e74eb931edce9ed3/open_manipulator_description/urdf/open_manipulator.urdf.xacro
# The origins of the joints are relative to the origin of the parent link

# Child of link1, parent of link2. Rotates in the XY plane
JOINT_1_ORIGIN = array([0.012, 0, 0.017])
# Child of link2, parent of link3. Rotates in the XZ plane
JOINT_2_ORIGIN = array([0, 0, 0.0595])
# Child of link3, parent of link4. Rotates in the XZ plane
JOINT_3_ORIGIN = array([0.024, 0, 0.128])
# Child of link3, parent of link5 (between joint 3 and gripper joint). Rotates in the XZ plane
JOINT_4_ORIGIN = array([0.124, 0, 0])
# Child of link5, which starts at joint4
JOINT_END_EFFECTOR = array([0.126, 0, 0])

# Link2 is the one above joint1 and because of that it rotates around its longitudinal axis. It is always pointing up
LINK_2_LONGITUDE = norm(JOINT_2_ORIGIN)
LINK_3_LONGITUDE = norm(JOINT_3_ORIGIN)
LINK_4_LONGITUDE = norm(JOINT_4_ORIGIN)
# Distance from joint 4 to the center of the end-effector (the red cube in gazebo)
LAST_BIT_LONGITUDE = norm(JOINT_END_EFFECTOR)

# These are the final parameters used for computation
LINK_LONGITUDES = (
    LINK_2_LONGITUDE,
    LINK_3_LONGITUDE,
    LINK_4_LONGITUDE,
    LAST_BIT_LONGITUDE,
)
OFFSET_XYZ = tuple(JOINT_1_ORIGIN)

# If you look closely, you will notice that joints 2 and 3 angles are not exactly at a 90° relative to fully extended
# This is how we calculate the angle
offset = arctan2(JOINT_3_ORIGIN[2], JOINT_3_ORIGIN[0])
JOINT_OFFSETS = (0, -offset, offset, 0)

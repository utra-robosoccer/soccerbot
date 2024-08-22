import placo


# TODO fix in future
def walk_parameters_bez1():
    # Walk parameters - if double_support_ratio is not set to 0, should be greater than replan_frequency
    parameters = placo.HumanoidParameters()

    # Timing parameters
    parameters.single_support_duration = 0.25  # Duration of single support phase [s]
    parameters.single_support_timesteps = 10  # Number of planning timesteps per single support phase
    parameters.double_support_ratio = 0.0  # Ratio of double support (0.0 to 1.0)
    parameters.startend_double_support_ratio = 2.0  # Ratio duration of supports for starting and stopping walk
    parameters.planned_timesteps = 48  # Number of timesteps planned ahead
    parameters.replan_timesteps = 10  # Replanning each n timesteps    # 50

    # Posture parameters
    parameters.walk_com_height = 0.21  # Constant height for the CoM [m]
    parameters.walk_foot_height = 0.04  # Height of foot rising while walking [m]
    parameters.walk_trunk_pitch = 0.0  # Trunk pitch angle [rad]
    parameters.walk_foot_rise_ratio = 0.2  # Time ratio for the foot swing plateau (0.0 to 1.0)

    # Feet parameters
    parameters.foot_length = 0.1200  # Foot length [m]
    parameters.foot_width = 0.072  # Foot width [m]
    parameters.feet_spacing = 0.122  # Lateral feet spacing [m]
    parameters.zmp_margin = 0.02  # ZMP margin [m]
    parameters.foot_zmp_target_x = 0.0  # Reference target ZMP position in the foot [m]
    parameters.foot_zmp_target_y = 0.0  # Reference target ZMP position in the foot [m]

    # Limit parameters
    parameters.walk_max_dtheta = 1  # Maximum dtheta per step [rad]
    parameters.walk_max_dy = 0.04  # Maximum dy per step [m]
    parameters.walk_max_dx_forward = 0.08  # Maximum dx per step forward [m]
    parameters.walk_max_dx_backward = 0.03  # Maximum dx per step backward [m]
    return parameters


def walk_parameters_bez2():
    # Walk parameters - if double_support_ratio is not set to 0, should be greater than replan_frequency
    parameters = placo.HumanoidParameters()

    # Timing parameters
    parameters.single_support_duration = 0.3  # Duration of single support phase [s]
    parameters.single_support_timesteps = 10  # Number of planning timesteps per single support phase
    parameters.double_support_ratio = 0.0  # Ratio of double support (0.0 to 1.0)
    parameters.startend_double_support_ratio = 1.5  # Ratio duration of supports for starting and stopping walk
    parameters.planned_timesteps = 48  # Number of timesteps planned ahead
    parameters.replan_timesteps = 10  # Replanning each n timesteps

    # Posture parameters
    parameters.walk_com_height = 0.25  # Constant height for the CoM [m]
    parameters.walk_foot_height = 0.04  # Height of foot rising while walking [m]
    parameters.walk_trunk_pitch = 0.0  # Trunk pitch angle [rad]
    parameters.walk_foot_rise_ratio = 0.2  # Time ratio for the foot swing plateau (0.0 to 1.0)

    # Feet parameters
    parameters.foot_length = 0.1576  # Foot length [m]
    parameters.foot_width = 0.092  # Foot width [m]
    parameters.feet_spacing = 0.122  # Lateral feet spacing [m]
    parameters.zmp_margin = 0.02  # ZMP margin [m]
    parameters.foot_zmp_target_x = 0.0  # Reference target ZMP position in the foot [m]
    parameters.foot_zmp_target_y = 0.0  # Reference target ZMP position in the foot [m]

    # Limit parameters
    parameters.walk_max_dtheta = 1  # Maximum dtheta per step [rad]
    parameters.walk_max_dy = 0.04  # Maximum dy per step [m]
    parameters.walk_max_dx_forward = 0.08  # Maximum dx per step forward [m]
    parameters.walk_max_dx_backward = 0.03  # Maximum dx per step backward [m]
    return parameters


def walk_parameters_sigmaban():
    # Walk parameters - if double_support_ratio is not set to 0, should be greater than replan_frequency
    parameters = placo.HumanoidParameters()

    # Timing parameters
    parameters.single_support_duration = 0.38  # Duration of single support phase [s]
    parameters.single_support_timesteps = 10  # Number of planning timesteps per single support phase
    parameters.double_support_ratio = 0.0  # Ratio of double support (0.0 to 1.0)
    parameters.startend_double_support_ratio = 1.5  # Ratio duration of supports for starting and stopping walk
    parameters.planned_timesteps = 48  # Number of timesteps planned ahead
    parameters.replan_timesteps = 10  # Replanning each n timesteps

    # Posture parameters
    parameters.walk_com_height = 0.32  # Constant height for the CoM [m]
    parameters.walk_foot_height = 0.04  # Height of foot rising while walking [m]
    parameters.walk_trunk_pitch = 0.15  # Trunk pitch angle [rad]
    parameters.walk_foot_rise_ratio = 0.2  # Time ratio for the foot swing plateau (0.0 to 1.0)

    # Feet parameters
    parameters.foot_length = 0.1576  # Foot length [m]
    parameters.foot_width = 0.092  # Foot width [m]
    parameters.feet_spacing = 0.122  # Lateral feet spacing [m]
    parameters.zmp_margin = 0.02  # ZMP margin [m]
    parameters.foot_zmp_target_x = 0.0  # Reference target ZMP position in the foot [m]
    parameters.foot_zmp_target_y = 0.0  # Reference target ZMP position in the foot [m]

    # Limit parameters
    parameters.walk_max_dtheta = 1  # Maximum dtheta per step [rad]
    parameters.walk_max_dy = 0.04  # Maximum dy per step [m]
    parameters.walk_max_dx_forward = 0.08  # Maximum dx per step forward [m]
    parameters.walk_max_dx_backward = 0.03  # Maximum dx per step backward [m]
    return parameters

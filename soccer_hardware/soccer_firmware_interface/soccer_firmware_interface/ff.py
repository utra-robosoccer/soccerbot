import yaml

motor_types = {}
with open("/home/manx52/ros2_ws/src/soccerbot/soccer_hardware/soccer_firmware_interface/config/motor_types.yaml") as f:
    motor_types = yaml.safe_load(f)

with open("/home/manx52/ros2_ws/src/soccerbot/soccer_hardware/soccer_firmware_interface/config/bez2.yaml") as f:
    motor_mapping = yaml.safe_load(f)
        
motor_id_to_name_dict = {motor_mapping[m]["id"]: m for m in motor_mapping}
motor_name = motor_id_to_name_dict[0]
motor_type = motor_mapping[motor_name]["type"]
motor_angle_zero = motor_mapping[motor_name]["angle_zero"]
max_angle_bytes = motor_types[motor_type]["max_angle_bytes"]
max_angle_radians = motor_types[motor_type]["max_angle_degrees"]
print("Motor name:", motor_name)
print("Motor type:", motor_type)
print("Motor angle zero:", motor_angle_zero)
print("Max angle:", max_angle_bytes)
print("Max angle degrees:", max_angle_radians)

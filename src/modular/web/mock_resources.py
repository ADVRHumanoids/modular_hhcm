#!/usr/bin/env python3
# Disable some of the pylint violations in this file
# see https://pylint.pycqa.org/en/latest/user_guide/messages/message_control.html#block-disables
# pylint: disable=line-too-long, missing-function-docstring

# used in [GET /resurces/modules] and [GET /resurces/families]
#   modular: yaml/{fn}.yaml
#   concert: json/concert/{fn}.json see https://github.com/ADVRHumanoids/concert_resources/tree/master/json/concert
resources = {
  "families": [
    { # TREE Modules (disabled)
      "id": "alberoboticsGen0",
      "group": "Alberobotics",
      "label": "TREE Modules",
      "disabled": True
    },
    { # PINO v1
      "id": "alberoboticsGenA",
      "group": "Alberobotics",
      "label": "PINO v1"
    },
    { # CONCERT
      "id": "alberoboticsGenB",
      "group": "Alberobotics",
      "label": "CONCERT"
    }
  ],
  "modules": [
  { # [PINO v1] Socket - socket
    "family": "alberoboticsGenA",
    "type": "Socket",
    "product":"yaml/socket.yaml",
    "label":"Socket"
  },
  { # [PINO v1] Cube - master_cube
    "family": "alberoboticsGenA",
    "type": "cube",
    "product":"yaml/master_cube.yaml",
    "label":"Cube",
    "disabled": True
  },
  { # [PINO v1] Elbow Joint - module_joint_double_elbow_ORANGE
    "family": "alberoboticsGenA",
    "type": "joint",
    "product":"yaml/module_joint_double_elbow_ORANGE.yaml",
    "label":"Elbow Joint"
  },
  { # [PINO v1] Elbow Joint - module_joint_elbow_ORANGE
    "family": "alberoboticsGenA",
    "type": "joint",
    "product":"yaml/module_joint_elbow_ORANGE.yaml",
    "label":"Elbow Joint",
    "disabled": True
  },
  { # [PINO v1] Straight Joint - module_joint_yaw_ORANGE
    "family": "alberoboticsGenA",
    "type": "joint",
    "product":"yaml/module_joint_yaw_ORANGE.yaml",
    "label":"Straight Joint",
  },
  { # [PINO v1] Passive Link L 135° - module_link_elbow_135
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_elbow_135.yaml",
    "label":"Passive Link L 135°"
  },
  { # [PINO v1] Passive Link L 45° - module_link_elbow_45
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_elbow_45.yaml",
    "label":"Passive Link L 45°"
  },
  { # [PINO v1] Passive Link L 90° - module_link_elbow_90
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_elbow_90.yaml",
    "label":"Passive Link L 90°"
  },
  { # [PINO v1] Passive Link Straight 70mm - module_link_straight_70
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_straight_70.yaml",
    "label":"Passive Link Straight 70mm"
  },
  { # [PINO v1] Passive Link Straight 140mm - module_link_straight_140
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_straight_140.yaml",
    "label":"Passive Link Straight 140mm"
  },
  { # [PINO v1] Passive Link Straight 350mm - module_link_straight_350
    "family": "alberoboticsGenA",
    "type": "link",
    "product":"yaml/module_link_straight_350.yaml",
    "label":"Passive Link Straight 350mm"
  },
  { # [PINO v1] Tool Exchanger - module_tool_exchanger_heavy
    "family": "alberoboticsGenA",
    "type": "tool_exchanger",
    "product":"yaml/module_tool_exchanger_heavy.yaml",
    "label":"Tool Exchanger"
  },
  { # [PINO v1] Tool Exchanger - module_tool_exchanger
    "family": "alberoboticsGenA",
    "type": "tool_exchanger",
    "product":"yaml/module_tool_exchanger.yaml",
    "label":"Tool Exchanger Light",
    "disabled": True
  },
  { # [PINO v1] Gripper - module_gripper
    "family": "alberoboticsGenA",
    "type": "gripper",
    "product":"yaml/module_gripper.yaml",
    "label":"Gripper"
  },

  { # [CONCERT] Mobile Platform - concert_mobile_platform
    "family": "alberoboticsGenB",
    "type": "mobile_base",
    "product": "json/concert/mobile_platform_concert.json",
    "label": "Mobile Platform",
  },
  { # [CONCERT] Mobile Platform (simplified) - concert_mobile_platform_simplified
    "family": "alberoboticsGenB",
    "type": "mobile_base",
    "product": "json/concert/mobile_platform_concert_timor_planar.json",
    "label": "Mobile Platform (simplified)",
  },
  { # [CONCERT] Drill - module_drill_concert
    "family": "alberoboticsGenB",
    "type": "tool_exchanger",
    "product": "json/concert/module_drill_concert.json",
    "label": "Drill",
  },
  { # [CONCERT] Elbow Joint (Type A) - module_joint_elbow_A_concert
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_joint_elbow_A_concert.json",
    "label": "Elbow Joint (Type A)",
  },
  { # [CONCERT] Elbow Joint (Type B) - module_joint_elbow_B_concert
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_joint_elbow_B_concert.json",
    "label": "Elbow Joint (Type B)",
  },
  { # [CONCERT] Straight Joint (Type A) - module_joint_yaw_A_concert
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_joint_yaw_A_concert.json",
    "label": "Straight Joint (Type A)",
  },
  { # [CONCERT] Straight Joint (Type B) - module_joint_yaw_B_concert
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_joint_yaw_B_concert.json",
    "label": "Straight Joint (Type B)",
  },
  { # [CONCERT] Passive Link Straight 10cm - module_link_straight_10_concert
    "family": "alberoboticsGenB",
    "type": "link",
    "product": "json/concert/module_link_straight_10_concert.json",
    "label": "Passive Link Straight 10cm",
  },
  { # [CONCERT] Passive Link Straight 20cm - module_link_straight_20_concert
    "family": "alberoboticsGenB",
    "type": "link",
    "product": "json/concert/module_link_straight_20_concert.json",
    "label": "Passive Link Straight 20cm",
  },
  { # [CONCERT] Passive Link Straight 50cm - module_link_straight_50_concert
    "family": "alberoboticsGenB",
    "type": "link",
    "product": "json/concert/module_link_straight_50_concert.json",
    "label": "Passive Link Straight 20cm",
  },
  { # [CONCERT] concert_steering_module_fl_rr
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_steering_concert_fl_rr.json",
    "label": "concert_steering_module_fl_rr",
  },
  { # [CONCERT] concert_steering_module_fr_rl
    "family": "alberoboticsGenB",
    "type": "joint",
    "product": "json/concert/module_steering_concert_fr_rl.json",
    "label": "concert_steering_module_fr_rl",
  },
  { # [CONCERT] Wheel - concert_wheel_module
    "family": "alberoboticsGenB",
    "type": "wheel",
    "product": "json/concert/module_wheel_concert.json",
    "label": "Wheel",
  },
  { # [CONCERT] Passive End Effector Panel - passive_end_effector_panel
    "family": "alberoboticsGenB",
    "type": "end_effector",
    "product": "json/concert/passive_end_effector_panel.json",
    "label": "Passive End Effector Panel",
  },
  { # [CONCERT] Passive End Effector Tube - passive_end_effector_tube
    "family": "alberoboticsGenB",
    "type": "end_effector",
    "product": "json/concert/passive_end_effector_tube.json",
    "label": "Passive End Effector Tube",
  },
  ]
}

# Modules
modules = resources['modules']
def get_avalilable_modules():
    return modules

def get_avalilable_module_types():
    module_types = [el['type'] for el in modules]
    return list(dict.fromkeys(module_types))

# Families
families = resources['families']
def get_avalilable_families():
    return families

def get_avalilable_family_ids():
    family_ids = [el['id'] for el in families]
    return list(dict.fromkeys(family_ids))

#!/usr/bin/env python3
# Disable some of the pylint violations in this file
# see https://pylint.pycqa.org/en/latest/user_guide/messages/message_control.html#block-disables
# pylint: disable=line-too-long, missing-function-docstring, missing-module-docstring

# used in [GET /model/stats]
stats={
  "modules": { "label": 'Modules', "value": '1' },
  "payload": { "label": 'Payload', "value": '75', "unit": 'Kg' },
  "reach": { "label": 'Reach', "value": '2.25', "unit": 'm' },
}

# Modules
def get_stats():
  return stats

# A message type for requesting docking or undocking
# The available commands are:
# 0 - dock
# 1 - undock
# 2 - cancel dock
# 3 - cancel undock

uint8 DOCK            = 0
uint8 UNDOCK          = 1
uint8 CANCEL_DOCK     = 2
uint8 CANCEL_UNDOCK   = 3

uint8 value

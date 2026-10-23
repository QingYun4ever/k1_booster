# Team data shared by GameController v12 and local v20.

uint8 team_number
uint8 field_player_colour
uint8 goalkeeper_colour         # v20; zero for v12
uint8 goalkeeper                # v20 player number; derived from v12 player flags
uint8 score
uint8 penalty_shot
uint16 single_shots
uint16 message_budget           # v20; zero for v12
uint8 coach_sequence
uint8[] coach_message           # v12 payload; empty for v20
RobotInfo coach
RobotInfo[] players             # 11 slots for v12, 20 slots for v20
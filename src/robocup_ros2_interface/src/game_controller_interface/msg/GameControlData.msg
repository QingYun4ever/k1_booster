# GameController v12 and local v20 wire data, normalized for ROS consumers.

char[4] header
uint16 version
uint8 packet_number
uint8 players_per_team
uint8 game_type                 # v12 game type; v20 competitionType
uint8 state
bool stopped                    # v20 stopped flag; v12 STATE_DANIEL projects as stopped
uint8 game_phase                # raw v20 gamePhase; zero for v12
uint8 set_play                  # raw v20 setPlay; zero for v12
uint8 first_half
uint8 kick_off_team             # v20 kickingTeam
uint8 secondary_state           # legacy-compatible projection for v20
char[4] secondary_state_info
uint8 drop_in_team
uint16 drop_in_time
int16 secs_remaining            # signed ROS view of the wire time
int16 secondary_time            # signed ROS view of the wire time
TeamInfo[2] teams
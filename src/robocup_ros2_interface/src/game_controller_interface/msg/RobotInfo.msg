# Player status in the GameController packet.

uint8 penalty
uint8 secs_till_unpenalised
uint8 number_of_warnings
uint8 yellow_card_count
uint8 red_card_count
uint8 cautions                  # raw v20 cautions; zero for v12
bool goal_keeper
bool available                  # false for slots outside players_per_team
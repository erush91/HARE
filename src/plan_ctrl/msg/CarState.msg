Header  header # ------ Time info
string  state # ------- Name of the state
string  reason # ------ Reason for transition
bool    FLAG_newCtrl #- Was there a new control command issued this timestep?
bool    FLAG_goodScan # Was there a good scan noted this timestep?
float32 steerAngle # -- Steering angle demanded
float32 linearSpeed # - Motor speed demanded
float32 up # ---------- Proportional command
float32 ui # ---------- Integral command
float32 ud # ---------- Derivative command
int16   turn_count # -- What turn is coming up?
float32 preturn_timer # Drift timing
float32 forward_timer # Amount of time in straightaway
bool    turn_debounce # Are we suppressing a turn count?

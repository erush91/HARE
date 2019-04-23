Header  header # ------ Time info
string  state # ------- Name of the state
string  reason # ------ Reason for transition
bool    FLAG_newCtrl #- Was there a new control command issued this timestep?
bool    FLAG_goodScan # Was there a good scan noted this timestep?
float32 steerAngle # -- Steering angle demanded
float32 linearSpeed # - Motor speed demanded
float32 up
float32 ui
float32 ud

##RAM_MAP
uint8  id
uint8  ackpolicy
uint8  alarmledpolicy
uint8  torquepolicy
uint8  reserved2
uint8  maxtemperature
uint8  minvoltage
uint8  maxvoltage
uint8  accelerationratio
uint8  maxaccelerationtime
uint8  deadzone
uint8  saturatoroffset
uint16 saturatorslope
int8   pwmoffset
uint8  minpwm
uint16 maxpwm
uint16 overloadpwmthreshold
uint16 minposition
uint16 maxposition
uint16 positionkp
uint16 positionkd
uint16 positionki
uint16 positionfeedforward1stgain
uint16 positionfeedforward2ndgain
uint16 velocitykd
uint16 velocityki
uint8  ledblinkperiod
uint8  adcfaultcheckperiod
uint8  packetgarbagecheckperiod
uint8  stopdetectionperiod
uint8  overloaddetectionperiod
uint8  stopthreshold
uint8  inpositionmargin
uint8  turn
int8   calibrationdifference_l
int8   calibrationdifference_h
uint8  statuserror
uint8  statusdetail
uint8  reserved7
uint8  reserved8
uint8  torquecontrol
uint8  ledcontrol
uint8  voltage
uint8  temperature
uint8  currentcontrolmode
uint8  tick
uint16 calibratedposition
uint16 absoluteposition
int16  differentialposition
uint16 pwm
uint16 reserved9
uint16 absolutegoalposition
uint16 absolutedesiredtrajectoryposition
int16  desiredvelocity


#define HARD_REV_FIXED    1
#define HARD_REV_COOLANT  2

byte checkEngineProtect(void);
byte checkRevLimit(void);
byte checkBoostLimit(void);
byte checkOilPressureLimit(void);
byte checkAFRLimit(void);
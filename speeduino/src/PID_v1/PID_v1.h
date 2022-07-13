#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
    PID(long*, long*, long*,        // * constructor.  links the PID to the Input, Output, and
        byte, byte, byte, byte);     //   Setpoint.  Initial tuning parameters are also set here

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(long, long); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(byte, byte,       // * While most users will set the tunings once in the
                    byte);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
	int16_t GetKp();						  // These functions query the pid for interal values.
	int16_t GetKi();						  //  they were created mainly for the pid front-end,
	int16_t GetKd();						  // where it's important to know what is actually
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();

	long dispKp;				// * we'll hold on to the tuning parameters in user-entered
	long dispKi;				//   format for display purposes
	long dispKd;				//

	long kp;                  // * (P)roportional Tuning Parameter
  long ki;                  // * (I)ntegral Tuning Parameter
  long kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    long *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    long *myOutput;             //   This creates a hard link between the variables and the
    long *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	long ITerm, lastInput;

	unsigned long SampleTime;
	long outMin, outMax;
	bool inAuto;
};

class integerPID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define PID_SHIFTS  10 //Increased resolution

  //commonly used functions **************************************************************************
    integerPID(long*, long*, long*,        // * constructor.  links the PID to the Input, Output, and
        int16_t, int16_t, int16_t, byte);     //   Setpoint.  Initial tuning parameters are also set here


    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute(bool, long FeedForwardTerm = 0);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
    bool Compute2(int, int, bool);
    bool ComputeVVT(uint32_t);
    
    void SetOutputLimits(long, long); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(int16_t, int16_t,       // * While most users will set the tunings once in the
                    int16_t, byte=0);       	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
  void SetSampleTime(uint16_t);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //
  void Initialize();
  void ResetIntegeral();

  private:


  int16_t dispKp;
  int16_t dispKi;
  int16_t dispKd;
	int16_t  kp;                  // * (P)roportional Tuning Parameter
  int16_t  ki;                  // * (I)ntegral Tuning Parameter
  int16_t  kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    long *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    long *myOutput;             //   This creates a hard link between the variables and the
    long *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	long outputSum, lastInput, lastMinusOneInput;
  int16_t lastError;

	uint16_t SampleTime;
	long outMin, outMax;
	bool inAuto;
};

class integerPID_ideal
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
    integerPID_ideal(long*, uint16_t*, uint16_t*, uint16_t*, byte*,        // * constructor.  links the PID to the Input, Output, and
        byte, byte, byte, byte);     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
    bool Compute(uint16_t);               // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(long, long); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(byte, byte,       // * While most users will set the tunings once in the
                    byte);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.



  //Display functions ****************************************************************
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //
	void Initialize();

  private:


	byte dispKp;				// * we'll hold on to the tuning parameters in user-entered
	byte dispKi;				//   format for display purposes
	byte dispKd;				//

	uint16_t  kp;                  // * (P)roportional Tuning Parameter
  uint16_t  ki;                  // * (I)ntegral Tuning Parameter
  uint16_t  kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

    long *myInput;              //
    uint16_t *myOutput;         //   This is a percentage figure multipled by 100 (To give 2 points of precision)
    uint16_t *mySetpoint;       //
    uint16_t *mySensitivity;
    byte *mySampleTime;


	unsigned long lastTime;
  long lastError;
	long ITerm, lastInput;

	long outMin, outMax;
};
#endif

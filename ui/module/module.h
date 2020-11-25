extern "C" {

#define MAX_STEPS  50
#define NUM_MOTORS 4
#define NUM_STATES 30
#define NUM_POLY   100

#define MAX_MEASUREMENTS 200

const int RUNTIME_STATE_EXIT = 0;
const int RUNTIME_STATE_IDLE = 1;
const int RUNTIME_STATE_MOVE_FORWARD = 2;
const int RUNTIME_STATE_MOVE_BACKWARD = 3;

const float RUNTIME_DEFAULT_IDLE_PAUSE = 0.99;
const float DEFAULT_PAUSE_RATE = 0.004;

const int CYCLE_MOVEMENT  = 0;
const int CYCLE_TURN      = 1;
const int CYCLE_MOUTH     = 2;
const int CYCLE_FULL_SCAN = 3;
const int CYCLE_IDLE      = 4;
const int CYCLE_DISTANCE  = 5;

const int FLAGS_NO_FLAGS = 0;
const int FLAGS_ABS      = 1;

#define CONTROL_STEP2 1
#define CONTROL_STEP3 2

struct TServo {
	int		min;
	int		mid;
	int		max;
	int		value;
	float	angle;
};

struct TPoly4 {
	int servo;
	int s;
	int flags;
	float	k[4];
};

struct TState {
	int num;
	int servoPoly[7];
};

struct TSObj {
	float x;
	float y;
};

struct TServoControl {
	void*		 hardware;
	void*		 runtime_thread;
	int			 runtime_state;
	float		 runtime_pause;
	TServo		 servos[16];
	float		 pause_rate;
	float		 stepSize;
	float		 currentTurn;
	float		 nextTurn;
	int			 currentState;
	int			 currentCycle;
	int			 mouthState;
	int			 submitMouthState;
	int			 scanFlag;
	int 		 numPoly;
	int			 numStates;
	TState		 states[NUM_STATES];
	int			 numTurnStates;
	TState		 turnStates[NUM_STATES];
	TPoly4		 poly[NUM_POLY];
	void*		 sensor;
	float		 R;
	float		 scan_angle_low_0;
	float		 scan_angle_low_1;
	float		 scan_angle_high_0;
	float		 scan_angle_high_1;
	float		 scan_from2;
	float		 scan_to2;
	float		 scan_pre_pause;
	float		 scan_pause_move;
	float		 scan_step2;
	int		 	 scan_d;
	float 		 scan_min_conf;
	float		 scan_decidion;
	int          scan_found_object;
	float        scan_decidion_progress;
	int 		 scan_distance;
	int		     num_measurements;
	int 		 measurements[MAX_MEASUREMENTS];
	float 		 measurements2[MAX_MEASUREMENTS];
};

int initServos(TServoControl* control);
void setServoValue(TServoControl* control, int servo, int value);
float setServoAngle(TServoControl* control, int servo, float angle);
float updateState(TServoControl* control);
void setTurn(TServoControl* control, float angle);

int initRuntime(TServoControl* control);
int stopRuntime(TServoControl* control);

int initRangeFinder(TServoControl* control);
int readDistance(TServoControl* control);
void HScan2(TServoControl* control, float angle0, float angle1, float from2, float to2, float step2, float prePause, float pauseMove);
void HScan3(TServoControl* control);
void HScan4(TServoControl* control);
}

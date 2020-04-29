extern "C" {

#define MAX_STEPS  50
#define NUM_MOTORS 4
#define NUM_STATES 30
#define NUM_POLY   100

const int RUNTIME_STATE_EXIT = 0;
const int RUNTIME_STATE_IDLE = 1;
const int RUNTIME_STATE_MOVE_FORWARD = 2;
const int RUNTIME_STATE_MOVE_BACKWARD = 3;

const float RUNTIME_DEFAULT_IDLE_PAUSE = 0.99;
const float DEFAULT_PAUSE_RATE = 0.002;

const int CYCLE_MOVEMENT = 0;
const int CYCLE_TURN     = 1;
const int CYCLE_MOUTH    = 2;

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
	float	k[4];
};

struct TState {
	int num;
	int servoPoly[7];
};

struct TServoControl {
	void*		hardware;
	void*		runtime_thread;
	int			runtime_state;
	float		runtime_pause;
	TServo		servos[16];
	float		pause_rate;
	float		stepSize;
	float		currentTurn;
	float		nextTurn;
	int			currentState;
	int			currentCycle;
	int			mouthState;
	int			submitMouthState;
	int 		numPoly;
	int			numStates;
	TState		states[NUM_STATES];
	int			numTurnStates;
	TState		turnStates[NUM_STATES];
	TPoly4		poly[NUM_POLY];
};

int initServos(TServoControl* control);
void setServoValue(TServoControl* control, int servo, int value);
float setServoAngle(TServoControl* control, int servo, float angle);
float updateState(TServoControl* control);
void setTurn(TServoControl* control, float angle);

int initRuntime(TServoControl* control);
int stopRuntime(TServoControl* control);

}

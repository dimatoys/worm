extern "C" {

#define MAX_STEPS 50
#define NUM_MOTORS 4

const int RUNTIME_STATE_EXIT = 0;
const int RUNTIME_STATE_IDLE = 1;
const int RUNTIME_STATE_MOVE_FORWARD = 2;
const int RUNTIME_STATE_MOVE_BACKWARD = 3;

const float RUNTIME_DEFAULT_IDLE_PAUSE = 0.99;
const float DEFAULT_PAUSE_RATE = 0.002;

#define CONTROL_STEP2 1

struct TServo {
	int		min;
	int		mid;
	int		max;
	int		value;
	float	angle;
};

struct TWormStep {
	float	motors[NUM_MOTORS];
};

struct TServoControl {
	void*		hardware;
	void*		runtime_thread;
	int			runtime_state;
	float		runtime_pause;
	TServo		servos[16];
	int			numSteps;
	int			currentStep;
	float		pause_rate;
	TWormStep	steps[MAX_STEPS];
};

int initServos(TServoControl* control);
void setServoValue(TServoControl* control, int servo, int value);
int setServoAngle(TServoControl* control, int servo, float angle);
float moveToStep(TServoControl* control, int step);

int initRuntime(TServoControl* control);
int stopRuntime(TServoControl* control);

int wormControl(TServoControl* control, int id, float value);
}

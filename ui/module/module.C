#include "module.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include <cerrno>
#include <unistd.h>
#include <sys/select.h>

#ifdef RASPBERRY
#include <unistd.h>
#include "bcm2835.h"
#include "pca9685.h"
#endif

int initServos(TServoControl* control) {
	
	control->currentStep = 0;
	control->runtime_thread = NULL;
	
#ifdef RASPBERRY
	if (getuid() != 0) {
		fprintf(stderr, "Program is not started as \'root\' (sudo)\n");
		return -1;
	}

	if (bcm2835_init() != 1) {
		fprintf(stderr, "bcm2835_init() failed\n");
		return -2;
	}

	PCA9685* pca9685 = new PCA9685();
	control->hardware = pca9685;
	pca9685->SetFrequency(100);

#endif
	return 0;
}

void updateServoValue(void* pca9685, int servo, int value) {
	//printf("updateServoValue: %d:%d\n", servo, value);

#ifdef RASPBERRY
	((PCA9685*)pca9685)->Write(CHANNEL(servo), VALUE(value));
#endif
}

void setServoValue(TServoControl* control, int servo, int value) {
	TServo* srec = &control->servos[servo];
	srec->value = value;
	updateServoValue(control->hardware, servo, value);
}

float setServoAngle(TServoControl* control, int servo, float angle) {
	TServo* srec = &control->servos[servo];
	float diff = fabs(angle - srec->angle);
	//printf("SetServoAngle: %f -> %f, diff=%f\n", srec->angle, angle, diff);
	srec->angle = angle;
	srec->value = srec->mid + (angle >= 0 ? srec->max - srec->mid : srec->mid - srec->min) * angle / 90.0;
	updateServoValue(control->hardware, servo, srec->value);
	return diff;
}

float setMuscleValue(TServoControl* control, int i, float value) {
	float sv = 0;
	for (int j = control->poly[i].s - 1; j >= 0 ; --j) {
		sv = sv * value + control->poly[i].k[j];
	}
	float diff = setServoAngle(control, control->poly[i].servo, sv);
	printf("set %d angle %f diff=%f\n", control->poly[i].servo, sv, diff);
	return diff;
}

float updateState(TServoControl* control) {
	float maxDiff = 0;
	auto state = ((TServoControl*)control)->currentState;
	if (state == 0) {
		((TServoControl*)control)->currentTurn = ((TServoControl*)control)->nextTurn;
		printf("Turn updated to %f\n", ((TServoControl*)control)->currentTurn);
	}

	for (int i = 0; i < control->states[state].num; ++i) {
		auto diff = setMuscleValue(control, control->states[state].servoPoly[i], control->stepSize);
		if (diff > maxDiff) {
			maxDiff = diff;
		}
	}
	for (int i = 0; i < control->states[state].numTurnServos; ++i) {
		auto diff = setMuscleValue(control, control->states[state].turnServo[i], control->currentTurn);
		if (diff > maxDiff) {
			maxDiff = diff;
		}
	}
	return maxDiff;
}

void* runtimeMain(void* control) {
	struct timespec  sleepTime;
	sleepTime.tv_sec = 0;

	struct timespec ts;

	while(((TServoControl*)control)->runtime_state != RUNTIME_STATE_EXIT) {
		switch(((TServoControl*)control)->runtime_state) {
			case RUNTIME_STATE_IDLE:
				timespec_get(&ts, TIME_UTC);
				//printf("idle %ld.%09ld\n", (long)ts.tv_sec, ts.tv_nsec);
				((TServoControl*)control)->runtime_pause = RUNTIME_DEFAULT_IDLE_PAUSE;
				break;
			case RUNTIME_STATE_MOVE_FORWARD: {
				int newState = ((TServoControl*)control)->currentState + 1;
				if (newState >= ((TServoControl*)control)->numStates) {
					newState = 0;
				}
				((TServoControl*)control)->currentState = newState;
				float move = updateState((TServoControl*)control);
				((TServoControl*)control)->runtime_pause = move * ((TServoControl*)control)->pause_rate;
				printf("move to state: %d move=%f sleep=%f stepsize=%f\n", newState, move, ((TServoControl*)control)->runtime_pause,((TServoControl*)control)->stepSize);
				break;
			}
/*
			case RUNTIME_STATE_MOVE_BACKWARD: {
				int newStep = ((TServoControl*)control)->currentStep - 1;
				if (newStep < 0) {
					newStep = ((TServoControl*)control)->numSteps - 1;
				}
				float move = moveToStep((TServoControl*)control, newStep);
				((TServoControl*)control)->runtime_pause = move * ((TServoControl*)control)->pause_rate;
				printf("move to step: %d move=%f sleep=%f\n", newStep, move, ((TServoControl*)control)->runtime_pause);
				break;
			}
*/
			default:
				printf("Incorrect stae: %d, switching to idle\n", ((TServoControl*)control)->runtime_state);
				((TServoControl*)control)->runtime_state = RUNTIME_STATE_IDLE;
		}
		sleepTime.tv_nsec = (long)(((TServoControl*)control)->runtime_pause * 1000000000L);
		int r =clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepTime, NULL);
		if (r < 0) {
			printf("sec=%d, nsec=%ld r=%d errno=%d\n", (int)sleepTime.tv_sec, sleepTime.tv_nsec, r, (int)errno);
		}
	}
	return NULL;
}

int initRuntime(TServoControl* control) {
	if (control->runtime_thread != NULL) {
		fprintf(stderr, "Runtime is active, nee to stop first\n");
		stopRuntime(control);
	}
	control->runtime_thread = new pthread_t;
	control->runtime_state = RUNTIME_STATE_IDLE;
	control->runtime_pause = RUNTIME_DEFAULT_IDLE_PAUSE;
	control->pause_rate = DEFAULT_PAUSE_RATE;
	if(pthread_create((pthread_t*)control->runtime_thread, NULL, runtimeMain, control)) {
		fprintf(stderr, "Error creating thread\n");
		delete (pthread_t*)control->runtime_thread;
		control->runtime_thread = NULL;
		return -1;
	}
	return 0;
}

int stopRuntime(TServoControl* control) {
	if (control->runtime_thread != NULL) {
		control->runtime_state = RUNTIME_STATE_EXIT;
		if(pthread_join(*(pthread_t*)control->runtime_thread, NULL)) {
			fprintf(stderr, "Error joining thread\n");
			return -1;
		}
		delete (pthread_t*)control->runtime_thread;
		control->runtime_thread = NULL;
		return 0;
	} else {
		fprintf(stderr, "No runtime to stop\n");
		return -2;
	}
}

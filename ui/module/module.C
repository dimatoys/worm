#include "module.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

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
	printf("updateServoValue: %d:%d\n", servo, value);

#ifdef RASPBERRY
	((PCA9685*)pca9685)->Write(CHANNEL(servo), VALUE(value));
#endif
}

void setServoValue(TServoControl* control, int servo, int value) {
	TServo* srec = &control->servos[servo];
	srec->value = value;
	updateServoValue(control->hardware, servo, value);
}

int setServoAngle(TServoControl* control, int servo, float angle) {
	TServo* srec = &control->servos[servo];
	srec->angle = angle;
	srec->value = srec->mid + (angle >= 0 ? srec->max - srec->mid : srec->mid - srec->min) * angle / 90.0;
	updateServoValue(control->hardware, servo, srec->value);
	return srec->value;
}

float moveToStep(TServoControl* control, int step) {
	if (step >= 0 && step < MAX_STEPS) {
		float maxDiff = 0;
		for (int i = 0; i< NUM_MOTORS; ++i) {
			float newAngle = control->steps[step].motors[i];
			float absDiff = fabs(newAngle - control->servos[i].angle);
			if (absDiff > maxDiff) {
				maxDiff = absDiff;
			}
			setServoAngle(control, i, newAngle);
		}
		control->currentStep = step;
		return maxDiff;
	} else {
		return -1;
	}
}

void* runtimeMain(void* control) {
	struct timespec  sleepTime;
	struct timespec returnTime;
	sleepTime.tv_sec = 0;
	while(((TServoControl*)control)->runtime_state != RUNTIME_STATE_EXIT) {
		switch(((TServoControl*)control)->runtime_state) {
			case RUNTIME_STATE_IDLE:
				printf("idle %ld\n", time(NULL));
				break;
			case RUNTIME_STATE_MOVE_FORWARD: {
				int newStep = ((TServoControl*)control)->currentStep + 1;
				if (newStep >= ((TServoControl*)control)->numSteps) {
					newStep = 0;
				}
				float move = moveToStep((TServoControl*)control, newStep);
				((TServoControl*)control)->runtime_pause = move * ((TServoControl*)control)->pause_rate;
				printf("move to step: %d move=%f sleep=%f\n", newStep, move, ((TServoControl*)control)->runtime_pause);
				break;
			}
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
		}
		sleepTime.tv_nsec = (long)(((TServoControl*)control)->runtime_pause * 1000000000);
		nanosleep(&sleepTime, &returnTime);
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


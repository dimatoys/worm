#include "module.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include <cerrno>
#include <unistd.h>
#include <sys/select.h>

#ifdef RASPBERRY

#define TOF
//#define VL



#include <unistd.h>
#include "bcm2835.h"
#include "pca9685.h"

#ifdef TOF
#include "tof.h"
#endif


#ifdef VL
#include "VL53L0X.hpp"
#endif

#endif


int initServos(TServoControl* control) {
	
	control->runtime_thread = NULL;
	control->sensor = NULL;
	
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
	auto cycle = control->currentCycle;
	auto state = control->currentState;
	
	printf("updateState: cycle=%d state=%d\n", (int)cycle, (int)state);
	
	switch(cycle) {
	case CYCLE_MOVEMENT:
		if (state < 0) {
			state = control->numStates - 1;
			control->currentState = state;
		} else {
			if (state >= control->numStates) {
				state = 0;
				control->currentState = state;
			}
		}
		for (int i = 0; i < control->states[state].num; ++i) {
			auto diff = setMuscleValue(control, control->states[state].servoPoly[i], control->stepSize);
			if (diff > maxDiff) {
				maxDiff = diff;
			}
		}
		break;
	case CYCLE_TURN:
		if (state < 0) {
			state = control->numTurnStates - 1;
			control->currentState = state;
		} else {
			if (state >= control->numTurnStates) {
				state = 0;
				control->currentState = state;
			}
		}
		for (int i = 0; i < control->turnStates[state].num; ++i) {
			auto diff = setMuscleValue(control, control->turnStates[state].servoPoly[i], control->currentTurn);
			if (diff > maxDiff) {
				maxDiff = diff;
			}
		}
		break;
	case CYCLE_MOUTH:
		control->currentState = 0;
		maxDiff = setServoAngle(control, 6, control->submitMouthState == 1 ? 90 : -90);
		
		break;
	}
	return maxDiff;
}

void setTurn(TServoControl* control, float angle) {
	control->nextTurn = angle;
}

void pauseSec(float sec) {
	struct timespec  sleepTime;
	sleepTime.tv_sec = (time_t)sec;
	sleepTime.tv_nsec = (long)((sec - sleepTime.tv_sec) * 1000000000L);
	int r =clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepTime, NULL);
	if (r < 0) {
		printf("sec=%d, nsec=%ld r=%d errno=%d\n", (int)sleepTime.tv_sec, sleepTime.tv_nsec, r, (int)errno);
	}
}

void pauseAngle(TServoControl* control, float angle) {
	pauseSec(angle * control->pause_rate);
}

void* runtimeMain(void* control) {
	//struct timespec  sleepTime;
	//sleepTime.tv_sec = 0;

	struct timespec ts;

	while(((TServoControl*)control)->runtime_state != RUNTIME_STATE_EXIT) {
		switch(((TServoControl*)control)->runtime_state) {
			case RUNTIME_STATE_IDLE:
				timespec_get(&ts, TIME_UTC);
				//printf("idle %ld.%09ld\n", (long)ts.tv_sec, ts.tv_nsec);
				((TServoControl*)control)->runtime_pause = RUNTIME_DEFAULT_IDLE_PAUSE;
				break;
			case RUNTIME_STATE_MOVE_FORWARD: {				
				if (((TServoControl*)control)->currentState == 0) {
					if (((TServoControl*)control)->mouthState != ((TServoControl*)control)->submitMouthState) {
						((TServoControl*)control)->currentCycle = CYCLE_MOUTH;
						((TServoControl*)control)->mouthState = ((TServoControl*)control)->submitMouthState;
					} else {
						if (((TServoControl*)control)->nextTurn != 0) {
							((TServoControl*)control)->currentCycle = CYCLE_TURN;
							((TServoControl*)control)->currentTurn = ((TServoControl*)control)->nextTurn;
						} else {
							((TServoControl*)control)->currentCycle = CYCLE_MOVEMENT;
						}
					}
				}
				
				((TServoControl*)control)->currentState++;
				float move = updateState((TServoControl*)control);
				((TServoControl*)control)->runtime_pause = move * ((TServoControl*)control)->pause_rate;
				//printf("move to state: %d move=%f sleep=%f stepsize=%f\n", newState, move, ((TServoControl*)control)->runtime_pause,((TServoControl*)control)->stepSize);
				break;
			}
			case RUNTIME_STATE_MOVE_BACKWARD: {
				int newState = ((TServoControl*)control)->currentState - 1;
				if (newState < 0) {
					newState = ((TServoControl*)control)->numStates - 1;
				}
				((TServoControl*)control)->currentState = newState;
				float move = updateState((TServoControl*)control);
				((TServoControl*)control)->runtime_pause = move * ((TServoControl*)control)->pause_rate;
				//printf("move to step: %d move=%f sleep=%f\n", newStep, move, ((TServoControl*)control)->runtime_pause);
				break;
			}

			default:
				printf("Incorrect stae: %d, switching to idle\n", ((TServoControl*)control)->runtime_state);
				((TServoControl*)control)->runtime_state = RUNTIME_STATE_IDLE;
		}
		//sleepTime.tv_nsec = (long)(((TServoControl*)control)->runtime_pause * 1000000000L);
		//int r =clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepTime, NULL);
		//if (r < 0) {
		//	printf("sec=%d, nsec=%ld r=%d errno=%d\n", (int)sleepTime.tv_sec, sleepTime.tv_nsec, r, (int)errno);
		//}
		pauseSec(((TServoControl*)control)->runtime_pause);
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

int initRangeFinder(TServoControl* control) {
#ifdef RASPBERRY

#ifdef TOF
  	int bLongRangeMode = 0;
 
	int status = tofInit(1, 0x29, bLongRangeMode);
	if (status != 1) {
		return -1; // problem - quit
	}
	printf("VL53L0X device successfully opened.\n");
#endif

#ifdef VL
	VL53L0X* sensor = new VL53L0X();
	sensor->initialize();
	//sensor->setTimeout(200);
/*
	#ifdef LONG_RANGE
	
	// Lower the return signal rate limit (default is 0.25 MCPS)
	sensor->setSignalRateLimit(0.1);

	// Increase laser pulse periods (defaults are 14 and 10 PCLKs)
	sensor->setVcselPulsePeriod(VcselPeriodPreRange, 18);
	sensor>setVcselPulsePeriod(VcselPeriodFinalRange, 14);

	#endif

	#if defined HIGH_SPEED

	// Reduce timing budget to 20 ms (default is about 33 ms)
	sensor->setMeasurementTimingBudget(20000);

	#elif defined HIGH_ACCURACY

	// Increase timing budget to 200 ms
	sensor->setMeasurementTimingBudget(200000);

	#endif
*/
	control->sensor = sensor;
#endif

#endif

	control->R = 262;
	control->S = 80;

	return 0;
}


int readDistance(TServoControl* control) {
#ifdef RASPBERRY
	
#ifdef TOF
	return tofReadDistance();
#endif

#ifdef VL
	uint16_t distance = ((VL53L0X*)control->sensor)->readRangeSingleMillimeters();
	return ((VL53L0X*)control->sensor)->timeoutOccurred() ? -1 : distance;
#endif

#else
	return 0;
#endif
}

void HScan2(TServoControl* control, float angle0, float angle1, float from2, float to2, float step2, float prePause, float pauseMove) {

	auto current = from2;
	
	auto pause1 = setServoAngle(control, 1, angle1);
	auto pause0 = setServoAngle(control, 0, angle0);
	
	pauseAngle(control, pause1 > pause0 ? pause1 : pause0);
	pauseAngle(control, setServoAngle(control, 2, current));

	pauseSec(prePause);
	control->num_measurements = 0;
	
	while (current <= to2) {
		control->measurements[control->num_measurements++] = readDistance(control);

		current += step2;
		pauseAngle(control, setServoAngle(control, 2, current));
		
		if (pauseMove != 0) {
			pauseSec(pauseMove);
		}
	}

	current = from2;
	int i = 0;
	while (current <= to2) {
		double a = current * M_PI / 180.0;
		auto sa = sin(a);
		auto ca = cos(a);
		auto d1 = control->measurements[i];
		
		auto m = &control->sobj[i];
		m->x = (float)((control->R + d1) * sa - control->S * ca);
		m->y = (float)((control->R + d1) * ca + control->S * sa);

		++i;
		current += step2;
	}

	pauseAngle(control, setServoAngle(control, 2, 0));
	pauseAngle(control, setServoAngle(control, 0, 0));
	pauseAngle(control, setServoAngle(control, 1, -3));
}


/*
-15/10/-10/10/0.5/0.2/0.1

-15/10/-10/10/1/0.2/0.1


* */

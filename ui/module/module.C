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

float ks2[21][20] = {
{},
{},
{},	
{ 0.5,        -1,           0.5},
{ 0.25,       -0.25,       -0.25,        0.25},
{ 0.14285714, -0.07142857, -0.14285714, -0.07142857, 0.14285714},
{ 0.08928571, -0.01785714, -0.07142857, -0.07142857, -0.01785714,  0.08928571},
{ 0.05952380,  8.67361738e-18, -3.57142857e-02, -4.76190476e-02, -3.57142857e-02,  1.11022302e-16,  5.95238095e-02},
{ 0.04166667,  0.00595238, -0.01785714, -0.0297619,  -0.0297619,  -0.01785714,  0.00595238,  0.04166667},
{ 0.03030303,  0.00757576, -0.00865801, -0.01839827, -0.02164502, -0.01839827, -0.00865801,  0.00757576,  0.03030303},
{ 0.02272727,  0.00757576, -0.00378788, -0.01136364, -0.01515152, -0.01515152, -0.01136364, -0.00378788,  0.00757576,  0.02272727},
{ 0.01748252,  0.00699301, -0.0011655,  -0.00699301, -0.01048951, -0.01165501, -0.01048951, -0.00699301, -0.0011655,   0.00699301,  0.01748252},
{ 0.01373626,  0.00624376,  0.00024975, -0.00424575, -0.00724276, -0.00874126, -0.00874126, -0.00724276, -0.00424575,  0.00024975,  0.00624376,  0.01373626},
{ 0.01098901,  0.00549451,  0.000999,   -0.0024975,  -0.004995,   -0.00649351, -0.00699301, -0.00649351, -0.004995,   -0.0024975,   0.000999,    0.00549451,  0.01098901},
{ 0.00892857,  0.00480769,  0.00137363, -0.00137363, -0.00343407, -0.00480769, -0.00549451, -0.00549451, -0.00480769, -0.00343407, -0.00137363,  0.00137363,  0.00480769,  0.00892857},
{ 0.00735294,  0.00420168,  0.00153523, -0.00064641, -0.00234324, -0.00355527, -0.00428248, -0.00452489, -0.00428248, -0.00355527, -0.00234324, -0.00064641,  0.00153523,  0.00420168,  0.00735294},
{ 0.00612745,  0.00367647,  0.00157563, -0.00017507, -0.00157563, -0.00262605, -0.00332633, -0.00367647, -0.00367647, -0.00332633, -0.00262605, -0.00157563, -0.00017507,  0.00157563,  0.00367647,  0.00612745},
{ 0.00515996,  0.00322497,  0.00154799,  0.000129,   -0.00103199, -0.00193498, -0.00257998, -0.00296698, -0.00309598, -0.00296698, -0.00257998, -0.00193498, -0.00103199,  0.000129,    0.00154799,  0.00322497,  0.00515996},
{ 0.00438596,  0.00283798,  0.00148349,  0.0003225,  -0.00064499, -0.00141899, -0.00199948, -0.00238648, -0.00257998, -0.00257998, -0.00238648, -0.00199948, -0.00141899, -0.00064499,  0.0003225,   0.00148349,  0.00283798,  0.00438596},
{ 0.0037594,   0.00250627,  0.00140056,  0.00044228, -0.00036857, -0.00103199, -0.00154799, -0.00191656, -0.0021377,  -0.00221141, -0.0021377,  -0.00191656, -0.00154799, -0.00103199, -0.00036857,  0.00044228,  0.00140056,  0.00250627,  0.0037594 },
{ 0.00324675,  0.00222146,  0.00131009,  0.00051265, -0.00017088, -0.00074049, -0.00119617, -0.00153794, -0.00176578, -0.0018797,  -0.0018797,  -0.00176578, -0.00153794, -0.00119617, -0.00074049, -0.00017088,  0.00051265,  0.00131009,  0.00222146,  0.00324675}
};

void countS2(TServoControl* control, float from, float step) {
	float* a = ks2[control->scan_d];
	int min_conf_distance = 10000;
	control->scan_found_object = 0;
	for (int i = 0; i < control->num_measurements - control->scan_d; ++i) {
		float s2 = 0;
		auto angle_idx = i + control->scan_d / 2;
		for (int j = 0; j < control->scan_d; ++j) {
			s2 += a[j] * control->measurements[i + j];
		}
		if (s2 >= control->scan_min_conf) {
			if (control->measurements[angle_idx] < min_conf_distance) {
				min_conf_distance = control->measurements[angle_idx];
				control->scan_decidion = from + angle_idx * step;
				control->scan_found_object = 1;				
			}
		}
		control->measurements2[i] = s2;
	}
	printf("Decidion: d=%d angle=%f\n", min_conf_distance, (double)control->scan_decidion);
}

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
	if (control->poly[i].flags == FLAGS_ABS) {
		value = fabs(value);
	}
	float sv = 0;
	for (int j = control->poly[i].s - 1; j >= 0 ; --j) {
		sv = sv * value + control->poly[i].k[j];
	}
	float diff = setServoAngle(control, control->poly[i].servo, sv);
	//printf("set %d angle %f diff=%f\n", control->poly[i].servo, sv, diff);
	return diff;
}

int stateInRange(TServoControl* control, int numStates) {
	auto state = control->currentState;
	if (state < 0) {
		state = numStates - 1;
		control->currentState = state;
	} else {
		if (state >= numStates) {
			state = 0;
			control->currentState = state;
		}
	}
	return state;
}

float updateState(TServoControl* control) {
	float maxDiff = 0;
	auto cycle = control->currentCycle;
	
	switch(cycle) {
	case CYCLE_MOVEMENT: {
			auto state = stateInRange(control, control->numStates);
			for (int i = 0; i < control->states[state].num; ++i) {
				auto diff = setMuscleValue(control, control->states[state].servoPoly[i], control->stepSize);
				if (diff > maxDiff) {
					maxDiff = diff;
				}
			}
		}
		break;
	case CYCLE_TURN: {
			auto state = stateInRange(control, control->numTurnStates);
			for (int i = 0; i < control->turnStates[state].num; ++i) {
				auto diff = setMuscleValue(control, control->turnStates[state].servoPoly[i], control->currentTurn);
				if (diff > maxDiff) {
					maxDiff = diff;
				}
			}
		}
		break;
	case CYCLE_MOUTH:
		stateInRange(control, 1);
		maxDiff = setServoAngle(control, 6, control->submitMouthState == 1 ? 90 : -90);
		
		break;
	case CYCLE_FULL_SCAN: {
			auto numTurns = (control->scan_to2 - control->scan_from2) / control->scan_step2;
			auto state = stateInRange(control, numTurns + 3);
			switch(state) {
				case 0: {
					auto diff1 = setServoAngle(control, 0, 0);
					auto diff2 = setServoAngle(control, 1, -3);
					maxDiff = diff1 > diff2 ? diff1 : diff2;
					break;
				}
			
				case 1: {
					control->num_measurements = 0;
					auto diff1 = setServoAngle(control, 1, control->scan_angle_low_1);
					auto diff2 = setServoAngle(control, 0, control->scan_angle_low_0);
					maxDiff = diff1 > diff2 ? diff1 : diff2;
					break;
				}
			
				case 2: {
					maxDiff = setServoAngle(control, 2, control->scan_from2);
					maxDiff += control->scan_pre_pause / control->pause_rate;
					break;
				}
			
				default: {
					control->measurements[control->num_measurements++] = readDistance(control);
					if (state == numTurns + 2) {
						maxDiff = setServoAngle(control, 2, 0);
						countS2(control, control->scan_from2, control->scan_step2);
					} else {
						auto new_turn = control->scan_from2 + (state - 2) * control->scan_step2;
						maxDiff = setServoAngle(control, 2, new_turn);
						maxDiff += control->scan_pause_move / control->pause_rate;
					}
					break;
				}
			}
		}
		break;
	case CYCLE_IDLE:
		stateInRange(control, 1);
		control->runtime_state = RUNTIME_STATE_IDLE;
		break;
	case CYCLE_DISTANCE:
		stateInRange(control, 1);
		control->scan_distance = readDistance(control);
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

int decide(TServoControl* control) {
	if (control->scanFlag == 1) {
		control->scanFlag = 2;
		control->scan_decidion_progress = 0;
		return CYCLE_FULL_SCAN;
	} else {
		if (control->scanFlag == 2) {
			if (control->scan_found_object == 1) {
				auto left = fabs(control->scan_decidion - control->scan_decidion_progress);
				float turn;
				if (left > 8) {
					turn = 8;
				} else {
					if (left > 0.45) {
						turn = left;
					} else {
						printf("Reached: decidion=%f progress=%f left=%f\n",
							(double)control->scan_decidion, (double)control->scan_decidion_progress, (double)left);
						control->scanFlag = 0;
						return CYCLE_IDLE;
					}
				}
				control->currentTurn = (-0.06606535 * turn + 2.22201163) * turn + 3.32568195;
				if (control->scan_decidion > 0) {
					control->scan_decidion_progress += turn;
				} else {
					control->currentTurn = -control->currentTurn;
					control->scan_decidion_progress -= turn;
				}
				printf("Turn: decidion=%f new_progress=%f turn=%f c_turn=%f\n",
							(double)control->scan_decidion, (double)control->scan_decidion_progress, (double)turn, (double)control->currentTurn);
				return CYCLE_TURN;
			} else {
				printf("Object not found\n");
				control->scanFlag = 0;
				return CYCLE_IDLE;
			}
		}
	}

	if (control->mouthState != control->submitMouthState) {
		control->mouthState = control->submitMouthState;
		return CYCLE_MOUTH;
	} else {
		if (control->nextTurn != 0) {
			control->currentTurn = ((TServoControl*)control)->nextTurn;
			return CYCLE_TURN;
		} else {
			return CYCLE_MOVEMENT;
		}
	}
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
					((TServoControl*)control)->currentCycle = decide((TServoControl*)control);
					printf("New cyrcle: %d pause_rate=%f\n", ((TServoControl*)control)->currentCycle, (double)((TServoControl*)control)->pause_rate);
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
/*
	for (int i = 0; i < 15;++i) {
		printf("%f\n", ks2[15][i]);
	}
*/
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

	pauseAngle(control, setServoAngle(control, 2, 0));
	pauseAngle(control, setServoAngle(control, 0, 0));
	pauseAngle(control, setServoAngle(control, 1, -3));
}

void HScan3(TServoControl* control) {
	auto pause1 = setServoAngle(control, 1, control->scan_angle_low_1);
	auto pause0 = setServoAngle(control, 0, control->scan_angle_low_0);
	pauseAngle(control, pause1 > pause0 ? pause1 : pause0);

	pauseAngle(control, setServoAngle(control, 2, control->scan_from2));

	pauseSec(control->scan_pre_pause);

	control->num_measurements = 0;
	auto current = control->scan_from2;
	while (current <= control->scan_to2) {
		control->measurements[control->num_measurements++] = readDistance(control);

		current += control->scan_step2;
		pauseAngle(control, setServoAngle(control, 2, current));
		
		if (control->scan_pause_move != 0) {
			pauseSec(control->scan_pause_move);
		}
	}

	pause0 = setServoAngle(control, 0, control->scan_angle_high_0);
	pause1 = setServoAngle(control, 1, control->scan_angle_high_1);
	auto pause2 = setServoAngle(control, 2, control->scan_from2);
	pauseAngle(control, pause1 > pause0 ? (pause1 > pause2 ? pause1 : pause2) : (pause0 > pause2 ? pause0 : pause2));

	pauseSec(control->scan_pre_pause);

	int i = 0;
	current = control->scan_from2;
	while (current <= control->scan_to2) {
		control->measurements2[i++] = readDistance(control);

		current += control->scan_step2;
		pauseAngle(control, setServoAngle(control, 2, current));
		
		if (control->scan_pause_move != 0) {
			pauseSec(control->scan_pause_move);
		}
	}

	pauseAngle(control, setServoAngle(control, 2, 0));
	pauseAngle(control, setServoAngle(control, 0, 0));
	pauseAngle(control, setServoAngle(control, 1, -3));
}

void HScan4(TServoControl* control) {
	auto pause1 = setServoAngle(control, 1, control->scan_angle_low_1);
	auto pause0 = setServoAngle(control, 0, control->scan_angle_low_0);
	pauseAngle(control, pause1 > pause0 ? pause1 : pause0);

	pauseAngle(control, setServoAngle(control, 2, control->scan_from2));

	pauseSec(control->scan_pre_pause);

	control->num_measurements = 0;
	auto current = control->scan_from2;
	while (current <= control->scan_to2) {
		control->measurements[control->num_measurements++] = readDistance(control);

		current += control->scan_step2;
		pauseAngle(control, setServoAngle(control, 2, current));
		
		if (control->scan_pause_move != 0) {
			pauseSec(control->scan_pause_move);
		}
	}

	countS2(control, control->scan_from2, control->scan_step2);

	pauseAngle(control, setServoAngle(control, 2, 0));
	pauseAngle(control, setServoAngle(control, 0, 0));
	pauseAngle(control, setServoAngle(control, 1, -3));
	
}

/*

5
215
			0.21						23.89

10
* 
29/30

27

**30


			1.53						6.56


12 47/47   3.83 degrees per cycle        3.1333333333333333 per angle
* 
15
* 
29/29       6.21 degrees per cycle		2.4166666666666665

**18



20
* 
17/18       10.21 degrees per cycle     1.9444444444444442


9
34
*
33
* 
27
* 
8
30
* 
*
31
**45


7

34

6
* 
55
* 
5
130


10.0.0.232 - - [27/May/2020 04:16:57] "GET /worm/startscan HTTP/1.1" 200 -
New cyrcle: 3 pause_rate=0.004000
Decidion: d=245 angle=-16.000000
Turn: decidion=-16.000000 new_progress=-8.000000 turn=8.000000 c_turn=-16.873592
New cyrcle: 1 pause_rate=0.004000
Turn: decidion=-16.000000 new_progress=-16.000000 turn=8.000000 c_turn=-16.873592
New cyrcle: 1 pause_rate=0.004000
Reached: decidion=-16.000000 progress=-16.000000 left=0.000000
New cyrcle: 4 pause_rate=0.004000


10.0.0.232 - - [27/May/2020 04:17:22] "GET /worm/startscan HTTP/1.1" 200 -
New cyrcle: 3 pause_rate=0.004000
Decidion: d=323 angle=-17.000000
Turn: decidion=-17.000000 new_progress=-8.000000 turn=8.000000 c_turn=-16.873592
New cyrcle: 1 pause_rate=0.004000
Turn: decidion=-17.000000 new_progress=-16.000000 turn=8.000000 c_turn=-16.873592
New cyrcle: 1 pause_rate=0.004000
Turn: decidion=-17.000000 new_progress=-17.000000 turn=1.000000 c_turn=-5.481628
New cyrcle: 1 pause_rate=0.004000
Reached: decidion=-17.000000 progress=-17.000000 left=0.000000
New cyrcle: 4 pause_rate=0.004000

* */

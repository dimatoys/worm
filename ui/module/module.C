#include "module.h"

#include <stdio.h>

#ifdef RASPBERRY
#include <unistd.h>
#include "bcm2835.h"
#include "pca9685.h"
#endif

int initServos(TServoControl* control) {
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

void updateServoValue(PCA9685* pca9685, int servo, int value) {
	printf("updateServoValue: %d:%d\n", servo, value);

#ifdef RASPBERRY
	pca9685->Write(CHANNEL(servo), VALUE(value));
#endif
}

void setServoValue(TServoControl* control, int servo, int value) {
	TServo* srec = &control->servos[servo];
	srec->value = value;
	updateServoValue((PCA9685*)control->hardware, servo, value);
}

int setServoAngle(TServoControl* control, int servo, float angle) {
	TServo* srec = &control->servos[servo];
	srec->angle = angle;
	srec->value = srec->mid + (angle >= 0 ? srec->max - srec->mid : srec->mid - srec->min) * angle / 90.0;
	updateServoValue((PCA9685*)control->hardware, servo, srec->value);
	return srec->value;
}

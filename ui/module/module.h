extern "C" {

struct TServo {
	int		min;
	int		mid;
	int		max;
	int		value;
	float	angle;
};

struct TServoControl {
	TServo	servos[16];
	void*	hardware;
};

int initServos(TServoControl* control);
void setServoValue(TServoControl* control, int servo, int value);
int setServoAngle(TServoControl* control, int servo, float angle);

}


class ServoMotor(object):
	def __init__(self, logger, params, prefix):
		self.Logger = logger
		self.prefix = prefix
		
		self.Min = int(params[prefix + ".min"])
		self.Mid = int(params[prefix + ".mid"])
		self.Max = int(params[prefix + ".max"])

		self.Angle = 0

		self.MOUTH_MOTOR = 6

	def getValue(self):
		if self.Angle >= 90:
			return int(self.Mid + (self.Max - self.Mid) * self.Angle / 90.0)
		else:
			return int(self.Mid + (self.Mid - self.Min) * self.Angle / 90.0)

	def setAngle(self, angle):
		self.Angle = angle
		value = self.getValue()
		self.setValue(value)
		return value

	def setValue(self, value):
		self.Logger.debug("set %s" % (value))

	def setParams(self, vmin, vmid, vmax):
		self.Min = int(vmin)
		self.Mid = int(vmid)
		self.Max = int(vmax)

	def exportParams(self, params):
		params[self.prefix + ".min"] = self.Min
		params[self.prefix + ".mid"] = self.Mid
		params[self.prefix + ".max"] = self.Max

class HardwareStub(object):
	def __init__(self, logger, config_file):
		self.Logger = logger
		self.ConfigFile = config_file
		
		self.loadParams()
		
		self.Servos = []
		for s in range(100):
			prefix = "hardware.servos.%d" % s
			if prefix + ".min" in self.params:
				self.Servos.append(ServoMotor(self.Logger, self.params, prefix))

	def loadParams(self):
		self.params = dict(line.strip().split('=') for line in open(self.ConfigFile))

	def saveParams(self):
		with open(self.ConfigFile, "w") as f:
			keys = self.params.keys()
			keys.sort()
			for k in keys:
				f.write("%s=%s\n" % (k,str(self.params[k])))

	def getServos(self):
		result = []
		for i in range(len(self.Servos)):
			servo = self.Servos[i]
			result.append({"channel": i,
			               "min": servo.Min,
			               "mid": servo.Mid,
			               "max": servo.Max,
			               "value":servo.getValue()})
		return result

	def setServoValue(self, servo, value):
		self.Servos[int(servo)].setValue(int(value))

	def setServoAngle(self, servo, angle):
		return self.Servos[int(servo)].setAngle(float(angle))

	def setServoParams(self, servo, vmin, vmid, vmax):
		sr = self.Servos[int(servo)]
		sr.setParams(vmin, vmid, vmax)
		sr.exportParams(self.params)
		self.saveParams()

	def getWormSteps(self):
		steps = []
		step = 0
		while True:
			stepProp = "worm.steps.%d." % step
			if stepProp + "0" in self.params:
				steps.append([self.params[stepProp + "0"],
							  self.params[stepProp + "1"],
							  self.params[stepProp + "2"],
							  self.params[stepProp + "3"]])
			else:
				break
			step += 1
		return steps

	def wormMoveToStep(self, step):
		angles = []
		for i in range(4):
			angle = self.params["worm.steps.%s.%d" % (step, i)]
			angles.append(angle)
			self.setServoAngle(i, angle)
		return angles

	def wormSetMouthState(self, state):
		self.setServoAngle(self.MOUTH_MOTOR, [-90,90][state])

	def wormControl(id, value):
		self.Logger.debug("control %s: %s" % (id, value))

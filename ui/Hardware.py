from ctypes import *
from HardwareStub import HardwareStub

class TServo(Structure):
	_fields_ = [("min", c_int),
				("mid", c_int),
				("max", c_int),
				("value", c_int),
				("angle", c_float)]

	def setParams(self, vmin, vmid, vmax):
		self.min = int(vmin)
		self.mid = int(vmid)
		self.max = int(vmax)

class TWormStep(Structure):
	_fields_ = [("motors", c_float * 4)]

class TServoControl(Structure):
	_fields_ = [("hardware", c_void_p),
				("runtime_thread", c_void_p),
				("runtime_state", c_int),
				("runtime_pause", c_float),
				("servos", TServo * 16),
				("numSteps", c_int),
				("currentStep", c_int),
				("pause_rate", c_float),
				("steps", TWormStep * 50)]

	def __init__(self, module, params):
		self.module = module
		self.module.initServos.argtypes = [POINTER(TServoControl)]
		self.module.initServos.restype  = c_int
		self.module.setServoValue.argtypes = [POINTER(TServoControl), c_int, c_int]
		self.module.setServoAngle.argtypes = [POINTER(TServoControl), c_int, c_float]
		self.module.setServoAngle.restype  = c_int
		self.module.moveToStep.argtypes = [POINTER(TServoControl), c_int]
		self.module.moveToStep.restype  = c_float
		self.module.initRuntime.argtypes = [POINTER(TServoControl)]
		self.module.initRuntime.restype  = c_int
		self.module.stopRuntime.argtypes = [POINTER(TServoControl)]
		self.module.stopRuntime.restype  = c_int
		self.module.wormControl.argtypes = [POINTER(TServoControl), c_int, c_float]
		self.module.wormControl.restype  = c_int


		self.active = []
		for s in range(16):
			param = "hardware.servos.%d" % s
			if param + ".min" in params:
				self.servos[s].setParams(params[param + ".min"],
				                         params[param + ".mid"],
				                         params[param + ".max"])
				self.active.append(s)

		step = 0
		while True:
			stepProp = "worm.steps.%d." % step
			if stepProp + "0" in params:
				for m in range(4):
					self.steps[step].motors[m] = float(params[stepProp + str(m)])
			else:
				break
			step += 1
		self.numSteps = step

		if "worm.pauseRate" in params:
			self.pause_rate = float(params["worm.pauseRate"])

		status = self.module.initServos(byref(self))
		if status != 0:
			raise Exception("servo status=%d" % status)

		"""
		status = self.module.initRuntime(byref(self))
		if status != 0:
			raise Exception("runtime status=%d" % status)
		"""

	def setValue(self, servo, value):
		self.module.setServoValue(byref(self), int(servo), int(value))

	def setAngle(self, servo, angle):
		servo = int(servo)
		self.module.setServoAngle(byref(self), servo, float(angle))
		return self.servos[servo].value

	def setParams(self, servo, vmin, vmid, vmax):
		self.servos[int(servo)].setParams(vmin, vmid, vmax)

	def exportParams(self, params):
		for s in self.active:
			param = "hardware.servos.%d" % s
			params[param + ".min"] = self.servos[s].min
			params[param + ".mid"] = self.servos[s].mid
			params[param + ".max"] = self.servos[s].max
		params["worm.pauseRate"] = self.pause_rate

	def movetoStep(self, step):
		return self.module.moveToStep(byref(self), int(step))

	def setState(self, state):
		self.runtime_state = int(state)

	def setRate(self, rate):
		self.pause_rate = float(rate)

	def wormControl(self, id, value):
		ids = {"step2": 1}
		if id in ids:
			return self.module.wormControl(byref(self), ids[id], float(value))
		else:
			return -1

class Hardware(HardwareStub):
	def __init__(self, logger, config_file):
		super(Hardware, self).__init__(logger, config_file)
		self.module = cdll.LoadLibrary('module/libhardwareModule.so')

		self.loadParams()

		self.ServoControl = TServoControl(self.module, self.params)

	def setServoValue(self, servo, value):
		self.ServoControl.setValue(servo, value)


	def setServoAngle(self, servo, angle):
		return self.ServoControl.setAngle(servo, angle)

	def setServoParams(self, servo, vmin, vmid, vmax):
		self.ServoControl.setParams(servo, vmin, vmid, vmax)
		self.ServoControl.exportParams(self.params)
		self.saveParams()

	def wormMoveToStep(self, step):
		return self.ServoControl.movetoStep(step)

	def wormStop(self):
		self.ServoControl.setState(1)

	def wormMoveForward(self):
		self.ServoControl.setState(2)

	def wormMoveBackward(self):
		self.ServoControl.setState(3)

	def wormSetPauseRate(self, rate):
		self.ServoControl.setRate(rate)

	def wormControl(self, id, value):
		return self.ServoControl.wormControl(id, value)


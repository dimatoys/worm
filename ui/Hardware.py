from ctypes import *
from HardwareStub import HardwareStub
import copy
import re

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

class TPoly4(Structure):
	_fields_ = [("servo", c_int),
	            ("s", c_int),
	            ("k", c_float * 4)]

	def load(self, params, prefix):
		servoKey = "%s.servo" % prefix
		if servoKey in params:
			self.servo = int(params[servoKey])
			for i in range(4):
				key = "%s.%d" % (prefix, i)
				if key in params:
					self.k[i] = float(params[key])
				else:
					self.s = i
					return
			self.s = 4

class TState(Structure):
	_fields_ = [("num", c_int),
				("servoPoly", c_int * 7)]

class TServoControl(Structure):
	_fields_ = [("hardware", c_void_p),
				("runtime_thread", c_void_p),
				("runtime_state", c_int),
				("runtime_pause", c_float),
				("servos", TServo * 16),
				("numSteps", c_int),
				("currentStep", c_int),
				("pause_rate", c_float),
				("states", TState * 30),
				("poly", TPoly4 * 50),
				("steps", TWormStep * 50)]

	def __init__(self, module, params, logger):
		self.Logger = logger
		
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

		self.NumPoly = 0
		self.NumStates = 0
		self.stateMap = {}
		pattern = re.compile("worm\\.state\\.(\w+)\\.")
		keys = list(params.keys())
		keys.sort()
		for key in keys:
			m = pattern.match(key)
			if m is not None:
				if m.group(1) not in self.stateMap:
					print(m.group(1))
					self.loadState(params, m.group(1))

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
		if id in self.stateMap:
			return self.module.wormControl(byref(self), self.stateMap[id]["id"], float(value))
		else:
			return -1

	def loadState(self, params, stateId):
		for stateMuscles in range(20):
			prefix = "worm.state.%s.poly.%c" % (stateId, ord('a') + stateMuscles)
			if prefix + ".servo" in params:
				self.poly[self.NumPoly].load(params, prefix)
				self.states[self.NumStates].servoPoly[stateMuscles] = self.NumPoly
				self.NumPoly += 1
			else:
				self.states[self.NumStates].num = stateMuscles
				self.stateMap[stateId] = {"id": self.NumStates,
										  "min": params["worm.state.%s.min" % stateId],
										  "max": params["worm.state.%s.max" % stateId],
										  "init": params["worm.state.%s.init" % stateId]}
				self.NumStates += 1
				break



class Hardware(HardwareStub):
	def __init__(self, logger, config_file):
		super(Hardware, self).__init__(logger, config_file)
		self.module = cdll.LoadLibrary('module/libhardwareModule.so')

		self.loadParams()

		self.ServoControl = TServoControl(self.module, self.params, logger)

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

	def wormGetStates(self):
		result = []
		for s,p in self.ServoControl.stateMap.items():
			np = copy.deepcopy(p)
			np["name"] = s
			result.append(np)
		result.sort(key=lambda x: x["id"])
		return result


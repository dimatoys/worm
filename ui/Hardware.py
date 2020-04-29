from ctypes import *
from HardwareStub import HardwareStub
import copy
import re

CYCLES = {"state":0, "turn":1}

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
				("pause_rate", c_float),
				("stepSize", c_float),
				("currentTurn", c_float),
				("nextTurn", c_float),
				("currentState", c_int),
				("currentCycle", c_int),
				("mouthState", c_int),
				("submitMouthState", c_int),
				("NumPoly", c_int),
				("NumStates", c_int),
				("states", TState * 30),
				("NumTurnStates", c_int),
				("turnStates", TState * 30),
				("poly", TPoly4 * 100)]

	def __init__(self, module, params, logger):
		self.Logger = logger
		
		self.module = module
		self.module.initServos.argtypes = [POINTER(TServoControl)]
		self.module.initServos.restype  = c_int
		self.module.setServoValue.argtypes = [POINTER(TServoControl), c_int, c_int]
		self.module.setServoAngle.argtypes = [POINTER(TServoControl), c_int, c_float]
		self.module.setServoAngle.restype  = c_float
		self.module.updateState.argtypes = [POINTER(TServoControl)]
		self.module.updateState.restype  = c_float
		self.module.setTurn.argtypes = [POINTER(TServoControl), c_float]

		self.module.initRuntime.argtypes = [POINTER(TServoControl)]
		self.module.initRuntime.restype  = c_int
		self.module.stopRuntime.argtypes = [POINTER(TServoControl)]
		self.module.stopRuntime.restype  = c_int


		self.active = []
		for s in range(16):
			param = "hardware.servos.%d" % s
			if param + ".min" in params:
				self.servos[s].setParams(params[param + ".min"],
				                         params[param + ".mid"],
				                         params[param + ".max"])
				self.active.append(s)

		if "worm.pauseRate" in params:
			self.pause_rate = float(params["worm.pauseRate"])

		self.currentCycle = CYCLES["state"]
		self.currentState = 0
		self.stepSize = 100
		self.currentTurn = 0
		self.nextTurn = 0
		self.mouthState = 0
		self.submitMouthState = 0

		self.stateMap = {}
		self.NumPoly = 0
		self.NumStates = 0
		self.NumTurnStates = 0

		for cycle in CYCLES.keys():
			self.loadStates(params,cycle)

		status = self.module.initServos(byref(self))
		if status != 0:
			raise Exception("servo status=%d" % status)

		status = self.module.initRuntime(byref(self))
		if status != 0:
			raise Exception("runtime status=%d" % status)

	def setValue(self, servo, value):
		self.module.setServoValue(byref(self), int(servo), int(value))

	def setAngle(self, servo, angle):
		servo = int(servo)
		self.module.setServoAngle(byref(self), servo, float(angle))
		return self.servos[servo].value

	def setParams(self, servo, vmin, vmid, vmax):
		self.servos[int(servo)].setParams(vmin, vmid, vmax)

	def getServos(self):
		result = []
		for s in range(len(self.active)):
			result.append({"channel": s,
						   "min": self.servos[s].min,
						   "mid": self.servos[s].mid,
						   "max": self.servos[s].max,
						   "value": self.servos[s].value,
						   "angle": self.servos[s].angle})
		return result

	def exportParams(self, params):
		for s in self.active:
			param = "hardware.servos.%d" % s
			params[param + ".min"] = self.servos[s].min
			params[param + ".mid"] = self.servos[s].mid
			params[param + ".max"] = self.servos[s].max
		params["worm.pauseRate"] = self.pause_rate

	def setState(self, state):
		self.runtime_state = int(state)

	def setRate(self, rate):
		self.pause_rate = float(rate)

	def wormControlSetState(self, mtype, id):
		if mtype in CYCLES and id in self.stateMap[mtype]:
			self.currentCycle = CYCLES[mtype]
			self.currentState = self.stateMap[mtype][id]["id"]
			if mtype == "turn":
				self.currentTurn = self.nextTurn
			self.module.updateState(byref(self))
			return 0
		else:
			return -1

	def wormControlSetStepsize(self, value):
		self.stepSize = float(value)
		self.module.updateState(byref(self))
		return 0

	def wormControlSetTurn(self, angle):
		self.module.setTurn(byref(self), float(angle))

	def loadStates(self, params, mtype):
		self.stateMap[mtype] = {}
		pattern = re.compile("worm\\.%s\\.(\w+)\\." % mtype)
		keys = list(params.keys())
		keys.sort()
		for key in keys:
			m = pattern.match(key)
			if m is not None:
				if m.group(1) not in self.stateMap[mtype]:
					stateId = m.group(1)
					print(mtype,stateId)
					stateServo = 0
					while True:
						prefix = "worm.%s.%s.poly.%c" % (mtype,stateId, ord('a') + stateServo)
						if prefix + ".servo" in params:
							self.poly[self.NumPoly].load(params, prefix)
							if mtype == "state":
								self.states[self.NumStates].servoPoly[stateServo] = self.NumPoly
							else:
								self.turnStates[self.NumTurnStates].servoPoly[stateServo] = self.NumPoly
							self.NumPoly += 1
							stateServo += 1
						else:
							break
					if mtype == "state":
						self.states[self.NumStates].num = stateServo
					else:
						self.turnStates[self.NumTurnStates].num = stateServo

					turnServo = 0
					"""
					while True:
						prefix = "worm.%s.%s.turn.%c" % (mtype, stateId, ord('a') + turnServo)
						if prefix + ".servo" in params:
							self.poly[self.NumPoly].load(params, prefix)
							if mtype == "state":
								self.states[self.NumStates].servoTurn[turnServo] = self.NumPoly
							else:
								self.turnStates[self.NumTurnStates].servoTurn[turnServo] = self.NumPoly
							self.NumPoly += 1
							turnServo += 1
						else:
							break
					"""
					if mtype == "state":
						self.states[self.NumStates].numTurnServos = turnServo
						self.stateMap[mtype][stateId] = {"id": self.NumStates}
					else:
						self.turnStates[self.NumTurnStates].numTurnServos = turnServo
						self.stateMap[mtype][stateId] = {"id": self.NumTurnStates}

					minParam = "worm.%s.%s.min" % (mtype, stateId)
					maxParam = "worm.%s.%s.max" % (mtype, stateId)
					initParam = "worm.%s.%s.init" % (mtype, stateId)
					self.stateMap[mtype][stateId]["min"] = float(params[minParam]) if minParam in params else 0
					self.stateMap[mtype][stateId]["max"] = float(params[maxParam]) if maxParam in params else 0
					self.stateMap[mtype][stateId]["init"] = float(params[initParam]) if initParam in params else 0
					if self.stateMap[mtype][stateId]["max"] > 0 and self.stateMap[mtype][stateId]["max"] < self.stepSize:
						self.stepSize = self.stateMap[mtype][stateId]["max"]
					
					if mtype == "state":
						self.NumStates += 1
					else:
						self.NumTurnStates += 1

class Hardware(HardwareStub):
	def __init__(self, logger, config_file):
		super(Hardware, self).__init__(logger, config_file)
		self.module = cdll.LoadLibrary('module/libhardwareModule.so')

		self.loadParams()

		self.ServoControl = TServoControl(self.module, self.params, logger)

		self.idlePause = 0.5

	def getServos(self):
		return self.ServoControl.getServos()

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
		self.ServoControl.runtime_pause = self.idlePause
		self.ServoControl.setState(1)

	def wormMoveForward(self):
		self.ServoControl.setState(2)

	def wormMoveBackward(self):
		self.ServoControl.setState(3)

	def wormSetPauseRate(self, rate):
		self.ServoControl.setRate(rate)

	def wormControlSetState(self, mtype, id):
		return self.ServoControl.wormControlSetState(mtype, id)

	def wormControlSetStepsize(self, value):
		return self.ServoControl.wormControlSetStepsize(value)

	def wormControlSetTurn(self, angle):
		self.ServoControl.wormControlSetTurn(angle)

	def wormGetStates(self):
		result = {}
		for cycle, smap in self.ServoControl.stateMap.items():
			result[cycle] = []
			for s,p in smap.items():
				np = copy.deepcopy(p)
				np["name"] = s
				result[cycle].append(np)
			result[cycle].sort(key=lambda x: x["id"])
		return result

	def wormSetMouthState(self, state):
		self.ServoControl.submitMouthState = state


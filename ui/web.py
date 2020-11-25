import sys

from flask import Flask
from flask import Response
from flask import render_template
from flask import request

import json

app = Flask(__name__)

g_Hardware = None

@app.route('/')
def root():
	return app.send_static_file('index.html')

@app.route('/getservos')
def getservos():
	servos = g_Hardware.getServos()
	return Response(json.dumps(servos), content_type='text/plain; charset=utf-8')

@app.route('/setservovalue/<servo>/<value>')
def set_servo_value(servo, value):
	g_Hardware.setServoValue(servo, value)
	response = {"servo": servo, "value": value}
	return Response(json.dumps(response), content_type='text/plain; charset=utf-8')


@app.route('/setservoangle/<servo>/<angle>')
def set_servo_angle(servo, angle):
	value = g_Hardware.setServoAngle(servo, angle)
	response = {"servo": servo, "angle": angle, "value": value}
	return Response(json.dumps(response), content_type='text/plain; charset=utf-8')

@app.route('/setservoparams/<servo>/<vmin>/<vmid>/<vmax>')
def set_servo_params(servo, vmin, vmid, vmax):
	g_Hardware.setServoParams(servo, vmin, vmid, vmax)
	response = {"servo": servo}
	return Response(json.dumps(response), content_type='text/plain; charset=utf-8')

@app.route('/worm/steps')
def get_worm_steps():
	steps = g_Hardware.getWormSteps()
	return Response(json.dumps(steps), content_type='text/plain; charset=utf-8')

@app.route('/worm/setstate/<state>')
def worm_set_state(state):
	states = {"stop": g_Hardware.wormStop,
			  "forward": g_Hardware.wormMoveForward,
			  "backward": g_Hardware.wormMoveBackward}
	if state in states:
		states[state]()
		result = {"stats":"OK"}
	else:
		result = {"stats":"INCORRECT STATE"}
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

@app.route('/worm/setrate/<rate>')
def worm_set_rate(rate):
	g_Hardware.wormSetPauseRate(rate)
	result = {"stats":"OK"}
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

@app.route('/worm/mouth/<state>')
def worm_mouth(state):
	states ={'open':1, 'close':0}
	if state in states:
		g_Hardware.wormSetMouthState(states[state])
		result = "OK"
	else:
		result = "NotSuchState"
	return Response(json.dumps({"status":result}), content_type='text/plain; charset=utf-8')

@app.route('/worm/control/<mtype>/<id>')
def worm_control(mtype,id):
	result = g_Hardware.wormControlSetState(mtype,id)
	return Response(json.dumps({"status":result}), content_type='text/plain; charset=utf-8')

@app.route('/worm/stepsize/<value>')
def worm_setstepsize(value):
	result = g_Hardware.wormControlSetStepsize(value)
	return Response(json.dumps({"status":result}), content_type='text/plain; charset=utf-8')

@app.route('/worm/turn/<value>')
def worm_setturn(value):
	g_Hardware.wormControlSetTurn(value)
	return Response(json.dumps({"status":"Ok"}), content_type='text/plain; charset=utf-8')

@app.route('/worm/getcontrolstates')
def worm_get_control_states():
	states = g_Hardware.wormGetStates()
	return Response(json.dumps(states), content_type='text/plain; charset=utf-8')

@app.route('/rangefinder/readDistance')
def getDistance():
	distance = g_Hardware.readDistance()
	return Response(json.dumps(distance), content_type='text/plain; charset=utf-8')

@app.route('/hscan2/<angle0>/<angle1>/<from2>/<to2>/<step2>/<prePause>/<movePause>')
def HScan2(angle0, angle1, from2, to2, step2, prePause, movePause):
	result = g_Hardware.HScan2(angle0, angle1, from2, to2, step2, prePause, movePause)
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

@app.route('/hscan3')
def HScan3():
	result = g_Hardware.HScan3()
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')	

@app.route('/hscan4')
def HScan4():
	result = g_Hardware.HScan4()
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')	

@app.route('/getscanvars')
def getScanVars():
	result = g_Hardware.getScanVars()
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

@app.route('/setscanvar/<var>/<value>')
def setScanVar(var, value):
	result = g_Hardware.setScanVar(var, value)
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

@app.route('/worm/startscan')
def startScan():
	result = g_Hardware.startScan()
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')
	
@app.route('/worm/getlastmeasurements')
def getLasMeasurements():
	result = g_Hardware.getLastMeasurements()
	return Response(json.dumps(result), content_type='text/plain; charset=utf-8')

if __name__ == "__main__":
	if len(sys.argv) >= 2 and sys.argv[1] == "stub":
		from HardwareStub import HardwareStub
		g_Hardware = HardwareStub(app.logger, "config/config.ini")
	else:
		from Hardware import Hardware
		g_Hardware = Hardware(app.logger, "config/config.ini")

	app.run(host='0.0.0.0', port=7778, debug=True)

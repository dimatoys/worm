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

if __name__ == "__main__":
	if len(sys.argv) >= 2 and sys.argv[1] == "stub":
		from HardwareStub import HardwareStub
		g_Hardware = HardwareStub(app.logger, "config/config.ini")
	else:
		from Hardware import Hardware
		g_Hardware = Hardware(app.logger, "config/config.ini")

	app.run(host='0.0.0.0', port=7778, debug=True)

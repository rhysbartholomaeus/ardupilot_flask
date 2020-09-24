import requests
from flask import Flask, request, jsonify, Response, json
from flask_cors import CORS
from collections import OrderedDict
import dronekit_sitl
import subprocess

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil # Needed for command message definitions

class Drone(object):
    def __init__(self):
        self.vehicle = None
        # Do nothing

    def setup(self):
        self.vehicle = connect ('127.0.0.1:14550', wait_ready = True)
        self.vehicle.add_attribute_listener('armed', self._armed_callback)

        while not self.vehicle.is_armable:
            time.sleep(1)

        self.vehicle.armed = True 
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        cmds.clear()

        lat = -35.45228
        lon = 139.279175
        altitude = 30.0

        takeoffCmd = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)

        cmds.add(takeoffCmd)
        cmds.upload()
        self.vehicle.mode = VehicleMode("AUTO")
        

    def goto(self, point):
        self.vehicle.simple_goto(point)

    def _armed_callback(self, vehicle, name, message):
        if self.vehicle.location.global_relative_frame.alt < 100:
            self.vehicle.mode = VehicleMode("AUTO")
            #self.vehicle.simple_takeoff(100) # takeoff at 100 meters

class droneProcess(object):
    def __init__(self):
        self.processId = None
    # Do nothing
    def setup(self,lat,lng):
        self.processId = subprocess.Popen(["python3", "/home/demo/ardupilot/Tools/autotest/sim_vehicle.py", "-w" , "-v", "ArduPlane", "-l",lat + "," + lng + ",100,0"])


app = Flask("ardupilot_control")
CORS(app)

d = Drone()
proc = droneProcess()

@app.route('/startSITL')
def startSimualtor():
    
    # Use '-l' to set a custom location (lat,lon,alt,heading)
    if(proc.processId == None):
        lat = -35.22313
        lng = 138.38434
        proc.setup(str(lat), str(lng))
    return "Ok"

@app.route("/track")
def track():
    return str(d.vehicle.location.global_frame)

@app.route("/goto", methods=['POST'])
def goto():
    lat = float(request.form['lat'])
    lon = float(request.form['lon'])
    alt = float(request.form['alt'])
    point = LocationGlobal(lat, lon, alt)
    d.goto(point)
    return str(point)

@app.route('/setupDrone')
def setupDrone():
    d.setup()
    return "Ok"

@app.route('/getVehicleStatus')
def getGeoJSON():

        locat = d.vehicle.location.global_frame
        print('!!! Location is: ',locat.lat, locat.lon)
        f = [
            OrderedDict(
                type='Feature',
                id="PersonalDrone",
                geometry=OrderedDict(type='Point', coordinates=[locat.lon,locat.lat]),
                properties=OrderedDict([('callsign', d.vehicle._vehicle_type), ('velocity',d.vehicle.groundspeed),('geo_altitude', locat.alt),('heading', d.vehicle.heading),('course', d.vehicle._yaw)]))
        ]

        tempResponse = Response()
        # This is to overcome issues with the QRC cross origin issues
        tempResponse.headers.add('Access-Control-Allow-Origin', '*')

        return Response(
            response=json.dumps(dict(
            type='FeatureCollection',
            features=f
        )),
            status=200,
            headers=tempResponse.headers,
            mimetype='application/json'
        )

@app.route('/goToLocation')
def goToLocation():

    #d.vehicle.airspeed = 15
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    alt = 100 # Hardcoded for now.
    
    d.vehicle.mode = VehicleMode("GUIDED")

    cmds = d.vehicle.commands
    # cmds.download()
    # cmds.wait_ready()
    cmds.clear()
    cmds.upload()
    cmds.wait_ready()

    goToCmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt)
    
    cmds.add(goToCmd)
    cmds.upload()
    cmds.wait_ready()
    d.vehicle.mode = VehicleMode("AUTO")
    return "Ok"

if __name__ == "__main__":
    app.run()

import math
import time
import socket
from pymavlink import mavutil
import telemetry
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from dronekit import Vehicle

vehicle = connect('COM5', baud=115200)
temtem = telemetry.Telemetry()


@vehicle.on_attribute('ekf_ok')
def ekf_callback(self, attr_name, value):
    print(attr_name, value)


@vehicle.on_attribute('last_heartbeat')
def lastheartbeat_callback(self, attr_name, value):
    print(attr_name, value)


@vehicle.on_attribute('location.global_relative_frame')
def relative_location_callback(self, attr_name, value):
    print(attr_name, value)


def get_mode():
    temtem.mode = vehicle.mode



def arm():
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)


def set_airspeed(airspeed):
    vehicle.airspeed = airspeed


def set_groundspeed(groundspeed):
    vehicle.groundspeed = groundspeed


def simple_go_to(lat, lon, alt):
    a_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(a_location)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    # TODO: will be calculated in ground station. need to save home location in local
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def download_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return cmds


def clear_mission():
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()


def change_next_waypoint(num):
    vehicle.commands.next = num


def upload_mission():
    # TODO: later will be complete currently there is no filter to differentiate takeoff, waypoint or cam trigger
    cmds = vehicle.commands
    cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0,
                   0, 0, 0, 0, 0, 10)
    cmd2 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                   0, 0, 0, 0, 10, 10, 10)
    cmds.add(cmd1)
    cmds.add(cmd2)
    cmds.upload()  # Send commands


def save_mission(aFileName):
    missionlist = download_mission()
    output = 'QGC WPL 110\n'
    for cmd in missionlist:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output += commandline
    with open(aFileName, 'w') as file_:
        file_.write(output)


def readmission(aFileName):
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission_from_file(aFileName):
    missionlist = readmission(aFileName)
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    vehicle.commands.upload()


def distance_to_current_waypoint():
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def wildcard_callback(self, attr_name, value):
    print(f"CALLBACK {attr_name} : {value}")


# 11171 ya da 1172,     21004 210074

def get_home_location():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return vehicle.home_location


@vehicle.on_attribute('groundspeed')
def groundspeed_callback(self, attr_name, value):
    temtem.groundSpeed = value


@vehicle.on_attribute('airspeed')
def airspeed_callback(self, attr_name, value):
    temtem.airSpeed = value


@vehicle.on_attribute('gps_0')
def gps_callback(self, attr_name, value):
    temtem.NumSat = value.satellites_visible
    temtem.gdop = value.eph



@vehicle.on_attribute('heading')
def heading_callback(self, attr_name, value):
    temtem.heading = value
    temtem.heightC = value


@vehicle.on_attribute('battery')
def battery_callback(self, attr_name, value):
    temtem.volts = value.voltage
    temtem.amps = value.current


@vehicle.on_attribute('attitude')
def attitude_callback(self, attr_name, value):
    temtem.pitch = value.pitch
    temtem.roll = value.roll
    temtem.heightC = value.heading

    # TODO: why there is no yaw
    # temtem.yaw = value.yaw


@vehicle.on_attribute('velocity')
def velocity_callback(self, attr_name, value):
    print(attr_name, value)

import math
@vehicle.on_attribute('location.global_frame')
def global_location_callback(self, attr_name, value):
    temtem.lat = value.lat * (math.pi/180)
    temtem.lon = value.lon * (math.pi/180)
    temtem.alt = value.alt
    temtem.rH = value.alt

@vehicle.on_attribute('wind')
def wind_callback(self, attr_name, value):
    # attr_name == 'raw_imu'
    # value == vehicle.raw_imu
    temtem.windSpeed = value.wind_speed

vehicle.add_attribute_listener('gps_0', gps_callback)
vehicle.add_attribute_listener('wind', wind_callback)



while True:
    time.sleep(0.1)
    temtem.sendTelemetry()

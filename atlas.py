from dronekit import Vehicle, connect


class RawIMU(object):

    def __init__(self, time_boot_us=None, xacc=None, yacc=None, zacc=None, xygro=None, ygyro=None, zgyro=None,
                 xmag=None, ymag=None, zmag=None):
        self.time_boot_us = time_boot_us
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.xgyro = zgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag

    def __str__(self):
        return "RAW_IMU: time_boot_us={},xacc={},yacc={},zacc={},xgyro={},ygyro={},zgyro={},xmag={},ymag={},zmag={}".format(
            self.time_boot_us, self.xacc, self.yacc, self.zacc, self.xgyro, self.ygyro, self.zgyro, self.xmag,
            self.ymag, self.zmag)


class Wind(object):
    def __init__(self, wind_direction=None, wind_speed=None, wind_speed_z=None):
        self.wind_direction = wind_direction
        self.wind_speed = wind_speed
        self.wind_speed_z = wind_speed_z

    def __str__(self):
        return "Wind: wind direction: {}, wind speed: {}, wind speed z: {}".format(self.wind_direction, self.wind_speed,
                                                                                   self.wind_speed_z)


class Atlas(Vehicle):
    def __init__(self, *args):
        super(Atlas, self).__init__(*args)

        # Create an Vehicle.raw_imu object with initial values set to None.
        self._raw_imu = RawIMU()
        self._wind = Wind()

        # Create a message listener using the decorator.
        @self.on_message('RAW_IMU')
        def listener(self, name, message):
            self._raw_imu.time_boot_us = message.time_usec
            self._raw_imu.xacc = message.xacc
            self._raw_imu.yacc = message.yacc
            self._raw_imu.zacc = message.zacc
            self._raw_imu.xgyro = message.xgyro
            self._raw_imu.ygyro = message.ygyro
            self._raw_imu.zgyro = message.zgyro
            self._raw_imu.xmag = message.xmag
            self._raw_imu.ymag = message.ymag
            self._raw_imu.zmag = message.zmag
            self.notify_attribute_listeners('raw_imu', self._raw_imu)

        @self.on_message('wind')
        def listenerr(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.

            The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object
            and notifies observers.
            """
            # self.wind_direction = wind_direction
            # self.wind_speed = wind_speed
            # self.wind_speed_z = wind_speed_z


            self._wind.wind_direction = message.wind_direction
            self._wind.wind_speed = message.wind_speed
            self._wind.wind_speed_z = message.wind_speed_z



            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('wind', self._wind)

    @property
    def wind(self):
        return self._wind

    @property
    def raw_imu(self):
        return self._raw_imu

vehicle = connect('COM8', baud=115200, vehicle_class=Atlas)

def raw_imu_callback(self, attr_name, value):
    # attr_name == 'raw_imu'
    # value == vehicle.raw_imu
    print(value)

def wind_callback(self, attr_name, value):
    # attr_name == 'raw_imu'
    # value == vehicle.raw_imu
    print(value)

vehicle.add_attribute_listener('raw_imu', raw_imu_callback)
vehicle.add_attribute_listener('wind', wind_callback)

while True:
    pass





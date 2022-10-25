



class Telemetry:
    mheader = 0
    ykiId = 0
    uckId = 0
    gorevID = 0
    MID = 100
    mode = 100 #
    lat = 34.12 #
    lon = 36.22 #
    alt = 900 #
    roll = 33 #
    pitch = 10 #
    heading = 90 #
    airSpeed = 10 #
    rH = 0  # Relative Height to Ground
    rDist = 0  # Distance from launch point
    groundSpeed = 10 #
    windSpeed = 10
    windHeading = 0  # Rüzgar yönü
    volts = 17 #
    amps = 4   #
    gdop = 0  # DOP, değiştirilebilir
    rssi = 0  # RF Çekim gücü - Değerler
    emergncyCode = 0  # Dökümante edilecek
    flightTime = 120
    distTraveled = 0  # Distance covered since the launch
    capacity = 0  # Total Power left
    PPKStat = 0  # None
    NumSat = 0  # Number of satellites #
    BatStat = 0
    heightC = 0  # Hedeflenen yükseklik
    subMode = 0  # Bunun sayıları var
    crc = 0  # çöp
    gorevDosyasi = "dosya"  # arayuz icin gerekli

    def constructMessage(self):
        return str(self.mheader) + ";" + \
               str(self.ykiId) + ";" + \
               str(self.uckId) + ";" + \
               str(self.gorevID) + ";" + \
               str(self.MID) + ";" + \
               str(self.mode) + ";" + \
               str(self.lat) + ";" + \
               str(self.lon) + ";" + \
               str(self.alt) + ";" + \
               str(self.roll) + ";" + \
               str(self.pitch) + ";" + \
               str(self.heading) + ";" + \
               str(self.airSpeed) + ";" + \
               str(self.rH) + ";" + \
               str(self.rDist) + ";" + \
               str(self.groundSpeed) + ";" + \
               str(self.windSpeed) + ";" + \
               str(self.windHeading) + ";" + \
               str(self.volts) + ";" + \
               str(self.amps) + ";" + \
               str(self.gdop) + ";" + \
               str(self.rssi) + ";" + \
               str(self.emergncyCode) + ";" + \
               str(self.flightTime) + ";" + \
               str(self.distTraveled) + ";" + \
               str(self.capacity) + ";" + \
               str(self.PPKStat) + ";" + \
               str(self.NumSat) + ";" + \
               str(self.BatStat) + ";" + \
               str(self.heightC) + ";" + \
               str(self.subMode) + ";" + \
               str(self.crc) + ";" + \
               str(self.gorevDosyasi)


    def send_to_udp(self):
        pass
import serial
import time
import queue
import threading
import math
from pyproj import CRS, Transformer
from .NtripReceiver import NtripReceiver

class UM982NtripDriver(threading.Thread):
    def __init__(self, port, baudrate=115200):
        super().__init__(name="UM982DriverClass")
        self.serial_port_name = port
        self.baudrate = baudrate
        self.caster_host = None
        self.caster_port = None
        self.mountpoint = None
        self.username = None
        self.password = None

        self.fix            = None
        self.orientation    = None
        self.vel            = None
        self.utmpos         = None    
        self.rtcm_status    = None  
        self.output_queue= queue.Queue()
        self.NMEA_EXPEND_CRC_TABLE = self._crc_table() 
        self.running=True
        
    def set_caster(self, caster_host, caster_port, mountpoint, username, password):
        self.caster_host = caster_host
        self.caster_port = caster_port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        
    def _crc_table(self):
        table = []
        for i in range(256):
            crc = i
            for j in range(8, 0, -1):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
            table.append(crc)
        return table
       
    def _print_data(self):
        while True:
            print(f"*****")
            print(f"FIX:    {self.fix}")
            print(f"UTMPOS: {self.utmpos}")
            print(f"ORIENT: {self.orientation}")
            print(f"VEL:    {self.vel}")
            print(f"RTCM:   {self.rtcm_status}")
            time.sleep(0.2)
         
    def _nmea_expend_crc(self, nmea_expend_sentence):
        def calculate_crc32(data):
            crc = 0
            for byte in data:
                crc = self.NMEA_EXPEND_CRC_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
            return crc & 0xFFFFFFFF
        try:
            sentence, crc = nmea_expend_sentence[1:].split("*")
            crc = crc[:8]
        except:
            return False
        calculated_crc = calculate_crc32(sentence.encode())
        return crc.lower() == format(calculated_crc, '08x')

    def _nmea_crc(self, nmea_sentence):
        try:
            sentence, crc = nmea_sentence[1:].split("*")
            crc = crc[:2]
        except:
            return False
        calculated_checksum = 0
        for char in sentence:
            calculated_checksum ^= ord(char)
        calculated_checksum_hex = format(calculated_checksum, 'X')
        return calculated_checksum_hex.zfill(2) == crc.upper()

    def _wgs84_to_UTM(self, lat, lon):
        #calcolo zona UTM
        zone_number = int((lon + 180) / 6) + 1
        #are we in norther hemisphere?
        isnorth = lat >= 0
        #wgs84 system EPSG code
        wgs84_crs = CRS("epsg:4326")
        #make the appropriate UTM EPSG code depending on whether you are in the Northern Hemisphere
        utm_crs_str = f"epsg:326{zone_number}" if isnorth else f"epsg:327{zone_number}"
        utm_crs     = CRS(utm_crs_str)
        #create a coordinate converter
        transformer = Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)
        return transformer
  
    def _msg_split(self, msg:str):
        return msg[1:msg.find('*')].split(',')
    
    def _PVTSLN_solver(self, msg:str):
        parts = self._msg_split(msg)
        bestpos_hgt    = float(parts[3+7])
        bestpos_lat    = float(parts[4+7])
        bestpos_lon    = float(parts[5+7])
        bestpos_hgtstd = float(parts[6+7])
        bestpos_latstd = float(parts[7+7]) 
        bestpos_lonstd = float(parts[8+7])
        fix = (bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd)
        return fix

    def _GNHPR_solver(self, msg:str):
        parts = self._msg_split(msg)
        heading = float(parts[3-1])
        pitch   = float(parts[4-1])
        roll    = float(parts[5-1])
        orientation = (heading, pitch, roll)
        return orientation

    def _BESTNAV_solver(self, msg:str):
        parts = self._msg_split(msg)
        vel_hor_std = float(parts[-1]) 
        vel_ver_std = float(parts[-2]) 
        vel_ver     = float(parts[-3])
        vel_heading = float(parts[-4])
        vel_hor     = float(parts[-5])
        vel_north   = vel_hor * math.cos(math.radians(vel_heading))
        vel_east    = vel_hor * math.sin(math.radians(vel_heading))
        return (vel_east, vel_north, vel_ver, vel_hor_std, vel_hor_std, vel_ver_std)
    
    def stop(self):
        self.running=False
    
    def run(self): 
        use_ntrip=False        
        if self.caster_host is not None and self.caster_port is not None and self.username is not None and self.password is not None and  self.mountpoint is not None:
            use_ntrip=True            
        
        #enable NTRIP caster
        if use_ntrip:
            ntrip_data_queue = queue.Queue()
            self.ntrip_receiver=NtripReceiver(self.caster_host, self.caster_port, self.username, self.password, self.mountpoint, ntrip_data_queue);
            
        #monitor_thread = threading.Thread(target=self._print_data, daemon=True)
        #monitor_thread.start()
        
        try:           
            with serial.Serial(port = self.serial_port_name, baudrate = self.baudrate, timeout=0.5) as ser:   
                while self.running:  
                    #Send RTCM byte stream
                    if use_ntrip:
                        try:       
                            data = ntrip_data_queue.get(block=False)
                            self.rtcm_status=self.ntrip_receiver.status
                            if not data is None:
                                ser.write(data)    
                        except Exception as e:
                            pass
                    
                    # Read a line from the serial port
                    frame = ser.readline().decode('utf-8')
                    if not frame is None:
                        if frame.startswith("#PVTSLNA") and self._nmea_expend_crc(frame):
                            self.fix = self._PVTSLN_solver(frame)
                            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.fix
                            self.transformer = self._wgs84_to_UTM(bestpos_lat, bestpos_lon)
                            self.utmpos = self.transformer.transform(bestpos_lon, bestpos_lat)
                            #print(self.fix)
                        elif frame.startswith("$GNHPR") and self._nmea_crc(frame):
                            self.orientation = self._GNHPR_solver(frame)
                            #print(self.orientation)
                        elif frame.startswith("#BESTNAVA") and self._nmea_expend_crc(frame):
                            self.vel = self._BESTNAV_solver(frame)
                            #print(self.vel)     
                    time.sleep(0.001)                     
        except serial.SerialException as e:
            return None    

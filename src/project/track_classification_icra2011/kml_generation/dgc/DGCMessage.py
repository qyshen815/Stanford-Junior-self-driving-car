'''
Created on Nov 24, 2009

@author: mvs
'''

class DGCMessage():
    '''
    classdocs
    '''
    module = ''
    host = ''
    hosttime = -1.0
    logtime = -1.0
    data = ()

    def __init__(self, logstr):
        '''
        Imports a line from a logfile and parses it to a generic data
        '''
        tokens = logstr.split()
        if len(tokens) < 4:
            raise TypeError('Improperly formatted log entry (too short):\n', logstr)
        
        # Import basic fields
        self.module = tokens[0]
        self.host = tokens[-2]
        try:
            self.hosttime = float(tokens[-3])
            self.logtime = float(tokens[-1])
        except ValueError:
            raise TypeError('Improperly formatted log entry (bad timestamps):\n', logstr)
        self.data = tuple(tokens[1:-3])
        
        mod = None
        
        try:
            mod = __import__('dgc.DGCMessage')
            mod = getattr(mod, self.module)
        except AttributeError:
            return
        
        self.__class__ = mod
        self.__init__(self.data)
        
        
        
class CAN3(DGCMessage):
    '''
    '''
    
    def __init__(self, data):
        if len(data) != 24:
            raise TypeError('Improperly formatted CAN3 log entry')
        self.throttle = float(data[0])
        self.steeringangle = float(data[1])
        self.steeringrate = float(data[2])
        self.rpm = int(float(data[3]))
        self.parkingbreak = bool(int(data[4]))
        self.gear = int(data[5])
        self.gearposition = int(data[6])
        self.wheelspeed = (float(data[7]), float(data[8]), float(data[9]), float(data[10]))
        self.brakepressure = float(data[15])
        self.esp = bool(int(data[16]))
        self.abs = bool(int(data[17]))
        self.errthrottle = bool(int(data[18]))
        self.errrpm = bool(int(data[19]))

class APPLANIX_POSE_V2(DGCMessage):
    '''
    '''
    
    def __init__(self, data):
        if len(data) != 25:
            raise TypeError('Improperly formatted APPLANIX_POSE_V2 log entry')
        self.position = (float(data[0]), float(data[1]), float(data[2]))
        self.gps = (float(data[3]), float(data[4]), float(data[5]))
        self.velocity = (float(data[6]), float(data[7]), float(data[8]))
        self.speed = float(data[9])
        self.direction = float(data[10])
        self.heading = (float(data[11]), float(data[12]), float(data[13]))
        self.headingrate = (float(data[14]), float(data[15]), float(data[16]))
        self.acceleration = (float(data[17]), float(data[18]), float(data[19]))
        self.wander = float(data[20])
        self.id = int(data[21])
        self.postprocess = int(data[22])
        self.hardwaretime = float(data[23])
        self.hardwaretimemode = int(data[24])
                         
        
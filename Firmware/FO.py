import socketio
from pylibrnp.defaultpackets import TelemetryPacket
from pylibrnp.rnppacket import *
import json

class FlyingObject:
    '''
    The object that Pickle Rick has been inserted into, and is being tracked
    '''

    sio = socketio.Client(logger=False, engineio_logger=False)

    def __init__(self):
        self.off_n = 0
        self.off_e = 0
        self.off_d = 0
        self.ref_lat = 0
        self.ref_long = 0
        self.ref_alt = 0

    def connect_to_address(self, address):
        self.sio.connect(address)

    @sio.on('telemetry', namespace='/telemetry')
    def on_message(self, data):
        self.off_n = data['pn']
        self.off_e = data['pe']
        self.off_d = data['pd']
        self.ref_lat = data['start_lat']
        self.ref_long = data['start_long']
        self.ref_alt = data['start_alt']


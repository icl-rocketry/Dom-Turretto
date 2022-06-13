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
        self.pn = 0
        self.pe = 0
        self.pd = 0
        self.start_lat = 0
        self.start_long = 0
        self.start_alt = 0

    def connect_to_address(self, address):
        self.sio.connect(address)

    @sio.on('telemtry', namespace='/telemetry')
    def on_message(data):
        try:
            packet = bytes.fromhex(data['Data'])
            header = RnpHeader.from_bytes(packet)
            if header.source_service is 2 and header.packet_type is 101:
                # Telemetry packet sent node wide


        except:
            print("Failed to decode header")

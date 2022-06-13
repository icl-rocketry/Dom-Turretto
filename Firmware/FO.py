import socketio

class FlyingObject:
    '''
    The object that Pickle Rick has been inserted into, and is being tracked
    '''

    def __init__(self):
        self.pn = 0
        self.pe = 0
        self.pd = 0
        self.start_lat = 0
        self.start_long = 0
        self.start_alt = 0

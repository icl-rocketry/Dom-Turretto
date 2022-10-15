from FO  import FlyingObject
from turret import Turret
import time

pickle_rick = FlyingObject()
dom_turretto = Turret(pickle_rick)

dom_turretto.set_heading(0)
dom_turretto.stepper_rig.set_gain(1,0.566666666666666)
pickle_rick.connect_to_address("http://host:port/")


while True:
    dom_turretto.set_tgt()
    dom_turretto.update()
    time.sleep(100)
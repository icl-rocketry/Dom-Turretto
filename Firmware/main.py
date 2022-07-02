import FO
from turret import Turret
import time

pickle_rick = FO()
dom_turretto = Turret(pickle_rick)

dom_turretto.set_heading(0)
dom_turretto.stepper_rig.set_gain(1,0.566666666666666)


while True:
    dom_turretto.set_tgt()
    dom_turretto.update()
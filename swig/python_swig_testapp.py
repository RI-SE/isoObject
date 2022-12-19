from isoObject_wrap import TestObject, CartesianPosition, SpeedType
from time import sleep


class myTestObject(TestObject):

    # This is now overridden in the target language (Python)
    def handleAbort(self):
        print("handleAbort() from python!")


if __name__ == "__main__":

    isoObject = myTestObject()
    pos = CartesianPosition()
    pos.xCoord_m = 0
    pos.yCoord_m = 0
    pos.zCoord_m = 0
    pos.heading_rad = 0
    pos.isHeadingValid = True
    pos.isPositionValid = True
        
    spd = SpeedType()
    spd.lateral_m_s = 0
    spd.longitudinal_m_s = 0
    spd.isLateralValid = True
    spd.isLongitudinalValid = True
    isoObject.setPosition(pos)
    isoObject.setSpeed(spd)

    while(True):
       sleep(1)

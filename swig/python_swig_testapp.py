from isoObject_wrap import TestObject, CartesianPosition, \
    SpeedType, AccelerationType
from time import sleep


class myTestObject(TestObject):

    # This is now overridden in the target language (Python).
    # The "director" feature of SWIG is used to achieve this.
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
    pos.isXcoordValid = True
    pos.isYcoordValid = True
    pos.isZcoordValid = True

    spd = SpeedType()
    spd.lateral_m_s = 0
    spd.longitudinal_m_s = 0
    spd.isLateralValid = True
    spd.isLongitudinalValid = True

    acc = AccelerationType()
    acc.isLateralValid = True
    acc.isLongitudinalValid = True
    acc.lateral_m_s2 = 0
    acc.longitudinal_m_s2 = 0

    isoObject.setPosition(pos)
    isoObject.setSpeed(spd)
    isoObject.setAcceleration(acc)

    x = -1.0
    y = 1.0
    while (True):
        sleep(1)
        x -= 1.0
        y += 1.0
        pos.xCoord_m = x
        pos.yCoord_m = y
        isoObject.setPosition(pos)

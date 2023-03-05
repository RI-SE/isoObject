from isoObject_wrap import TestObject, CartesianPosition, \
    SpeedType, AccelerationType
from time import sleep
from math import cos, sin


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

    # Let the object move in a circle around the origin, jumping up and down
    originX = 0.0
    originY = 0.0
    originZ = 0.0
    radius = 5.0
    angle = 0.0
    x = 0.0
    y = 0.0
    z = 0.0
    while (True):
        sleep(0.05)
        angle += 0.05
        if (angle > 2*3.14159):
            angle = 0.0
        x = originX + cos(angle)*radius
        y = originY + sin(angle)*radius
        z = originZ + sin(angle)*radius/2.0
        if (z < 0.0):
            z = 0.0
        pos.xCoord_m = x
        pos.yCoord_m = y
        pos.zCoord_m = z
        isoObject.setPosition(pos)

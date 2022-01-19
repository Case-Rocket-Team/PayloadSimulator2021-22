import math
import numpy

#actually make work at some point
lastLooked = 0

def distFormula3D(pos, point):
    return math.sqrt((pos.getX() - point.getX()) ** 2) + ((pos.getY() - point.getY()) ** 2) #+ (pos.getZ() - point.getZ()) ** 2)

def main(pos, path, lookAheadDist, velocity):
    global lastLooked
    prev = lastLooked
    #traversal variable
    trav = lastLooked.getNext()

    while distFormula3D(pos, trav) < distFormula3D(pos, prev):
        prev = trav
        trav = trav.getNext()
    while distFormula3D(pos, trav) < lookAheadDist:
        prev = trav
        trav = trav.getNext()

    #finding distance between us and where we want to go
    #(pos.getX() - trav.getX() + (pos.getY() - trav.getY() + #(pos.getZ() - trav.getZ()) 
    #invverse (cos u . v) / (mag u) (mag v)
    math.acos(numpy.dot(a, b)[] / 
    #rotation matrix code
    rotX = (point.getX() * cos(angle)) - (point.getY() * sin(angle))
    #rotY = (point.getX() * sin(angle)) + (point.getY() * cos(angle))



main()

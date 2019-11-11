from pathgenerator import PathGenerator
from pathprimitive import PathPrimitive
from primitiveline import PrimitiveLine
from primitivecurve import PrimitiveCurve
import random as rand
import numpy as np
from math import pi

class PathGeneratorTerrainSmooth(PathGenerator):

    def __init__(self, straightChance, pathLengthRange, radiusRange, pathTime):
        self._straightChance = straightChance
        self._pathLengthRange = pathLengthRange
        self._radiusRange = radiusRange
        self._pathTime = pathTime
        self._primitiveTries = 5

    #TODO: Obstacle avoidance + speeds
    def generateRandomPrimitivePath(self, map, agent):
        ''' Returns an array of primitives corresponding to a path for the given agent

            Args:
                map         (Map): Map to generate path on
                agent       (Agent): Agent to generate path for
        '''
        inBounds = True
        primitivePath = []
        timeLeft = self._pathTime
        heading = agent.rotation
        speed = agent.currentSpeed
        if agent.terrainCode == 3:
            g = PathGeneratorStraight(self._pathTime)
            return g.generateRandomPrimitivePath(map,agent)
        if (map._terrainDistribution[int(agent.position[0])][int(agent.position[1])] != agent.terrainCode):
            print("Changing start")
            if (agent.terrainCode in map._terrainDistribution[50:60, 50:60]):
                agent.position = agent.setStartingPointCentre(map)
            else:
                agent.position = agent.setStartingPoint(map)
        point = map.mapToWorld(agent.position[0], agent.position[1])
        firstPrimitive = True
        while timeLeft > 0:
            primitiveAttempts = self._primitiveTries
            inBounds = False
            primitive = None
            time = 0

            while inBounds == False and primitiveAttempts > 0:
                (pathTimeMin, pathTimeMax) = self._pathLengthRange
                time = min(rand.uniform(pathTimeMin, pathTimeMax), timeLeft)
                if (rand.random() < self._straightChance):
                    primitive = PrimitiveLine(time, speed, point, heading)
                else:
                    #print("Gets here")
                    validTerrain = False
                    count = 1
                    while (validTerrain == False):
                        #print(count)
                        if (count%1000 == 0):
                            print("Overshoot" + str(count))
                        if (count%1000 == 0):
                            radius = 0.0000000000000000001
                            dir = -1
                            heading = heading - pi/4
                        else:
                            dir = 1 + ((rand.random() < 0.5) * -2)
                            (radiusMin, radiusMax) = self._radiusRange
                            radius = rand.uniform(max(radiusMin, agent.minRadius), radiusMax)
                        primitive = PrimitiveCurve(time, speed, point, heading, radius, dir)
                        (x,y) = primitive.getEndPoint()
                        #validTerrain = True
                        #if not map.isWorldPointOutOfBounds((x,y)):
                        #    if (map._terrainDistribution[int(x)][int(y)] == agent.terrainCode):
                                #print("check")
                        #        validTerrain = True
                        flag = 0
                        #print("Center of circle: "+ str(primitive.getCircleCenter()))
                        #print("Radius: " + str(radius))
                        for dth in range(18):
                            (x,y) = primitive.getPointOnCircle(20*dth*pi/180)
                            #print("Point: " + str(x) + ", " + str(y))
                            if not map.isWorldPointOutOfBounds((y,x)):
                                if (map._terrainDistribution[int(y)][int(x)] != agent.terrainCode):
                                    flag += 1
                        if flag == 0:
                            validTerrain = True
                        count += 1
                inBounds = primitive.isInMapBounds(map) or (firstPrimitive and not map.isWorldPointOutOfBounds( primitive.getEndPoint() ))
                #print("Gets here")
                primitiveAttempts -= 1
            if inBounds == False:
                break
            else:
                timeLeft -= time
                heading = primitive.getEndHeading()
                point = primitive.getEndPoint()
                minSpeed = max(speed - agent.acceleration * time, agent.minSpeed)
                maxSpeed = min(speed + agent.acceleration * time, agent.maxSpeed)
                speed = rand.uniform(minSpeed, maxSpeed)
                primitivePath = np.append(primitivePath, [primitive])
                firstPrimitive = False

        return (PathPrimitive(primitivePath),inBounds)

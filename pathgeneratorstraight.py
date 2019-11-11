from pathgenerator import PathGenerator
from pathprimitive import PathPrimitive
from primitiveline import PrimitiveLine
from primitivecurve import PrimitiveCurve
import random as rand
import numpy as np
import math

class PathGeneratorStraight(PathGenerator):

    def __init__(self, pathTime):
        self._pathTime = pathTime

    #TODO: Obstacle avoidance + speeds
    def generateRandomPrimitivePath(self, map, agent):
        ''' Returns an array of primitives corresponding to a path for the given agent

            Args:
                map         (Map): Map to generate path on
                pathTIme    (float): How long should travelling on the path take?
                agent       (Agent): Agent to generate path for
        '''

        primitivePath = []
        speed = agent.currentSpeed
        timeLeft = self._pathTime
        point = map.mapToWorld(agent.position[0], agent.position[1])
        count = 0
        while timeLeft > 0:
            count += 1
            if (count%5000 == 0):
                return(PathPrimitive(primitivePath), False)
            nextPoint = map.getRandomPoint()
            dist = ((point[0] - nextPoint[0]) ** 2 + (point[1] - nextPoint[1]) ** 2) ** 0.5
            heading = math.atan2(-(nextPoint[1]-point[1]), nextPoint[0] - point[0])
            time = min( dist/speed, timeLeft)
            primitive = PrimitiveLine(time, speed, point, heading)
            timeLeft -= time
            point = primitive.getEndPoint()
            minSpeed = max(speed - agent.acceleration * time, agent.minSpeed)
            maxSpeed = min(speed + agent.acceleration * time, agent.maxSpeed)
            speed = rand.uniform(minSpeed, maxSpeed)
            primitivePath = np.append(primitivePath, [primitive])
        return (PathPrimitive(primitivePath),True)
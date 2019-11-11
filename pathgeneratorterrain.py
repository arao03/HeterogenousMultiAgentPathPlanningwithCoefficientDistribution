from pathgenerator import PathGenerator
from pathprimitive import PathPrimitive
from primitiveline import PrimitiveLine
from primitivecurve import PrimitiveCurve
import random as rand
import numpy as np
import math
import matplotlib.pyplot as plt
from pathgeneratorstraight import PathGeneratorStraight

class PathGeneratorTerrain(PathGenerator):

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

        '''Changing starting point for agent to be on suitable terrain'''
        if agent.terrainCode == 3:
            g = PathGeneratorStraight(self._pathTime)
            return g.generateRandomPrimitivePath(map,agent)
        if (map._terrainDistribution[int(agent.position[0])][int(agent.position[1])] != agent.terrainCode):
            if (agent.terrainCode in map._terrainDistribution[50:60, 50:60]):
                agent.position = agent.setStartingPointCentre(map)
            else:
                agent.position = agent.setStartingPoint(map)
        point = map.mapToWorld(agent.position[0], agent.position[1])

        while timeLeft > 0:
            checkPoints = False
            count = 0
            while checkPoints == False:
                count += 1
                validTerrain = False
                while validTerrain == False:
                    count+=1
                    nextPoint = map.getRandomPoint()
                    (x,y) = nextPoint
                    if int(map._terrainDistribution[y][x]) == agent.terrainCode:
                        validTerrain = True
                dist = ((point[0] - nextPoint[0]) ** 2 + (point[1] - nextPoint[1]) ** 2) ** 0.5
                heading = math.atan2(-(nextPoint[1]-point[1]), nextPoint[0] - point[0])
                time = min( dist/speed, timeLeft)
                primitive = PrimitiveLine(time, speed, point, heading)
                '''Ensuring entire path primitive is within suitable terrain'''
                flag = 0
                for t in range(int(primitive.getTotalTime())):
                    for dt in range(1):
                        (i,j) = primitive.getPointAtTime((t+dt*0.1))
                        if (int(map._terrainDistribution[int(j)][int(i)]) != agent.terrainCode):
                            flag = 1
                if (flag == 0):
                    checkPoints = True

            timeLeft -= time
            point = primitive.getEndPoint()
            minSpeed = max(speed - agent.acceleration * time, agent.minSpeed)
            maxSpeed = min(speed + agent.acceleration * time, agent.maxSpeed)
            speed = rand.uniform(minSpeed, maxSpeed)
            primitivePath = np.append(primitivePath, [primitive])

        return (PathPrimitive(primitivePath),True)

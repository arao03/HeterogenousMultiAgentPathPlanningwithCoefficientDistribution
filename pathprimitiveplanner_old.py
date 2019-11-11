import operator
import math
from map import Map
import numpy as np
import random as rand
import mathlib
from agent import Agent
from primitiveline import PrimitiveLine
from primitivecurve import PrimitiveCurve
from pathprimitive import PathPrimitive
import pygame
import enum
import gpupdate


class PrimitivePathPlanner():

    def __init__(self, replanTime, tries, generators, initialMap):
        self._replanTime = replanTime
        self._replanTimeCurrent = 0
        self.splitMapPaths = [[],[],[]]
        self._tries = tries
        self._splitMapAgents = [[],[],[]]
        self._generators = generators
        self.observations = []
        self.recordedObservations = []
        self.observationDuration = 20
        self.infoMap = []
        self.initialMap = initialMap


    def getAllAgents(self):
        agents = []
        for agentList in self._splitMapAgents:
            for agent in agentList:
                agents.append(agent)
        return agents

    def reportObservation(self, agent, observation):
        ''' records a new observation in the path planner
        :param agent:
        :param observation:
        :return:
        '''
        (data, type) = observation
        if( type == 1):
            #print("New observation from agent " + str(agent.id) + ": " + str(observation))
            self.recordedObservations.append(observation)
        self.observations.append((data,0))

    def registerNewAgent(self, agent):
        ''' Registers a new agent to plan paths for
        :param agent: (Agent) the new agent to register
        '''
        self._splitMapAgents[agent.splitMapIndex].append(agent)

    def replanPaths(self, map, time):
        ''' Replans paths for all registered agents for map, considering all current observations
        :param map: (Map) map to plan on
        '''
        #time = pygame.time.get_ticks() / 1000
        clipTime = time - self.observationDuration
        self.observations = [observation for observation in self.observations if observation[0][0] >= clipTime]
        dataArray = []
        stateArray = []
        for observation in self.observations:
            (data, state) = observation
            dataArray.append(data)
            stateArray.append(state)

        nx = (map.sizeX - 1) * map.dX
        ny = (map.sizeY - 1) * map.dY
        x = np.linspace(0, nx, map.sizeX)
        y = np.linspace(0, ny, map.sizeY)
        X, Y = np.meshgrid(x, y)
        ell = np.array([5, 5, 5])

        if( len(dataArray) > 0):
            updateMap = gpupdate.GPUpdate(self.initialMap._distribution, X,Y,np.matrix(dataArray),np.array(stateArray), time, ell, 1)
            updateMap = np.transpose(updateMap)
            map.updateMapDistribution( updateMap / updateMap.sum())

        self.splitMapPaths = self.generatePrimitivePaths(map, self._splitMapAgents, self._tries)

        for splitMapIndex in range(len(self._splitMapAgents)):
            agentList = self._splitMapAgents[splitMapIndex]
            for i in range(len(agentList)):
                agentList[i].setNewPath(self.splitMapPaths[splitMapIndex][i])

        self.infoMap = Map(map.sizeX, map.sizeY)
        self.infoMap.updateMapDistribution( self.generateCurrentInfoMap(map, 1.5) )


    def tick(self, deltaTime, world):
        prePlanTime = pygame.time.get_ticks()
        self._replanTimeCurrent += deltaTime
        if( self._replanTimeCurrent >= self._replanTime):
            self.replanPaths(world.map, world.getTimePassed())
            self._replanTimeCurrent = 0
        postPlanTime = pygame.time.get_ticks()
        return (postPlanTime - prePlanTime)/1000

    def getAllPaths(self):
        paths = []
        for splitMapIndex in range(len(self.splitMapPaths)):
            for pathIndex in range(len(self.splitMapPaths[splitMapIndex])):
                paths.append(self.splitMapPaths[splitMapIndex][pathIndex])
        return paths

    def generatePrimitivePaths(self, map, splitMapAgents, tries):
        ''' Returns an array of primitive paths (an array of primitives) that take pathTime time for each agent.
            path at index i corresponds to agent i in agents array.

            Args:
                map         (Map): Map to generate path on
                pathTime    (float): time path traversal should take
                agents      (Agent array): the agents to generate path for
                tries       (int): how many tries should we do for the path
                                   (we pick best one using ergodicity calculation)
        '''


        '''bestSplitMapPaths = [[],[],[]]
        for splitMapIndex in range(len(splitMapAgents)):
            minErg = -1
            agentList = splitMapAgents[splitMapIndex]
            tries = 1
            for trial in range(tries * len(agentList)):
                paths = []
                for i in range(len(agentList)):
                    currentAgent = agentList[i]
                    generator = self._generators[currentAgent.generatorIndex]
                    inBounds = False
                    path = PathPrimitive([])
                    while inBounds == False:
                        (path, inBounds) = generator.generateRandomPrimitivePath(map, currentAgent)
                    paths.append(path)

                infoMap = self.generateInfoMapFromPrimitivePaths(map, 5, agentList, paths)
                erg = mathlib.calcErgodicity(map, infoMap, splitMapIndex, 15)
                if minErg < 0 or erg < minErg:
                    minErg = erg
                    bestSplitMapPaths[splitMapIndex] = np.array(paths)
            print(minErg)
        return np.array(bestSplitMapPaths)'''

        bestSplitMapPaths = [[],[],[]]
        originalMap = Map(map.sizeX,map.sizeY,map.dX,map.dY,50,map._distribution)
        tries = 15
        for splitMapIndex in range(len(splitMapAgents)):
            print("Start new")
            minErg = -1
            agentList = splitMapAgents[splitMapIndex]
            lowestx = 100
            highestx = 0
            #print("Number of trials: " + str(tries))
            print("Length of agent list: " + str(len(agentList)))
            for trial in range(tries * len(agentList)):
                #print("New trial")
                paths = []
                for i in range(len(agentList)):
                    map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)

                    #print("new agent " + str(i))
                    '''
                    print("Map info (x): " + str(len(map._distribution)) + ", " + str(map.sizeY))
                    print("Map info (y): " + str(len(map._distribution[0])) + ", " + str(map.sizeX))
                    print("Map slicing coordinates: " + str(lowestx) + ", " + str(highestx))
                    '''
                    currentAgent = agentList[i]
                    generator = self._generators[currentAgent.generatorIndex]
                    for n in range(1):
                        inBounds = False
                        count = 0
                        path = PathPrimitive([])
                        #print("Path finding level " + str(n))
                        while inBounds == False:
                            count += 1
                            #print("Gets to here")
                            if (count%500 == 0):
                                map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                                #print("Overshoot " + str(count))
                            #print("Before finding path" + str(n))
                            (path, inBounds) = generator.generateRandomPrimitivePath(map, currentAgent)
                            #print("Found path")
                            #if (count%1000 == 0):
                             #   print("stuck here")
                        lowestx = 100
                        lowesty = 100
                        highestx = 0
                        highesty = 0
                        for x in range(int(path.getTotalTime())):
                            (currentx, currenty) = path.getPointAtTime(float(x))
                            if currentx > highestx:
                                highestx = currentx
                            if currentx < lowestx:
                                lowestx = currentx
                            if currenty > highesty:
                                highesty = currenty
                            if currenty < lowesty:
                                lowesty = currenty
                        '''
                        print("Map size: " + str(map.sizeX) + ", " + str(map.sizeY))
                        print("Starting coordinate is: " + str(int(lowestx)) + ", " + str(int(lowesty)))
                        print("Test value before slice: " + str(map._distribution[0][0]))
                        print("Sum of slice: " + str(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)].sum()))
                        print("Coordinates for slice: lowx = " + str(lowestx) + ", highx = " + str(highestx) + ", lowy = " + str(lowesty) + ", highy = " + str(highesty))
                        '''

                        #Changes map back to original map if either dimension is 0
                        if (len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)]) == 0):
                            #print("Size of map was 0")
                            map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                        elif (len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)][0]) == 0):
                            map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                        else:
                            map = Map(abs(int(highestx)-int(lowestx)), abs(int(highesty)-int(lowesty)), map.dX, map.dY, 50, originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)])

                        #Changes map back to original map if sum of probabilities in map is 0.0
                        if (originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)].sum() == 0.0):
                            map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                            #print("Changed to original map")
                            #print("Slice:" + str(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)]))
                        else:
                            map = Map(abs(int(highestx)-int(lowestx)), abs(int(highesty)-int(lowesty)), map.dX, map.dY, 50, originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)])

                        #Changes map back to original map if only one value is none-zero
                        #if (originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)].sum() >= 0.99999):
                        flag = 0
                        for i in range(len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)])):
                            if (originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)][i].sum() == 0.0):
                                flag += 1
                        if flag >= (len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)]) - 3):
                            #print("Only one none zero value in map")
                            map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                        else:
                            map = Map(abs(int(highestx)-int(lowestx)), abs(int(highesty)-int(lowesty)), map.dX, map.dY, 50, originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)])
                        #Changes map back to original map if map dimensions goes below 5x5
                        if(len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)]) > 0):
                            if ((len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)]) * len(originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)][0])) < 25):
                                #print("Map got too small")
                                map = Map(originalMap.sizeX, originalMap.sizeY, originalMap.dX, originalMap.dY, 50, originalMap._distribution)
                        else:
                            map = Map(abs(int(highestx)-int(lowestx)), abs(int(highesty)-int(lowesty)), map.dX, map.dY, 50, originalMap._distribution[int(lowesty):int(highesty), int(lowestx):int(highestx)])



                        #print("Test value from distribution: " + str(map._distribution[0][0]))
                        #print(len(map._distribution))

                    paths.append(path)
                infoMap = self.generateInfoMapFromPrimitivePaths(originalMap, 5, agentList, paths)
                erg = mathlib.calcErgodicity(originalMap, infoMap, splitMapIndex, 15)
                if minErg < 0 or erg < minErg:
                    minErg = erg
                    bestSplitMapPaths[splitMapIndex] = np.array(paths)
            print(minErg)
        return np.array(bestSplitMapPaths)


    def generateCurrentInfoMap(self, map, sampleTime):
        '''
        Generates a info map that is created from all of the currently assigned paths
        :param map: (Map): the map to generate info map for
        :param sampleTime:  (float): how often we should sample a point on the path for the info map
        :return: (Map): generated info map
        '''
        infoMap = np.zeros((map.sizeX, map.sizeY))
        for splitMapIndex in range(len(self.splitMapPaths)):
            if(len(self._splitMapAgents[splitMapIndex]) > 0):
                tempMap = self.generateInfoMapFromPrimitivePaths(map, sampleTime
                                                                ,self._splitMapAgents[splitMapIndex]
                                                                ,self.splitMapPaths[splitMapIndex])
                infoMap += tempMap
        return infoMap

    def generateInfoMapFromPrimitivePaths(self, map, sampleTime, agents, paths ):
        ''' Returns an info map corresponding to the given primitive paths.
            agents[i] should correspond to the path in paths[i] to work correctly

            Args:
                map         (Map): the map to generate info map for
                sampleTime  (float): how often should we sample a point on the path for the info map
                agents      (Agent array): agents to get data from
                paths       (BasePrimitive array array): paths to get info map from
        '''
        infoMap = np.zeros((map.sizeX, map.sizeY))
        for i in range(len(agents)):
            agentInfoMap = self.generateInfoMapFromPrimitivePath(map, sampleTime, agents[i], paths[i])
            infoMap += agentInfoMap
        return infoMap/len(agents)

    def generateInfoMapFromPrimitivePath(self, map, sampleTime, agent, path):
        ''' Returns an info map corresponding to the given primitive paths.
            agents[i] should correspond to the path in paths[i] to work correctly

            Args:
                map         (Map): the map to generate info map for
                sampleTime  (float): how often should we sample a point on the path for the info map
                agent      (Agent): agent to get data from
                path       (BasePrimitive array): path to get info map from
        '''
        infoMap = np.zeros((map.sizeX, map.sizeY))
        pTime = 0
        pIndex = 0
        samples = 0;

        while( pIndex < len(path.primitiveList) ):
            primitive = path.primitiveList[pIndex]
            (pX,pY) = primitive.getPointAtTime(pTime)
            (x,y) = map.worldToMap(pX, pY)

            #TODO: Maybe we can make this in matricial form? Not sure if it will be faster though
            for vX in range (0, len(agent.visionMap)):
                for vY in range (0, len(agent.visionMap[0])):
                    mapX = x + vX - len(agent.visionMap)//2
                    mapY = y + vY - len(agent.visionMap[0])//2
                    if  mapX >= 0 and mapX < len(map.getDistribution()) and mapY >= 0 and mapY < len(map.getDistribution()[0]):
                        infoMap[mapX][mapY] += (agent.visionMap[vX][vY])

            samples += 1
            pTime += sampleTime
            while pTime >= primitive.getTotalTime():
                pTime -= primitive.getTotalTime()
                pIndex += 1
                if pIndex >= len(path.primitiveList):
                    break
                else:
                    primitive = path.primitiveList[pIndex]

        return (infoMap / samples)

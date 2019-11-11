from map import Map
from entity import Entity
from entitytarget import EntityTarget
import mathlib
import pygame
import numpy as np
import random as rand
from scipy import fftpack

class Agent(Entity):
    '''Stores information of an agent.
        Attributes:
            startPt           (int, int): starting point of the agent

    '''

    def onSpawned(self, tc=0):
        #self.terrainCode = rand.choice([0,1]) #0 - Ground, 1 - Water, 2 - Trees, 3 - Unrestricted
        #print("Terrain Code: " + str(self.terrainCode))
        self.startPt = self.position
        self.minRadius = 2
        self.heading = 0
        (self.normalSpeed, self.minSpeed, self.maxSpeed, self.acceleration) = (4, 1, 6, 1)
        self.currentSpeed = self.normalSpeed
        self.path = None
        self.sensor_footprint = rand.choice([(2,6,0.45),(8,7,0.15)]) #(8,7,0.15),(2,6,0.45), (5,4,0.3)]) #visionRadius, visionfwhm, findChance); large imprecise footprint, small precise footprint
        ''' changing this in order to implement 2 kinds of sensor footprints'''
        self.visionRadius = self.sensor_footprint[0] #5
        self.visionfwhm = self.sensor_footprint[1] #4
        self.findChance = self.sensor_footprint[2] #0.3
        self.visionMap = mathlib.makeGaussian(2*self.visionRadius,2*self.visionRadius,self.visionfwhm, self.findChance)
        self.probeTime = 0.1
        self.probeTimeCurrent = 0
        self.pathTime = 0
        self.size = rand.choice([5.0, 15.0]) #10.0
        self.drawHeading = True
        self.image = None
        self.generatorIndex = 0
        self.splitMapIndex = 0 #0 - No split map, 1 - Primitive split map, 2- omnidirectional split map
        #storing information about how many and which Fourier coefficients to use
        #self.coefficientInfo = [True, 250] #take coefficients from beginning (lower frequencies?), number of coefficients to use
        #self.coefficientInfo = 0.3 if self.visionRadius < 5 else 0.02
        #self.coefficientInfo = 0.1 if self.visionRadius < 5 else 0.025
        print(self.visionRadius)

    def findValidStartingPoint(self, map, rangeX, rangeY):
        for i in range(0,rangeX//5):
            for j in range(0,rangeY//5):
                if (map._terrainDistribution[int(j)][int(i)] == self.terrainCode):
                    return (j,i)
        return (-1,0)

    #TODO: Change search to find valid position closest to centre
    def setStartingPoint(self, map):
        (x,y) = self.position
        if (map._terrainDistribution[int(y)][int(x)] == self.terrainCode):
            return self.position
        else:
            #TODO: implement smarter starting point search
            cornerList = [[(0,map.sizeX//10,1),(0,map.sizeY//10,1)],[(0,map.sizeX//10,1),(map.sizeY-1,map.sizeY-11,-1)],[(map.sizeX-1,map.sizeX-11,-1),(0,map.sizeY//10,1)],[(map.sizeX-1,map.sizeX-11,-1),(map.sizeY-1,map.sizeY-11,-1)]]
            '''Check if there is enough suitable terrain in the corner to search it'''
            cornerListnew = []
            for n in range(len(cornerList)):
                s = cornerList[n]
                (xs,ys) = s
                startx = min(xs[0],xs[1])
                endx = max(xs[0],xs[1])
                starty = min(ys[0],ys[1])
                endy = max(ys[0],ys[1])
                if (map._terrainDistribution[startx:endx,starty:endy] == self.terrainCode).sum() > int(0.5*(endx-startx)*(endy-starty)):
                    cornerListnew.append(s)
            rand.shuffle(cornerListnew)
            print(cornerListnew)
            '''Search for point'''
            for (xs,ys) in cornerListnew:
                for i in range(xs[0],xs[1],xs[2]):
                    for j in range(ys[0],ys[1],ys[2]):
                        if (map._terrainDistribution[int(j)][int(i)] == self.terrainCode):
                            print("Starting point is: " + str(j) + ", " + str(i))
                            return (j,i)
            for i in range(map.sizeX):
                for j in range(map.sizeY):
                    if (map._terrainDistribution[int(j)][int(i)] == self.terrainCode):
                        return (j,i)

    def setStartingPointCentre(self, map):
        (x,y) = self.position
        if (map._terrainDistribution[int(y)][int(x)] == self.terrainCode):
            return self.position
        else:
            for round in range(min(map.sizeX, map.sizeY)//10):
                possiblePoints = []
                for i in range(map.sizeX//2-1-round, map.sizeX//2+round):
                    for j in range(map.sizeY//2 -1-round, map.sizeY//2+round):
                        possiblePoints.append((j,i))
                while possiblePoints:
                    (newx, newy) = rand.choice(possiblePoints)
                    if (map._terrainDistribution[newy][newx] == self.terrainCode):
                        return (newy, newx)
                    possiblePoints.remove((newx,newy))



    def onActivated(self):
        pass#self.image = pygame.image.load("UAV.png").convert_alpha()

    def setNewPath(self, newPath):
        self.path = newPath
        self.pathTime = 0

    def tick(self, deltaTime, world):
        if (len(self.path.primitiveList) > 0 and self.path.getTotalTime() > self.pathTime):
            self.pathTime += deltaTime
            self.position = self.path.getPointAtTime(self.pathTime)
            self.rotation = self.path.getHeadingAtTime(self.pathTime)
            self.probeTimeCurrent += deltaTime
            if( self.probeTimeCurrent > self.probeTime):
                self.probeTimeCurrent = 0
                self.attemptObservation(world)

    def attemptObservation(self, world):
        reportFlag = False
        time = world.getTimePassed()
        for ent in world.getAllEntities():
            r = self.visionRadius
            if isinstance(ent, EntityTarget) and self.distanceTo(ent) <= self.visionRadius:
                x = ent.position[0]
                y = ent.position[1]
                (entx_m, enty_m) = world.map.worldToMap(x, y)
                (posx_m, posy_m) = world.map.worldToMap(self.position[0], self.position[1])
                (ent_posx_g, ent_posy_g) = (entx_m + r - posx_m, enty_m + r - posy_m)
                if(min(ent_posx_g, ent_posy_g) >= 0 and max(ent_posx_g, ent_posy_g) < 2*r):
                    prob_detected = self.findChance * self.visionMap[ent_posx_g][ent_posy_g] / self.visionMap[r][r]
                    if np.random.uniform() < prob_detected and ent.spotted == False:
                        world.pathPlanner.reportObservation(self, ([time, x, y], 1))
                        ent.spot()
                        reportFlag = True
        if reportFlag == False:
            x = self.position[0]
            y = self.position[1]
            world.pathPlanner.reportObservation(self, ([time, x, y], 0))

    def getInfoUnderMap(self, map):
        infoVal = 0
        visionMap = self.visionMap / np.max(self.visionMap)
        for vX in range(0, len(visionMap)):
            for vY in range(0, len(visionMap[0])):
                (x,y) = map.worldToMap(self.position[0], self.position[1])
                mapX = x + vX - len(visionMap) // 2
                mapY = y + vY - len(visionMap[0]) // 2
                if mapX >= 0 and mapX < len(map.getDistribution()) and mapY >= 0 and mapY < len(map.getDistribution()[0]):
                    infoVal += (visionMap[vX][vY] * map.getDistribution()[mapX][mapY])
        #print("Info is: " + str(infoVal))
        return infoVal

    '''def truncate_descriptor(self, descriptors, degree):
        #truncates an unshifted fourier descriptor array and returns one also unshfited
        newDiscriptors = np.zeros(len(descriptors))
        descriptors = np.fft.fftshift(descriptors)
        center_index = len(descriptors) // 2
        newDiscriptors[center_index - degree // 2:center_index + degree // 2] = descriptors[center_index - degree // 2:center_index + degree // 2]
        newDiscriptors[0:center_index - degree // 2] = 0
        newDiscriptors[center_index + degree // 2 - 1] = 0
        # descriptors = np.fft.ifftshift(descriptors)
        newDiscriptors = np.fft.ifftshift(newDiscriptors)
        return newDiscriptors

    def keepLowerCoefficients(self, map, keep_fraction):
        #print("Keeping Lower Coefficients")
        im_fft2 = map.copy()
        r, c = map.shape
        im_fft2[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        im_fft2[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        reconstructedmap = fftpack.ifft2(im_fft2).real
        return reconstructedmap

    def keepHigherCoefficients(self, map, keep_fraction):
        #print("Keeping Higher Coefficients")
        im_fft2 = map.copy()
        r, c = map.shape
        im_fft2[0:1] = 0
        im_fft2[:, 0:1] = 0
        im_fft2[int(r)-1: int(r)] = 0
        im_fft2[:, int(c)-1: int(c)] = 0
        im_fft2[1+int(r*keep_fraction):int(r*(1-keep_fraction))-1, 1+int(c*keep_fraction):int(c*(1-keep_fraction))-1] = 0
        reconstructedmap = fftpack.ifft2((im_fft2)).real
        return reconstructedmap

    def keepCoefficients(self, map):
        if self.coefficientInfo < 0.05:
            return self.keepLowerCoefficients(map, self.coefficientInfo)
        else:
            return self.keepHigherCoefficients(map, self.coefficientInfo)'''

    def primitiveCoarse(self, map_fft, keep_fraction=0.035, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        map_fft[0:int(r*keep_fraction), 0:int(c*keep_fraction)] = 0
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def primitiveFocused(self, map_fft, keep_fraction=0.05, inner_corner=1):
        r, c = map_fft.shape
        #map_fft[0:inner_corner] = 0
        #map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def omniCoarse(self, map_fft, keep_fraction=0.075, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        '''map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        map_fft[0:int(r*keep_fraction), int(c*(1-keep_fraction)):int(c)] = 0'''
        '''map_fft[0:inner_corner] = 0
        map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0
        map_fft[int(r*(1-keep_fraction))-inner_corner:int(r)-inner_corner,inner_corner:inner_corner+int(c*(keep_fraction))] = 0'''
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def omniFocused(self, map_fft, keep_fraction=0.035, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r * keep_fraction):int(r * (1 - keep_fraction))] = 0
        map_fft[:, int(c * keep_fraction):int(c * (1 - keep_fraction))] = 0
        map_fft[int(c * (1 - keep_fraction)):int(c), 0:int(r * keep_fraction)] = 0
        '''map_fft[0:inner_corner] = 0
        map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0
        map_fft[inner_corner:inner_corner+int(r*keep_fraction), inner_corner:inner_corner+int(c*keep_fraction)] = 0'''

        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def primitiveCoarse2(self, map_fft, keep_fraction=0.035, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r) - inner_corner:int(r)] = 0
        map_fft[:, int(c) - inner_corner:int(c)] = 0
        map_fft[inner_corner + int(r * keep_fraction):int(r * (1 - keep_fraction)) - inner_corner] = 0
        map_fft[:, inner_corner + int(c * keep_fraction):int(c * (1 - keep_fraction)) - inner_corner] = 0
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def primitiveFocused2(self, map_fft, keep_fraction=0.05, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r) - inner_corner:int(r)] = 0
        map_fft[:, int(c) - inner_corner:int(c)] = 0
        map_fft[inner_corner + int(r * keep_fraction):int(r * (1 - keep_fraction)) - inner_corner] = 0
        map_fft[:, inner_corner + int(c * keep_fraction):int(c * (1 - keep_fraction)) - inner_corner] = 0
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def omniCoarse2(self, map_fft, keep_fraction=0.05, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r * keep_fraction):int(r * (1 - keep_fraction))] = 0
        map_fft[:, int(c * keep_fraction):int(c * (1 - keep_fraction))] = 0
        map_fft[int(r*(1-keep_fraction))-inner_corner:int(r)-inner_corner, int(c*(1-keep_fraction))-inner_corner:int(c)-inner_corner] = 0
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def omniFocused2(self, map_fft, keep_fraction=0.05, inner_corner=1):
        r, c = map_fft.shape
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        '''map_fft[0:inner_corner] = 0
        map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0
        map_fft[inner_corner:inner_corner+int(r*keep_fraction), inner_corner:inner_corner+int(c*keep_fraction)] = 0'''
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def focused(self, map_fft, keep_fraction=0.035, inner_corner=1):
        r, c = map_fft.shape
        '''#distribution 1
        map_fft[0:inner_corner] = 0
        map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0'''
        #distribution 2
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        '''#distribution 3
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0
        map_fft[0:int(r*keep_fraction), 0:int(c*keep_fraction)] = 0'''
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map

    def coarse(self, map_fft, keep_fraction=0.1, inner_corner=1):
        r, c = map_fft.shape
        map_fft[0:inner_corner] = 0
        map_fft[:,0:inner_corner] = 0
        map_fft[int(r)-inner_corner:int(r)] = 0
        map_fft[:, int(c)-inner_corner:int(c)] = 0
        map_fft[inner_corner+int(r*keep_fraction):int(r*(1-keep_fraction))-inner_corner] = 0
        map_fft[:, inner_corner+int(c*keep_fraction):int(c*(1-keep_fraction))-inner_corner] = 0
        '''#distribution 1
        map_fft[int(r*keep_fraction):int(r*(1-keep_fraction))] = 0
        map_fft[:, int(c*keep_fraction):int(c*(1-keep_fraction))] = 0'''
        reconstructed_map = fftpack.ifft2(map_fft).real
        return reconstructed_map


    #Distribution based on sensor
    '''def distributeAgent(self, map_fft):
        if self.visionRadius < 5:
            return self.focused(map_fft)
        else:
            return self.coarse(map_fft)'''

    #Distribution based on sensor + path generator
    def distributeAgent(self, map_fft):
        if self.visionRadius < 5:
        #if self.size > 10.0:
            if self.generatorIndex == 0:
                return self.omniFocused(map_fft)
            else:
                return self.primitiveFocused(map_fft)
        else:
            if self.generatorIndex == 0:
                return self.omniCoarse(map_fft)
            else:
                return self.primitiveCoarse(map_fft)

    #Distribution based on sensor + path generator + size
    '''def distributeAgent(self, map_fft):
        if self.visionRadius < 5:
            if self.size > 10.0:
                if self.generatorIndex == 0:
                    return self.omniFocused(map_fft)
                else:
                    return self.primitiveFocused(map_fft)
            else:
                if self.generatorIndex == 0:
                    return self.omniFocused2(map_fft)
                else:
                    return self.primitiveFocused2(map_fft)
        else:
            if self.size > 10.0: #self.size <= 10.0 -> V1; self.size > 10.0 -> V2
                if self.generatorIndex == 0:
                    return self.omniCoarse(map_fft)
                else:
                    return self.primitiveCoarse(map_fft)
            else:
                if self.generatorIndex == 0:
                    return self.omniCoarse2(map_fft)
                else:
                    return self.primitiveCoarse2(map_fft)'''




    def printDetails(self):
        print("Sensor footprint: " + str(self.sensor_footprint))
        print("Generator: " + str(self.generatorIndex))
#111

from multiprocessing import Pool
import queue
import numpy as np
import main
from map import Map
from mapsimulation import SimulationData
from pathprimitiveplanner import PrimitivePathPlanner
from entitymanager import EntityManager
import pygame
from pygame.locals import *
import random
from entitytarget import EntityTarget
from agent import Agent
from pathgeneratorsmooth import PathGeneratorSmooth
from pathgeneratorstraight import PathGeneratorStraight
import os, sys


def run_simulations(next):
    (a,i,j,k,l,o) = next
    print('Starting simulation ({},{},{},{},{})'.format(i,j,k,l,o))
    s = main.RunSimulation(a)
    print('Saving output of simulation ({},{},{},{},{})'.format(i,j,k,l,o))
    print("Saving to " + str(o)+"/"+str(i)+"/"+str(j)+"/"+str(k)+"/"+str(l)+".txt")
    f = open(str(o)+"/"+str(i)+"/"+str(j)+"/"+str(k)+"/"+str(l)+".txt",'w')
    f.write(s)
    f.close()
    print('Finished simulation ({},{},{},{},{})'.format(i,j,k,l,o))


def run_threads(start_map, stop_map, num_threads):
    for i in range(start_map, stop_map): #33 maps
        taskList = []
        print('Adding map {} to the queue...'.format(i), end='')
        if i < 13:
            m = Map(100, 100, 1, 1, 50, np.loadtxt("saved data maps/notrand" + str(i + 1)+".txt")[:100])
        else:
            m = Map(100, 100, 1, 1, 50, np.loadtxt("saved data maps/rand" + str(i - 12)+".txt")[:100])
        for j in range(1): #10 sets of targets
            dk = []
            for ll in range (10):
                dk.append(m.getRandomPoint(False))
            for k in range(1): # 3 sets of agent setup
                for l in range(1): # 20 trials per setup
                    for o in range(1): # split map or not
                        path_time, no_samples, replan_ratio = 50, 15, 0.5
                        generator = PathGeneratorSmooth(0.2, (1, 5), (8, 12), path_time) # straightChance, pathLengthRange, radiusRange, pathTime
                        #generator = PathGeneratorSmooth(0.2, (1, 10), (15, 25), path_time) # WIDE # straightChance, pathLengthRange, radiusRange, pathTime
                        generator2 = PathGeneratorStraight(path_time)
                        testPlanner = PrimitivePathPlanner(replan_ratio * path_time, no_samples, [generator,generator2], m) # replanTime, tries, generators, initialMap
                        testEntityManager = EntityManager()

                        for ii in range(10):  # setting each agent
                            agent = testEntityManager.spawnEntity(Agent, (50, 50), 0)
                            testPlanner.registerNewAgent(agent)
                            if ii < 5 : # <7 for primitive-spot_omni-details, <3
                                agent.mapSplitIndex = 2 #1, 2
                            else :
                                agent.mapSplitIndex = 1 #2, 1
                            if k == 0 :
                                if ii < 2 :
                                    agent.generatorIndex = 0
                                else :
                                    agent.generatorIndex = 1
                            elif k == 1 :
                                if ii < 5 :
                                    agent.generatorIndex = 0
                                else :
                                    agent.generatorIndex = 1
                            elif k == 2 :
                                if ii < 8 :
                                    agent.generatorIndex = 0
                                else :
                                    agent.generatorIndex = 1
                            if o == 0 :
                                agent.mapSplitIndex = 0

                        for jj in range(10): # putting targets into entitymanager
                            e = testEntityManager.spawnEntity(EntityTarget, dk[jj], 0)

                        taskList.append((SimulationData(m, testPlanner, testEntityManager),i,j,k,l,o))
        random.shuffle(taskList)
        print('OK')

        p = Pool(num_threads)
        p.map(run_simulations, taskList)


if __name__ == "__main__":
    num_threads = 1
    start_map, stop_map = 0, 1

    if len(sys.argv) == 2:
        start_map    = int(sys.argv[1])
        stop_map     = start_map + 1
    elif len(sys.argv) == 3:
        start_map    = int(sys.argv[1])
        stop_map     = int(sys.argv[2])
    elif len(sys.argv) == 4:
        start_map    = int(sys.argv[1])
        stop_map     = int(sys.argv[2])
        num_threads  = int(sys.argv[3])

    print('Starting tests from map {:d} until map {:d} with {:d} threads...'.format(start_map, stop_map, num_threads))
    run_threads(start_map, stop_map, num_threads)

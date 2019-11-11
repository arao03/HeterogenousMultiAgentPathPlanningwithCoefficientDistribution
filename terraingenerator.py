import os, sys
import numpy as np
import matplotlib.pyplot as plt

'''Generates 100x100 terrain information map
    0 - ground, 1 - water, 2 - trees
'''

def generateTerrain(numberOfMaps):
    for n in range(numberOfMaps): #Number of maps
        print("Generating terrain" + str(n))
        t = np.zeros([100, 100])
        for i in range(100):
            for j in range(100):
                #TODO: modify how terrain is changed from 0
                if (i >= 0) and (i <= 40) and (j >= 0) and (j <= 40):
                    t[i][j] = 1
                if (i >= 60) and (i <= 100) and (j >= 60) and (j < 100):
                    t[i][j] = 1
                #if (i >= 0) and (i < 55) and (j >= 55) and (j < 100):
                #    t[i][j] = 1
        np.savetxt('terrainmaps/' + str(n) + '.txt', t, fmt='%.2e')
        plt.imshow(t)
        plt.show()
    print("OK")

generateTerrain(1)
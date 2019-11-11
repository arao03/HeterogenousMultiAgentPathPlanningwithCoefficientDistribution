import numpy as np
import random
from itertools import product
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C


#def makeGaussian(sizeX, sizeY, fwhm=3, center=None, normalize=True):
def makeGaussiannew(sizeX, sizeY):
    """
    Make a rectangular gaussian kernel.
    Args:
            size       ((int,int)): is the width and height of the
            normalize  (bool): should all values in distribution sum up to 1
            fwhm       (int): is full-width-half-maximum, which
                              can be thought of as an effective radius.
            center     ((int,int)): the center of the gaussian distribution
    Example: makeGaussian((5,3)false,2,[2,2])
    """

    '''N_1d = 10
    #range = np.linspace(-5, 5, N_1d)
    xs = np.linspace(0,5,N_1d)
    ys = np.linspace(0,5,N_1d)
    xx, yy = np.meshgrid(xs, ys)
    x = [np.ravel(xx.T), np.ravel(yy.T)]
    N = np.size(x, 1)
    mu = np.zeros(N)

    sigma = np.zeros([N, N])
    for j in range(N):
        for k in range(N):
            xa = x[0][j]
            ya = x[1][j]
            xp = x[0][k]
            yp = x[1][k]
            sigma[j][k] = covariance([xa, ya], [xp, yp]) + covariance([ya, xa], [xp, yp]) + covariance([xa, ya], [yp, xp]) + covariance([ya, xa], [yp, xp])

    f = np.random.multivariate_normal(mu, sigma)
    f = f.reshape((N_1d,N_1d))
    print(f.shape)
    #plt.imshow(f)
    fig = plt.figure()
    ax = fig.gca(projection = '3d')
    surf = ax.plot_surface(xx,yy,f)
    plt.show()'''

    x1 = np.linspace(0,sizeX)
    print(x1.shape)
    x2 = np.linspace(0,sizeY)
    x = (np.array([x1, x2])).T
    #X = np.array([[1.,0.], [3.,0.], [5.,0.], [6.,2.], [7.,2.], [8.,2.]])
    X = random.sample(list(x), int(.7*x.shape[0]))
    #X = np.vstack((x1,x2)).T

    #y = functionToPredict(X).ravel()
    y = np.array([])
    for xs in X:
        y = np.append(y,functionToPredict(xs,sizeX/2, sizeY/2))

    #x = np.atleast_2d(np.linspace(0,10,1000)).T

    kernel = C(1.0, (1e-3,1e3)) * RBF(10, (1e-2,1e2))
    gp = GaussianProcessRegressor(kernel=kernel, alpha=0.15, n_restarts_optimizer=3)
    gp.fit(X,y)

    x1x2 = np.array(list(product(x1, x2)))
    y_pred, sigma = gp.predict(x1x2, return_std=True)


    X0p, X1p = x1x2[:,0].reshape(50,50), x1x2[:,1].reshape(50,50)
    Zp1 = np.reshape(y_pred/y_pred.sum(), (50,50))
    #Zp1 = np.reshape(y_pred+0.95*sigma, (50,50))
    #Zp2 = np.reshape(y_pred-0.95*sigma, (50,50))


    print((y_pred/y_pred.sum()).sum())
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')
    surf1 = ax.plot_surface(X0p, X1p, Zp1)
    #surf2 = ax.plot_surface(X0p, X1p, Zp2)
    plt.show()


def covariance(x, y):
    d1 = x[0] - y[0]
    d2 = x[1] - y[1]
    sqdist = d1**2 + d2**2
    c = np.exp(-0.5*sqdist)
    return c

def functionToPredict(x, x0=5., y0=5., fwhm=3):
    return np.exp(-4 * np.log(2) * ((x[0] - x0) ** 2 + (x[1] - y0) ** 2) / fwhm ** 2)

makeGaussiannew(15,15)


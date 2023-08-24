import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import sleep
import sys
import glob

def find(elt,list):
    for i in range(len(list)):
        if list[i] == elt:
            return i
    return len(list)

path = sys.argv[1]
list_gp = glob.glob(path+"/gp_*.dat")
print(list_gp)
n_rows_plot = int(len(list_gp)/3)+1
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
inc_r = 0
inc_c = 0

#memory error solved!!!
for index in range(len(list_gp)-2):
    
    i = find(path+"gp_"+str(index)+".dat",list_gp)
    
    data = np.loadtxt(list_gp[i], skiprows=1)
    samples = np.loadtxt(path+"/samples.dat", skiprows=1)
    observations = np.loadtxt(path+"/observations.dat", skiprows=1)
    
    dim = int(np.sqrt(len(data[:,0])))
    Xt = np.reshape(data[:,0],(dim,dim))
    X = 0.8*np.ones(dim) + Xt #sensitivity
    Yt = np.reshape(data[:,1],(dim,dim))
    Y = (np.ones(dim) + Yt)*X #acceleration
    Z = np.reshape(data[:,2],(dim,dim))
    sigma = np.reshape(data[:,2] + data[:,3],(dim,dim))


    surface = ax.plot_surface(X,Y,Z,cmap='coolwarm')
    #surface2 = ax.plot_surface(X,Y,sigma,cmap='coolwarm')
    surface2 = ax.plot_surface(X,Y,sigma,cmap='viridis')
    ax.set_xlabel('sensitivity')
    ax.set_ylabel('acceleration')
    ax.set_zlabel('mean of GP')
    ax.set_title("plot for gp_"+str(index)+"'s results")
    
    sampled_s = []
    sampled_acc = []
    for i in range(index+4):
        sampled_s.append(samples[i,1] + 0.8)
        sampled_acc.append(sampled_s[i]*(1 + samples[i,2]))
    ax.scatter(sampled_s,sampled_acc,observations[0:index+4,1],c='purple',s=50)

    # figures[inc_r,inc_c].fill_between(data[:,0], data[:,1] - data[:,2], data[:,1] + data[:,2],alpha=0.2)
    # figures[inc_r,inc_c].plot(samples[0:index + 4,1], observations[0:index + 4,1], 'o')
    # figures[inc_r,inc_c].ylim(-100, 120)
    #figures[inc_r,inc_c].savefig('gp.pdf')
    #figures[inc_r,inc_c].set_title("plot for gp_"+str(index)+"'s results")
    # figures[inc_r,inc_c].set_ylim(-1,1)
    
    if inc_c < 2:
        inc_c += 1
    else:
        inc_c = 0
        inc_r += 1
        

fig.savefig('gp.pdf')

# # Define the function to plot
# def my_function(x, y):
#     return np.sin(np.sqrt(x**2 + y**2))

# # Generate x, y values
# x = np.linspace(-5, 5, 100)
# y = np.linspace(-5, 5, 100)
# X, Y = np.meshgrid(x, y)

# # Compute the function values for each point in the grid
# Z = my_function(X, Y)

# # Create a 3D surface plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# surface = ax.plot_surface(X, Y, Z, cmap='viridis')

# # Add color bar
# fig.colorbar(surface)

# # Set labels for the axes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('3D Surface Plot of z = sin(sqrt(x^2 + y^2))')


plt.show()



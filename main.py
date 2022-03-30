import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Circle

x_list,y_list, r = [10, 30, 50, 70, 90], [10, 30, 50, 70, 90], 5


def plot_cicle(x, y, r, ax):
    cir1 = Circle(xy=(x, y), radius=r, alpha=0.5)
    ax.add_patch(cir1)
    ax.plot(x, y, 'ro')
    plt.axis('scaled')
    plt.axis('equal')  # changes limits of x or y axis so that equal increments of x and y have the same length
    plt.ylim([-50, 150])
    plt.xlim([-50, 150])


def ODS_sim(step_length, body_length, body_width, ini_ori, ini_posx, ini_posy):
    ori, posx, posy, HalfBodyDiag, beta_rad = ini_ori, ini_posx, ini_posy, np.sqrt(body_length^2 + body_width^2), np.atan(body_width/body_length)
    for t in range(200):
        posy += step_length/30
        if t%30 == 0:
            X_LF_absolute = posy + HalfBodyDiag * np.cos(beta_rad - ori)
            X_RB_absolute = posy - HalfBodyDiag * np.cos(beta_rad - ori)
            contact_detect(X_LF_absolute)

def contact_detect(leg_posx, leg_posy, x_list, y_list):
    for i in x_list:
        for j in y_list:
            if np.sqrt((leg_posx - i)^2 + (leg_posy - j)^2) <= 5:
                return i, j
    return 0, 0


def obstacle_force(x, y, leg_posx, leg_posy):
    force_vector = np.array(leg_posx - x, leg_posy - y)/np.sqrt((leg_posx - x)^2 + (leg_posy - y)^2)


fig = plt.figure()
ax = fig.add_subplot(111)
for i in x_list:
    for j in y_list:
        plot_cicle(i,j,r, ax)

plt.show()

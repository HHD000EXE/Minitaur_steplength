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
    for t in range(6000):
        posy += step_length/3000
        if t % 3000 == 0:  # Leg position for LF, RB
            y_LF_absolute = posy + HalfBodyDiag * np.cos(beta_rad - ori)
            x_LF_absolute = posx - HalfBodyDiag * np.sin(beta_rad - ori)
            y_RB_absolute = posy - HalfBodyDiag * np.cos(beta_rad - ori)
            x_RB_absolute = posx + HalfBodyDiag * np.sin(beta_rad - ori)
            x_contact, y_contact = contact_detect([x_LF_absolute, x_RB_absolute], [y_LF_absolute, y_RB_absolute], x_list, y_list, r)

        if t % 3000 == 1500:  # Leg position for RF, LB
            y_RF_absolute = posy + HalfBodyDiag * np.cos(beta_rad + ori)
            x_RF_absolute = posx + HalfBodyDiag * np.sin(beta_rad + ori)
            y_LB_absolute = posy - HalfBodyDiag * np.cos(beta_rad + ori)
            x_LB_absolute = posx - HalfBodyDiag * np.sin(beta_rad + ori)
            x_contact, y_contact = contact_detect([x_RF_absolute, x_LB_absolute], [y_RF_absolute, y_LB_absolute], x_list, y_list, r)





def contact_detect(leg_list_x, leg_list_y, x_list, y_list, r):
    # output contact obstacle list coordinates
    contact_list = []
    for leg_posx, leg_posy in leg_list_x, leg_list_y:
        for i in x_list:
            for j in y_list:
                if np.sqrt((leg_posx - i)^2 + (leg_posy - j)^2) <= r:
                    contact_list += [i, j]
    return contact_list


def obstacle_force(x, y, leg_posx, leg_posy, com_x, com_y):
    force_mag = 10
    # obstacle force direction vector
    force_unit_vector = np.linalg.norm([leg_posx - x, leg_posy - y])
    # magnitude of obstacle force
    force_vector = force_mag * force_unit_vector
    # CoM to obstacle center direction normal vector
    obs_com_unit_vector = np.linalg.norm([com_x - x, com_y - y])
    # Force on x-axis direction
    f = np.dot(np.array(force_vector), np.array(obs_com_unit_vector))
    f_x, f_y = f[0], f[1]

    # normal vector for CoM to obstacle center direction normal vector
    v1, v2 = get_vertical_vector1(obs_com_unit_vector), get_vertical_vector2(obs_com_unit_vector)
    

    return f_x, f_y



def get_vertical_vector1(vec):
    """ normal vector """
    vec = np.linalg.norm(vec)
    return [vec[1], -vec[0]]


def get_vertical_vector2(vec):
    """ normal vector """
    vec = np.linalg.norm(vec)
    return [-vec[1], vec[0]]


###############  plot obstacle  ############
fig = plt.figure()
ax = fig.add_subplot(111)
for i in x_list:
    for j in y_list:
        plot_cicle(i,j,r, ax)

plt.show()
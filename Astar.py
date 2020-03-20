import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.spatial import distance

X0 = np.array((0))
Y0 = np.array((0))
U0 = np.array((2))
V0 = np.array((-2))
fig, ax = plt.subplots()
q0 = plt.quiver(X0, Y0, U0, V0, units='xy', scale = 1, color = 'r', headwidth =1, headlength=0)
Node = [X0+U0, Y0+V0]
print('Node0: ')
print(Node)
X1 = np.array((2))
Y1 = np.array((-2))
U1 = np.array((3))
V1 = np.array((-2))
q1 = plt.quiver(X1, Y1, U1, V1, units='xy', scale=1)
Node1=[X1+U1, Y1+V1]
print('Node1: ')
print(Node1)
X2 = np.array((2))
Y2 = np.array((-2))
U2 = np.array((3))
V2 = np.array((-1))
q2 = ax.quiver(X2, Y2, U2, V2, units = 'xy', scale = 1)
Node2 = [X2+U2, Y2+V2]
print('Node2: ')
print(Node2)
X3 = np.array((2))
Y3 = np.array((-2))
U3 = np.array((-1))
V3 = np.array((3))
q3 = ax.quiver(X3, Y3, U3, V3, units='xy', scale = 1)
Node3 = [X3+U3, Y3+V3]
print('Node3: ')
print(Node3)
plt.grid()
ax.set_aspect('equal')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.title('How to plot a vector in matplotlib ?', fontsize=10)
plt.savefig('how_to_plot_a_vector_in_matplotlib_fig3.png', bbox_inches='tight')
plt.show()
#plt.close()

DIST_THRESHOLD = 1
ANG_THRESHOLD = 30
STEP = 1

start = input("Enter start point coordinates (x,y,theta):- ")
goal = input("Enter goal point coordinates (x,y):- ")
robot_radius = input("Enter robot radius:- ")
clearance = input("Enter clearance value:- ")
step = input("Enter step size (between 1-10):- ")

start = np.array([0,0,30], dtype="float")

def move_straight(start):
    new_node = np.array([0,0,0], dtype="float")
    angle = math.radians(start[2] + 0)
    new_node[0] = round(start[0] + math.cos(angle), 2)
    new_node[1] = round(start[1] + math.sin(angle), 2)
    new_node[2] = start[2] + 0
    return new_node

def move_up_1(start):
    new_node = np.array([0,0,0], dtype="float")
    angle = math.radians(start[2] + ANG_THRESHOLD)
    new_node[0] = round(start[0] + math.cos(angle), 2)
    new_node[1] = round(start[1] + math.sin(angle), 2)
    new_node[2] = start[2] + ANG_THRESHOLD
    return new_node

def move_up_2(start):
    new_node = np.array([0,0,0], dtype="float")
    angle = math.radians(start[2] + ANG_THRESHOLD*2)
    new_node[0] = round(start[0] + math.cos(angle), 2)
    new_node[1] = round(start[1] + math.sin(angle), 2)
    new_node[2] = start[2] + ANG_THRESHOLD*2
    return new_node

def move_down_1(start):
    new_node = np.array([0,0,0], dtype="float")
    angle = math.radians(start[2] - ANG_THRESHOLD)
    new_node[0] = round(start[0] + math.cos(angle), 2)
    new_node[1] = round(start[1] + math.sin(angle), 2)
    new_node[2] = start[2] - ANG_THRESHOLD
    return new_node

def move_down_2(start):
    new_node = np.array([0,0,0], dtype="float")
    angle = math.radians(start[2] - ANG_THRESHOLD*2)
    new_node[0] = round(start[0] + math.cos(angle), 2)
    new_node[1] = round(start[1] + math.sin(angle), 2)
    new_node[2] = start[2] - ANG_THRESHOLD*2
    return new_node

print(start)
print(move_straight(start))
print(move_up_1(start))
print(move_up_2(start))
print(move_down_1(start))
print(move_down_2(start))
print(start)

visited = np.zeros([int(300/DIST_THRESHOLD),int(200/DIST_THRESHOLD),12])
#print(visited)
parent = np.zeros([int(300/DIST_THRESHOLD),int(200/DIST_THRESHOLD),12])
CostToCome = np.zeros([int(300/DIST_THRESHOLD),int(200/DIST_THRESHOLD),12])
Cost = np.array(np.ones([int(300/DIST_THRESHOLD),int(200/DIST_THRESHOLD),12]) * np.inf)

def add_node(new_node, visited):  # math.modf(new_node[0])[0]
    if new_node[0] % 1 > DIST_THRESHOLD / 2:
        i = int(math.ceil(new_node[0]) / DIST_THRESHOLD)
    else:
        i = int(math.floor(new_node[0]) / DIST_THRESHOLD)

    if new_node[1] % 1 > DIST_THRESHOLD / 2:
        j = int(math.ceil(new_node[1]) / DIST_THRESHOLD)
    else:
        j = int(math.floor(new_node[1]) / DIST_THRESHOLD)

    # if new_node[2] >= 360:
    #   new_node[2] = new_node[2] - 360

    if -15 <= new_node[2] <= 15 or 345 < new_node[2] < 360 or -360 < new_node[2] < -345:
        k = 0
    elif 15 < new_node[2] <= 45 or -345 <= new_node[2] < -315:
        k = 1
    elif 45 < new_node[2] <= 75 or -315 <= new_node[2] < -285:
        k = 2
    elif 75 < new_node[2] <= 105 or -285 <= new_node[2] < -255:
        k = 3
    elif 105 < new_node[2] <= 135 or -255 <= new_node[2] < -225:
        k = 4
    elif 135 < new_node[2] <= 165 or -225 <= new_node[2] < -195:
        k = 5
    elif 165 < new_node[2] <= 195 or -195 <= new_node[2] < -165:
        k = 6
    elif 195 < new_node[2] <= 225 or -165 <= new_node[2] < -135:
        k = 7
    elif 225 < new_node[2] <= 255 or -135 <= new_node[2] < -105:
        k = 8
    elif 255 < new_node[2] <= 285 or -105 <= new_node[2] < -75:
        k = 9
    elif 285 < new_node[2] <= 315 or -75 <= new_node[2] < -45:
        k = 10
    elif 315 < new_node[2] <= 345 or -45 <= new_node[2] < -15:
        k = 11

    print(i, j, k)
    visited[i][j][k] = 1
    return visited


def generate_obstacles(robot_radius, clearance):
    margin = robot_radius + clearance

    # make entire map black
    screen = []
    for x in range(301):
        for y in range(201):
            if x >= 0 and x <= 300 and y >= 0 and y <= 200:
                screen.append([x,y])

    X=[col[0] for col in screen]
    Y=[col[1] for col in screen]

    # add obstacles to the map
    obstacle_space = []

    for x in range(301):
        for y in range(201):
            # workspace area walls
            if margin > 0:
                if (y - margin <= 0) or (x - margin <= 0) or (y - (200 - margin) >= 0) or (x - (300 - margin) >= 0):
                    obstacle_space.append([x, y])

            # circle
            if (x - 225) ** 2 + (y - 150) ** 2 <= (25 + margin) ** 2:
                obstacle_space.append([x, y])

            # ellipse
            if ((x - 150) / (40 + margin)) ** 2 + ((y - 100) / (20 + margin)) ** 2 <= 1:
                obstacle_space.append([x, y])

            # rhombus
            if (5 * y - 3 * x + 5 * (95 - margin) <= 0) and (5 * y + 3 * x - 5 * (175 + margin) <= 0) and \
                    (5 * y - 3 * x + 5 * (125 + margin) >= 0) and (5 * y + 3 * x - 5 * (145 - margin) >= 0):
                obstacle_space.append([x, y])

            # rectangle
            if (5 * y - 9 * x - 5 * (13 + margin) <= 0) and (65 * y + 37 * x - 5 * 1247 - 65 * margin <= 0) and \
                    (5 * y - 9 * x + 5 * (141 + margin) >= 0) and (65 * y + 37 * x - 5 * 1093 + 65 * margin >= 0):
                obstacle_space.append([x, y])

            # polygon
            if (y <= 13 * x - 140 + margin) and (y - x - 100 + margin >= 0) and \
                    (5 * y + 7 * x - 5 * 220 <= 0):
                obstacle_space.append([x, y])
            if (y - 185 - margin <= 0) and (5 * y + 7 * x - 5 * (290 + margin) <= 0) and \
                    (5 * y - 6 * x - 5 * (30 - margin) >= 0) and (5 * y + 6 * x - 5 * (210 - margin) >= 0) and \
                    (5 * y + 7 * x - 5 * (220 - margin) >= 0):
                obstacle_space.append([x, y])

    obstacle_space = np.array(obstacle_space)
    x_obs = [col[0] for col in obstacle_space]
    y_obs = [col[1] for col in obstacle_space]
    # return obstacle_space

    # print obstacle map
    # plt.plot(X,Y,'ks')
    plt.scatter(x_obs, y_obs, c='b')
    plt.axis([0, 300, 0, 200])
    # plt.axis('off')
    plt.show()
    return obstacle_space



def draw_map_rigid(obstacle_space,X,Y):
    x_obs=[col[0] for col in obstacle_space]
    y_obs=[col[1] for col in obstacle_space]

    # print obstacle map
    plt.plot(X,Y,'ks')
    plt.plot(x_obs,y_obs,'bs')
    plt.axis([0,300,0,200])
    plt.axis('off')
    plt.show()

radius = int(input("Enter radius of the robot:- "))
clearance = int(input("Enter clearance value:- "))
obs = generate_obstacles(radius, clearance)
#draw_map_rigid(obs[0],obs[1],obs[2])

obstacle_space = generate_obstacles(0, 0)
for x, y in obstacle_space:
    visited[x][y] = 1

# Main Code
def CostToGo(x,y,goal):
    point=[x,y]
    return distance.euclidean(goal,point)
Q = []
Q.append(start)
v[start] = 1
while len(Q) != 0:
    current_node = Q.GetFirst()
    if current_node == goal_node:
        break
        print("Goal Reached")
    new_node = move_straight(current_node)
    # new_node[0], new_node[1], new_node[2] = 2*round(temp[0]), 2*round(temp[1]), 2*(temp[2]/30)
    if v[new_node] == 0:
        v[new_node] = 1
        Q.append(new_node)
        parent[new_node] = current_node
        CostToCome(new_node) = CostToCome(current_node) + 1
        Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
    else:
        if Cost(new_node) > CostToCome(current_node) + 1 + CostToGo(new_node):
            CostToCome(new_node) = CostToCome(current_node) + 1
            Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
            current_node = parent(new_node)

    new_node = move_up_1(current_node)
    # new_node[0], new_node[1], new_node[2] = 2*round(temp[0]), 2*round(temp[1]), 2*(temp[2]/30)
    if v[new_node] == 0:
        v[new_node] = 1
        Q.append(new_node)
        parent[new_node] = current_node
        CostToCome(new_node) = CostToCome(current_node) + 1
        Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
    else:
        if Cost(new_node) > CostToCome(current_node) + 1 + CostToGO(new_node[0],new_node[1],goal):
            CostToCome(new_node) = CostToCome(current_node) + 1
            Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
            current_node = parent(new_node)

    new_node = move_up_2(current_node)
    # new_node[0], new_node[1], new_node[2] = 2*round(temp[0]), 2*round(temp[1]), 2*(temp[2]/30)
    if v[new_node] == 0:
        v[new_node] = 1
        Q.append(new_node)
        parent[new_node] = current_node
        CostToCome(new_node) = CostToCome(current_node) + 1
        Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
    else:
        if Cost(new_node) > CostToCome(current_node) + 1 + CostToGO(new_node[0],new_node[1],goal):
            CostToCome(new_node) = CostToCome(current_node) + 1
            Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
            current_node = parent(new_node)

    new_node = move_down_1(current_node)
    # new_node[0], new_node[1], new_node[2] = 2*round(temp[0]), 2*round(temp[1]), 2*(temp[2]/30)
    if v[new_node] == 0:
        v[new_node] = 1
        Q.append(new_node)
        parent[new_node] = current_node
        CostToCome(new_node) = CostToCome(current_node) + 1
        Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
    else:
        if Cost(new_node) > CostToCome(current_node) + 1 + CostToGO(new_node[0],new_node[1],goal):
            CostToCome(new_node) = CostToCome(current_node) + 1
            Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
            current_node = parent(new_node)

    new_node = move_down_2(current_node)
    # new_node[0], new_node[1], new_node[2] = 2*round(temp[0]), 2*round(temp[1]), 2*(temp[2]/30)
    if v[new_node] == 0:
        v[new_node] = 1
        Q.append(new_node)
        parent[new_node] = current_node
        CostToCome(new_node) = CostToCome(current_node) + 1
        Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
    else:
        if Cost(new_node) > CostToCome(current_node) + 1 + CostToGO(new_node[0],new_node[1],goal):
            CostToCome(new_node) = CostToCome(current_node) + 1
            Cost(new_node) = CostToCome(new_node) + CostToGO(new_node[0],new_node[1],goal)
            current_node = parent(new_node)

return 0

#backtracking
path = []
def back_track(goal, start):
    final_node = goal
    path.append(final_node)
    while (final_node != start):
        cn = parent(final_node)
        path.append(cn)
        final_node = cn
back_track(goal, start)
print('path', path)









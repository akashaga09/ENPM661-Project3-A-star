import numpy as np
import math
import matplotlib.pyplot as plt
import queue

#defining constants
DIST_THRESHOLD = 0.5
ANG_THRESHOLD = 30.0

#defining obstacles using half plane equations
def check_obstacle(x, y, robot_radius, clearance):
    margin = robot_radius + clearance
    
    # workspace area walls
    if margin > 0:
        if (y - margin <= 0) or (x - margin <= 0) or (y - (200 - margin) >= 0) or (x - (300 - margin) >= 0):
            return True

    # circle
    if (x - 225) ** 2 + (y - 150) ** 2 <= (25 + margin) ** 2:
        return True

    # ellipse
    if ((x - 150) / (40 + margin)) ** 2 + ((y - 100) / (20 + margin)) ** 2 <= 1:
        return True

    # rhombus
    if (5 * y - 3 * x + 5 * (95 - margin) <= 0) and (5 * y + 3 * x - 5 * (175 + margin) <= 0) and \
            (5 * y - 3 * x + 5 * (125 + margin) >= 0) and (5 * y + 3 * x - 5 * (145 - margin) >= 0):
        return True

    # rectangle
    elif (5 * y - 9 * x - 5 * (13 + margin) <= 0) and (65 * y + 37 * x - 5 * 1247 - 65 * margin <= 0) and \
            (5 * y - 9 * x + 5 * (141 + margin) >= 0) and (65 * y + 37 * x - 5 * 1093 + 65 * margin >= 0):
        return True

    # polygon
    if (y <= 13 * x - 140 + margin) and (y - x - 100 + margin >= 0) and \
            (5 * y + 7 * x - 5 * 220 <= 0):
        return True
    
    if (y - 185 - margin <= 0) and (5 * y + 7 * x - 5 * (290 + margin) <= 0) and \
            (5 * y - 6 * x - 5 * (30 - margin) >= 0) and (5 * y + 6 * x - 5 * (210 - margin) >= 0) and \
            (5 * y + 7 * x - 5 * (220 - margin) >= 0):
        return True
    
    else:
        return False
    
#generating obstacle map
def create_map(robot_radius, clearance):
    x_obs = []
    y_obs = []
    for x in range(301):
        for y in range(201):
            if check_obstacle(x, y, robot_radius, clearance):
                x_obs.append(x)
                y_obs.append(y)                               
    return x_obs, y_obs
    
#checking if point with in goal radius
def check_goal(x, y, x_g, y_g):
    if (((x_g - x)**2) + ((y_g - y)**2) <= (1.5)**2):
        return True
    else:
        return False

#defining the movements
def action_step(current_node, step, ANG_THRESHOLD):
    ANG_THRESHOLD_rad = math.radians(ANG_THRESHOLD)
    angle = math.radians(current_node[1][2])
    actions = [[step * math.cos(angle), step * math.sin(angle), 0, step],
               [step * math.cos(angle + ANG_THRESHOLD_rad), step * math.sin(angle + ANG_THRESHOLD_rad), ANG_THRESHOLD, step],
               [step * math.cos(angle + (2*ANG_THRESHOLD_rad)), step * math.sin(angle + (2*ANG_THRESHOLD_rad)),
                2*ANG_THRESHOLD, step],
               [step * math.cos(angle - ANG_THRESHOLD_rad), step * math.sin(angle - ANG_THRESHOLD_rad), -ANG_THRESHOLD, step],
               [step * math.cos(angle - (2*ANG_THRESHOLD_rad)), step * math.sin(angle - (2*ANG_THRESHOLD_rad)),
                -2*ANG_THRESHOLD, step]]
    return actions

def discretize(x, y, theta, ANG_THRESHOLD, DIST_THRESHOLD):
    x = (round(x/DIST_THRESHOLD) * DIST_THRESHOLD)
    y = (round(y/DIST_THRESHOLD) * DIST_THRESHOLD)
    theta = (round(theta / ANG_THRESHOLD) * ANG_THRESHOLD)
    return (x, y, theta)

#calculating cost to go
def cost_to_go(point1, point2):
    distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    return distance

def a_star_search(start_point, goal_point, step, robot_radius, clearance, ANG_THRESHOLD, DIST_THRESHOLD):
    x_node =[]
    y_node =[]
    x_parent = []
    y_parent = []
    
    start_point = discretize(start_point[0], start_point[1], start_point[2], ANG_THRESHOLD, DIST_THRESHOLD)
    goal_point = discretize(goal_point[0], goal_point[1], goal_point[2], ANG_THRESHOLD, DIST_THRESHOLD)

    visited = np.zeros([int(300/DIST_THRESHOLD),int(200/DIST_THRESHOLD),int(360/ANG_THRESHOLD)])
    start = (0, start_point, None)
    goal = (0, goal_point, None)

    nodes_explored = queue.PriorityQueue()
    path_dict = {}
    nodes_explored.put(start)

    while True:
        current_node = list(nodes_explored.get())
        current_node[0] = current_node[0] - cost_to_go(current_node[1], goal[1])

        if(current_node[2] != None):
            str1 = str(current_node[1][0])
            str2 = str(current_node[1][1])
            str3 = str(current_node[1][2])

            str4 = str(current_node[2][0])
            str5 = str(current_node[2][1])
            str6 = str(current_node[2][2])

            str_node = str1+','+str2+','+str3
            str_parent = str4+','+str5+','+str6
            path_dict[str_node] = str_parent

        
        else:
            str1 = str(current_node[1][0])
            str2 = str(current_node[1][1])
            str3 = str(current_node[1][2])
            str4 = str(current_node[2])

            str_node = str1+','+str2+','+str3
            str_parent = str4
            path_dict[str_node] = str_parent

        actions = action_step(current_node, step, ANG_THRESHOLD)
        
        for new_node in actions:
            angle = new_node[2] + current_node[1][2]
            if angle < 0:
                angle = angle + 360
                
            if angle > 360:
                angle = angle - 360

            if angle == 360:
                angle = 0
                
            node = (current_node[1][0] + new_node[0], current_node[1][1] + new_node[1], angle)
            node = discretize(node[0], node[1], node[2], ANG_THRESHOLD, DIST_THRESHOLD)
            node_cost = current_node[0] + new_node[3] + cost_to_go(node, goal[1])
            node_parent = current_node[1]
    
            if check_obstacle(node[0], node[1], robot_radius, clearance) == False:
                if (visited[int(node[0] / DIST_THRESHOLD)][int(node[1] / DIST_THRESHOLD)][int(node[2] / ANG_THRESHOLD)] == 0):
                    visited[int(node[0] / DIST_THRESHOLD)][int(node[1] / DIST_THRESHOLD)][int(node[2] / ANG_THRESHOLD)] = 1
                    x_node.append(node[0])
                    y_node.append(node[1])
                    x_parent.append(node_parent[0])
                    y_parent.append(node_parent[1])
                    new_node = (node_cost, node, node_parent)
                    nodes_explored.put(new_node)

        if check_goal(current_node[1][0], current_node[1][1], goal_point[0], goal_point[1]):
            print("GOAL REACHED!!!")
            print("Plotting path...")
            path = []
            
            str_p1 = str(current_node[2][0])
            str_p2 = str(current_node[2][1])
            str_p3 = str(current_node[2][2])
            parent = str_p1+','+str_p2+','+str_p3
            
            while parent != "None": 
                temp = path_dict.get(parent)
                if(parent[1]=='.' and parent[5]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4])+float(parent[6])/10
                if(parent[2]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5]+parent[6])+float(parent[8])/10
                if(parent[1]=='.' and parent[6]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4]+parent[5])+float(parent[7])/10
                if(parent[2]=='.' and parent[6]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5])+float(parent[7])/10
                if(parent[3]=='.' and parent[9]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6]+parent[7]+parent[8])+float(parent[10])/10
                if(parent[3]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6])+float(parent[8])/10
                if(parent[3]=='.' and parent[8]=='.'):
                    par_1 = float(parent[0]+parent[1]+parent[2])+float(parent[4])/10
                    par_2 = float(parent[6]+parent[7])+float(parent[9])/10
                if(parent[1]=='.' and parent[7]=='.'):
                    par_1 = float(parent[0])+float(parent[2])/10
                    par_2 = float(parent[4]+parent[5]+parent[6])+float(parent[8])/10
                if(parent[2]=='.' and parent[8]=='.'):
                    par_1 = float(parent[0]+parent[1])+float(parent[3])/10
                    par_2 = float(parent[5]+parent[6]+parent[7])+float(parent[9])/10
                path.append((par_1,par_2))
                parent = temp
                if((par_1,par_2) == (start_point[0],start_point[1])):
                    break
                
            path.append((start_point[0], start_point[1]))
            
            return path, x_node, y_node, x_parent, y_parent
        
    
#initializing variables
robot_radius = int(input("Enter robot radius:- "))
clearance = int(input("Enter clearance value:- "))

start_point = input("Enter start node coordinates (x,y,theta):- ")
start_point = tuple(map(float, start_point.split(",")))

while check_obstacle(start_point[0], start_point[1], robot_radius, clearance) == True:
    print("Invalid start node!")
    start_point = input("Enter start node coordinates (x,y,theta):- ")
    start_point = tuple(map(float, start_point.split(",")))

goal_point = input("Enter goal node coordinates (x,y):- ")
goal_point = tuple(map(float, goal_point.split(",")))

while check_obstacle(goal_point[0], goal_point[1], robot_radius, clearance) == True:
    print("Invalid goal node!")
    goal_point = input("Enter goal node coordinates (x,y):- ")
    goal_point = tuple(map(float, goal_point.split(",")))

goal_point = (goal_point[0],goal_point[1],0.0)

step = float(input("Enter step size (between 1-10):- "))

#algorithm
path,x_node,y_node,x_parent,y_parent = a_star_search(start_point, goal_point, step, robot_radius, clearance, ANG_THRESHOLD, DIST_THRESHOLD)

if path == None:
    print("No path found!")

else:
    #printing obstacle map
    x_obs, y_obs = create_map(robot_radius, clearance)
    plt.ion()
    plt.plot(x_obs, y_obs, ".b")
    plt.axis([0,300,0,200])

    #printing explored nodes
    length = 0
    while length < len(x_node):
        plt.plot([x_node[length], x_parent[length]], [y_node[length], y_parent[length]], "-k")
        length = length + 1
        plt.show()
        plt.pause(0.000000000000000000000000000000001)

    #printing path
    path = path[::-1]
    x_path = [path[i][0] for i in range(len(path))]
    y_path = [path[i][1] for i in range(len(path))]
    plt.plot(x_path, y_path, "-r")
    plt.show()

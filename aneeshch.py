#################################################################################################
# ENPM661 Project 3: A Star Algorithm 
# Aneesh Chodisetty (UID 117359893) and Bhargav Kumar Soothram (UID 117041088)
#################################################################################################
import cv2
import sys
import time
import copy
import queue
import argparse
import itertools
import threading
import numpy as np
from support_files.node import Node
from support_files.mapping import *
from support_files.actions import *

# Loading bar animation
def loading_bar():

    global animation_complete

    loading_bar = threading.current_thread()
    loading_bar.alive = True

    for symbols in itertools.cycle(['|', '/', '-', '\\']):
        
        if animation_complete or not loading_bar.alive:
            break
        
        sys.stdout.write('\rFinding path.....' + symbols)
        sys.stdout.flush()
        time.sleep(0.1)

# Creates the obstacle space visualization
def get_playground(playground, clearance = 0):
    breadth, length, _ = playground.shape
    for b in range(breadth):
        for l in range(length):
            if is_ptinkite((l, b), clearance) or is_ptinhex((l, b), clearance) or is_ptincircle((l, b), clearance):
                playground[breadth - b, l] = [0, 0, 255]
    print('Select the "Created maze" window and click any button to contiue...')
    cv2.imshow('Created maze', playground)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return playground

# Checks if a given point is in the obstacle space
def is_ObstacleSpace(u, v, clearance):
    psn = (u, v)
    if (0 + clearance <= u <= 399 - clearance) and (0 + clearance <= v <= 249 - clearance) and \
        (is_ptinkite(psn, clearance) == False) and (is_ptinhex(psn, clearance) == False) and \
        (is_ptincircle(psn, clearance) == False):
        return False
    else:
        return True

# Rounds the number to the closest half value
def customRounding(num):
    rounded = round(2*num)/2
    return rounded

# Computes the "distance" of a given state from the goal state
def get_HeuristicCost(state, goal_state):
    if state is not None:
        hcost = np.sqrt((state[0]-goal_state[0])**2 + (state[1]-goal_state[1])**2)
    else:
        hcost = 0.
    return hcost

# Returns true if the cost of the current node is less than the previously held value
def is_visited(node, goal_state, visited_distances, threshold=0.5, theta_increment=30):
    crnt_state = node.get_state()
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    x, y, theta = int(customRounding(x)/threshold), int(customRounding(y)/threshold), int(theta/theta_increment)
    if node.get_cost() + get_HeuristicCost(crnt_state, goal_state) < visited_distances[x, y, theta]:
        return True
    else:
        return False

# Checks if the goal is reached
def is_GoalReached(node, goal_state, threshold_dist=1.5):
    crnt_state = node.get_state()
    distance = np.sqrt((crnt_state[0]-goal_state[0])**2 + (crnt_state[1]-goal_state[1])**2)
    if distance <= threshold_dist:
        return True
    else:
        return False

# Flips the matrix upside down to facilitate plotting
def ImagetoPlotCoords(state, plground):
    height, _ , _ = plground.shape
    U = state[0]
    V = height - 1 - state[1] 
    return [int(U), int(V)]

# Plots the given visited node on the playground
def plot_visited(plground, node, color=(0, 255, 0)):
    height, width , _ = plground.shape
    parent = node.get_parent() 
    crnt_state = node.get_state()
    if parent is not None:
        parent_state = parent.get_state()
        u, v = ImagetoPlotCoords(parent_state, plground) 
        u_new, v_new = ImagetoPlotCoords(crnt_state, plground) 
        cv2.line(plground, (u, v), (u_new, v_new), color, 1)
    else:
        u, v = u, v = ImagetoPlotCoords(crnt_state, plground) 
        plground[u, v, :] = color
    return plground

# A-Star implementation
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--StartState', default="15,15,0", type=str, help="Starting state (x, y, angle), enter values like 20,10,0!")
    parser.add_argument('-g','--GoalState', default="200,180,0", type=str, help="Goal state (x, y, angle), enter values like 20,10,0!")
    parser.add_argument('-z', '--StepSize', default=10, type=int, help="Size of each step taken by the robot. Give a value between 1 and 10.")
    parser.add_argument('-c', '--Clearance', default=5, type=int, help="Clearance bwteen the robot and the obstacles.")
    parser.add_argument('-r', '--RobotRadius', default=10, type=int, help="Radius of the robot.")
    parser.add_argument('-sp', '--SavePath', default="", help="Radius of the robot.")
    
    args = parser.parse_args()
    start_state = args.StartState
    start_state = [int(i) for i in start_state.split(",")]
    goal_state = args.GoalState
    goal_state = [int(i) for i in goal_state.split(",")]
    step_size = args.StepSize
    clearance = args.Clearance
    robot_radius = args.RobotRadius
    save_path = args.SavePath + "project3.avi"
    start_node = Node(start_state, None, None, 0.)
    
    my_queue = queue.PriorityQueue()
    my_queue.put((start_node.get_cost(), start_node))
    threshold = 0.5
    theta_incrmnt = 30
    threshold_dist = 1.5
    clearance += robot_radius

    if is_ObstacleSpace(start_state[0], start_state[1], clearance):
        print('The entered start state is in the obstacle space, please rerun the code with the correct arguments')
        sys.exit(1)
    if is_ObstacleSpace(goal_state[0], goal_state[1], clearance):
        print('The entered goal state is in the obstacle space, please rerun the code with the correct arguments')
        sys.exit(2)

    print(f'Using intial and goal nodes, {start_state} and {goal_state} respectively.\n')
    
    node_distances = np.ones((int(400/threshold), int(250/threshold), int(360/theta_incrmnt)))*np.inf
    playground = np.zeros([250, 400, 3], dtype=np.uint8)
    playground = get_playground(playground)
    
    # For time taken to find the node
    start_time = time.time()

    # For the loading symbol
    global animation_complete
    animation_complete = False
    loading_screen = threading.Thread(target = loading_bar)

    loading_screen.start()

    try:
        
        goal_flag = False
        states_passed = list()
        while my_queue.empty() == False:
            current_node = my_queue.get()[1]
            
            playground = plot_visited(playground, current_node)
            ma_ground = copy.deepcopy(playground)
            states_passed.append(ma_ground)
            if is_GoalReached(current_node, goal_state, threshold_dist):
                animation_complete = True
                time.sleep(0.2)
                print("Goal Found!!!")
                stop_time = time.time()
                total_time = stop_time - start_time
                print(f'Goal found in {round(total_time, 2)} seconds.')
                goal_flag = True
                complete_path = current_node.get_path()[1]
                break
            else:
                neighbors = get_neighbors(current_node, step_size, clearance)
                for neighbor in neighbors:
                    nbr_state = neighbor.get_state()
                    if is_visited(neighbor, goal_state, node_distances): 
                        i = int(customRounding(nbr_state[0])/threshold)
                        j = int(customRounding(nbr_state[1])/threshold)
                        k = int(nbr_state[2]/theta_incrmnt)
                        net_cost = neighbor.get_cost() + get_HeuristicCost(nbr_state, goal_state)
                        node_distances[i, j, k] = net_cost
                        my_queue.put((net_cost, neighbor))
        if goal_flag == False:
            print("Could not find a path!!!")

        finals = list()
        for my_node in complete_path:
            playground = plot_visited(playground, my_node, (210, 0, 90))
            grounds = copy.deepcopy(playground)
            for i in range(15):
                finals.append(grounds)
        result = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'DIVX'), 240, (400, 250))
        for st in states_passed:
            result.write(st)
            cv2.imshow('finding path', st)
            if cv2.waitKey(1) == ord('q'):
                print("Saving video.......")
                break
        for frm in finals:
            result.write(frm)
            cv2.imshow('finding path', frm)
            if cv2.waitKey(1) == ord('q'):
                print("Saving video.......")
                break
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except KeyboardInterrupt as e:
        
        loading_screen.alive = False
        loading_screen.join()
        print('\nExiting due to keyboard interrupt!')
        sys.exit(e)
        
    result.release()
    cv2.destroyAllWindows() 
        
if __name__ == "__main__":
    main()
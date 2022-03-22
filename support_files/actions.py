# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 19:28:54 2022

@author: Bhargav Kumar
"""

import numpy as np
from support_files.node import *
from support_files.mapping import *

    
def move_neg30(crnt_state, step_size, clearance):
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    theta_new = theta - 30
    if theta_new <= -360:
        theta_new += 360
    x_new = x + step_size*np.cos(np.radians(theta_new))
    y_new = y + step_size*np.sin(np.radians(theta_new))
    if is_ObstacleSpace(x_new, y_new, clearance):
        return None
    new_state = [x_new, y_new, theta_new]
    return new_state

def move_pos30(crnt_state, step_size, clearance):
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    theta_new = theta + 30
    if theta_new >= 360:
        theta_new -= 360
    x_new = x + step_size*np.cos(np.radians(theta_new))
    y_new = y + step_size*np.sin(np.radians(theta_new))
    
    if is_ObstacleSpace(x_new, y_new, clearance):
        return None
    new_state = [x_new, y_new, theta_new]
    return new_state

def move_neg60(crnt_state, step_size, clearance):
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    theta_new = theta - 60
    if theta_new <= -360:
        theta_new += 360
    x_new = x + step_size*np.cos(np.radians(theta_new))
    y_new = y + step_size*np.sin(np.radians(theta_new))
    if is_ObstacleSpace(x_new, y_new, clearance):
        return None
    new_state = [x_new, y_new, theta_new]
    return new_state

def move_pos60(crnt_state, step_size, clearance):
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    theta_new = theta + 60
    if theta_new >= 360:
        theta_new -= 360
    x_new = x + step_size*np.cos(np.radians(theta_new))
    y_new = y + step_size*np.sin(np.radians(theta_new))
    if is_ObstacleSpace(x_new, y_new, clearance):
        return None
    new_state = [x_new, y_new, theta_new]
    return new_state    

def move_straight(crnt_state, step_size, clearance):
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    theta_new = theta + 0
    x_new = x + step_size*np.cos(np.radians(theta_new))
    y_new = y + step_size*np.sin(np.radians(theta_new))
    if is_ObstacleSpace(x_new, y_new, clearance):
        return None
    new_state = [x_new, y_new, theta_new]
    return new_state

def get_neighbors(node, step_size, clearance):
    crnt_state = node.get_state()
    neighbors = list()
    neg60_neighbor = Node(move_neg60(crnt_state, step_size, clearance), node, "neg60", node.get_cost() + step_size)
    if neg60_neighbor.get_state() is not None:
        neighbors.append(neg60_neighbor)
    neg30_neighbor = Node(move_neg30(crnt_state, step_size, clearance), node, "neg30", node.get_cost() + step_size)
    if neg30_neighbor.get_state() is not None:
        neighbors.append(neg30_neighbor)
    straight_neighbor = Node(move_straight(crnt_state, step_size, clearance), node, "straight", node.get_cost() + step_size)
    if straight_neighbor.get_state() is not None:    
        neighbors.append(straight_neighbor)
    pos30_neighbor = Node(move_pos30(crnt_state, step_size, clearance), node, "pos30", node.get_cost() + step_size)
    if pos30_neighbor.get_state() is not None:
        neighbors.append(pos30_neighbor)
    pos60_neighbor = Node(move_pos60(crnt_state, step_size, clearance), node, "pos60", node.get_cost() + step_size)
    if pos60_neighbor.get_state() is not None:
        neighbors.append(pos60_neighbor)
    return neighbors
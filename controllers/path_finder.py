#!/usr/bin/env python3.8
import numpy as np
import json
import rospy
import queue
from dataclasses import dataclass
from enum import Enum
from std_msgs.msg import String
from collections import deque
import heapq
from PIL import Image  # Import the Image module


def findTargetCenter(target_num, maze): #finding coordinates of the targets
    #a = np.array(maze)
    indices = np.where(maze == target_num)
    coordinates = list(zip(indices[0], indices[1]))

    if not coordinates:
        print("No matching coordinates found.")
    else:
        middle_index = len(coordinates) // 2
        middle_x, middle_y = coordinates[middle_index]
    
    return int(middle_x), int(middle_y)


def findRobotPosition(target_num, maze): #finding robot coordinates
    #a = np.array(maze)
    indices = np.where(maze == target_num)
    coordinates = list(zip(indices[0], indices[1]))

    if not coordinates:
        print("No matching coordinates found.")
    else:
        middle_index = len(coordinates) // 2
        middle_x, middle_y = coordinates[middle_index]
    
    return int(middle_x), int(middle_y)

#dijkstra algorythm for finding closest route between target and the robot
def dijkstra(maze, start, end): 
    rows, cols = maze.shape
    distances = {(i, j): float('inf') for i in range(rows) for j in range(cols)}
    distances[start] = 0
    
    pq = [(0, start)]  # Priority queue of (distance, position) tuples
    parents = {}
    
    while pq:
        dist, current = heapq.heappop(pq)
        
        if current == end:
            path = []
            while current != start:
                path.append(current)
                current = parents[current]
            path.append(start)
            return path[::-1]
        
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and maze[neighbor] != 1:
                new_dist = dist + 1
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    parents[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
    
    return None  # No path found

def load_route(file_path): 
    with open(file_path, "r") as file:
        lines = file.readlines()
        route = [tuple(map(int, line.strip().split())) for line in lines]
    return route

#calculating movements based on position and orientation of robot 
def calculate_movement(current_position, next_position, previous_orientation): #returns movement and orientation(based on view from above on our simulation)
    row_diff = current_position[0] - next_position[0] 
    col_diff = current_position[1] - next_position[1]

    if previous_orientation == "down": 
        if row_diff == 1:
            return "p", "down" #using sing p for reverse because r is taken for the right turn
        elif row_diff == -1:
            return "f", "down"
        elif col_diff == 1:
            return "r", "left"
        elif col_diff == -1:
            return "l", "right"
    elif previous_orientation == "up":
        if row_diff == 1:
            return "f", "up"
        elif row_diff == -1:
            return "p", "up"
        elif col_diff == -1:
            return "r", "right"
        elif col_diff == 1:
            return "l", "left"
    elif previous_orientation == "right":
        if row_diff == 1:
            return "l", "up"
        elif row_diff == -1:
            return "r", "down"
        elif col_diff == -1:
            return "f", "right"
        elif col_diff == 1:
            return "p", "right"
    elif previous_orientation == "left":
        if row_diff == 1:
            return "r", "up"
        elif row_diff == -1:
            return "l", "down"
        elif col_diff == 1:
            return "f", "left"
        elif col_diff == -1:
            return "p", "left"
    else:
        return None, None  # Handle invalid orientation
        
    
def save_movements_to_file(movements, file_path):
    with open(file_path, "w") as file:
        for movement in movements:
            file.write(movement + "\n")


#check if the color around object match colors of finish zone
def check_color_around_object(matrix, position, target_color, radius):
    x, y = position
    for i in range(-radius, radius + 1):
        for j in range(-radius, radius + 1):
            adj_x, adj_y = x + i, y + j
            if 0 <= adj_x < len(matrix) and 0 <= adj_y < len(matrix[0]):
                color = matrix[adj_x][adj_y]
                if color == target_color:
                    return True
    return False 


maze = np.loadtxt("/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/matricaa.txt", dtype=int).tolist()
maze = np.array(maze)


def main(): #main
    
    rospy.init_node('maze_solver_node') #pokrenuo novi node
    pub = rospy.Publisher('turtlebot_motion', String, queue_size=10)
 
    robot_x, robot_y = findRobotPosition(4, maze) #robot
    finish_center1_x, finish_center1_y = findTargetCenter(2, maze)    #cilj 1
    finish_center2_x, finish_center2_y = findTargetCenter(3, maze)    # cilj 2
    object_center_x, object_center_y = findTargetCenter(5, maze)    #kocka
    
    finish1_color = 2  # Color value of finish1
    finish2_color = 3  # Color value of finish2
    
    #print coordinates of everything on the map
    print("object_center_x:", object_center_x)
    print("object_center_y:", object_center_y)
    print("robot x:", robot_x)
    print("robot y:", robot_y)
    print("finish 1x:", finish_center1_x)
    print("finish 1y:", finish_center1_y)
    print("finish 2x:", finish_center2_x)
    print("finish 2y:", finish_center2_y)
    
    #positions of the robot, object, finish1 and the finish2 at the beggining 
    robot_position = (robot_x, robot_y)
    target_position = (object_center_x, object_center_y)
    finish1_position = (finish_center1_x, finish_center1_y)
    finish2_position = (finish_center2_x, finish_center2_y)
    
    radius = 5 #check at this radius
        
        
    route = dijkstra(maze, robot_position, target_position)
    
    if route:
        #print("Route found:")
        
        # Write the route to a text file
        output_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/route.txt"
        with open(output_file_path, "w") as file:
            for position in route:
                file.write(f"{position[0]} {position[1]}\n")
                
        print(f"Route saved to '{output_file_path}'")
    
        
        route_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/route.txt"
        route = load_route(route_file_path)
        
        # Initialize previous_movement to "stop" at the start
        previous_orientation = "down"
        
        # Calculate movements based on the route
        movements = []
        for i in range(len(route) - 1):
            current_position = route[i]
            next_position = route[i + 1]
            movement, previous_orientation = calculate_movement(current_position, next_position, previous_orientation)
            movements.append(movement)
        movements.append('f') #adding more movements to push object and then to return back
        movements.append('p')
        
            
        # Save movements to a text file
        movements_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/movements.txt"
        save_movements_to_file(movements, movements_file_path)

        print("Movements saved to:", movements_file_path)
        
        # Publish each movement as a ROS message
        rate = rospy.Rate(0.25)  # 0.25hz
        rate.sleep()
        for movement in movements:
            pub.publish(movement)
            print("Published movement:", movement)
            rate.sleep()
    else:
        print("No route found.")

    # Wait for the camera to load the right image
    for _ in range(4):
        rate.sleep()
    
    #load maze with the latest camera picture
    #maze1 is regular matrix for color check and maze2 is numpy array for positions and route finding
    maze1 = np.loadtxt("/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/matricaa.txt", dtype=int).tolist()
    maze2 = np.array(maze1)
    print("Maze1 loaded:")
    
    rate.sleep() #waiting for camera to load the right image

    robot_new_x, robot_new_y = findRobotPosition(4, maze2) #robot
    robot_position_new = (robot_new_x, robot_new_y)
    print("robot x:", robot_new_x) #new robot coordinates
    print("robot y:", robot_new_y)
    
    
    
    #check color of the desired finish        
    if check_color_around_object(maze1, robot_position_new, finish1_color, radius):
        print("finish1 color!")
        
        #find route to the finish1
        route = dijkstra(maze2, robot_position_new, finish1_position)
        if route:
            print("Route found:")
            
            # Write the route to a text file
            output_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/new_route.txt"
            with open(output_file_path, "w") as file:
                for position in route:
                 file.write(f"{position[0]} {position[1]}\n")
                
            print(f"Route saved to '{output_file_path}'")
            
            movements = []
            for i in range(len(route) - 1):
                current_position = route[i]
                next_position = route[i + 1]
                movement, previous_orientation = calculate_movement(current_position, next_position, previous_orientation)
                movements.append(movement)
            
            # Save movements to a text file
            movements_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/new_movements.txt"
            save_movements_to_file(movements, movements_file_path)

            print("Movements saved to:", movements_file_path)
            
            # Publish each movement as a ROS message
            rate = rospy.Rate(0.25)  # 0.25hz
            rate.sleep()
            for movement in movements:
                pub.publish(movement)
                print("Published movement:", movement)
                rate.sleep()
        else:
            print("no route found:")
            
    #check color of the desired finish          
    elif check_color_around_object(maze1, robot_position_new, finish2_color, radius):
        print("finish2 color!")
        
        #find route to the finish2
        route = dijkstra(maze2, robot_position_new, finish2_position)
        if route:
            print("Route found:")
            
            # Write the route to a text file
            output_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/new_route.txt"
            with open(output_file_path, "w") as file:
                for position in route:
                 file.write(f"{position[0]} {position[1]}\n")
                
            print(f"Route saved to '{output_file_path}'")
            
            movements = []
            for i in range(len(route) - 1):
                current_position = route[i]
                next_position = route[i + 1]
                movement, previous_orientation = calculate_movement(current_position, next_position, previous_orientation)
                movements.append(movement)
            
            # Save movements to a text file
            movements_file_path = "/home/vanja/MEHATRONIKA/project_ws/src/camera_filter/src/camera_filter/nodes/new_movements.txt"
            save_movements_to_file(movements, movements_file_path)

            print("Movements saved to:", movements_file_path)
            
            # Publish each movement as a ROS message
            rate = rospy.Rate(0.25)  # 0.25hz
            rate.sleep()
            for movement in movements:
                pub.publish(movement)
                print("Published movement:", movement)
                rate.sleep()
            
        else:
            print("no route found:")
            
            
if __name__ == "__main__":
         main()


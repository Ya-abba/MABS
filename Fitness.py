import math
from numpy import dot
from numpy.linalg import norm
# -*- coding: utf-8 -*-
"""
Fitness Functions

@author: Jose Matamoros
"""
from AssignedBS import distance

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def line_cross(chromosome_set):
    '''
    The ratio of intersected paths the specimen has
    
    Parameters
    ----------
    chromosome_set : [chromosome]
        A set of chromosomes
        
    Returns
    -------
    (int)
        the number of intersections between all paths within the specimen
    '''
    
    paths = []
    for path in chromosome_set[:]:
        tmp_lines = path[:]
        tmp_lines.append(path[0])
        paths.append(tmp_lines)                
    
    segments=[]
    for path in paths:
        aa = [point for point in path][:-1]
        bb = [point for point in path][1:]
        tmp_segments = []
        for point_a,point_b in zip(aa,bb):
            tmp_segments.append([point_a,point_b])
        segments.append(tmp_segments)
    
    intersections = 0
    for i,lines in enumerate(segments):
        for j,lines2 in enumerate(segments):
            if(j>i):
                for line in lines:
                    for line2 in lines2:
                        if(intersect(line[0],line[1],line2[0],line2[1])):
                            intersections += 1
    return(intersections)

def total_distance(chromosome_set):
    '''
    The total distance of all the paths for a given child (set of chromosomes)
    
    Parameters
    ----------
    chromosome_set : [chromosome]
        A set of chromosomes
        
    Returns
    -------
    (float)
        The sum of all the distances of all paths (chromosomes)
    '''
    import numpy as np
    
    suma=0
    for path in chromosome_set.copy():
        aa = [point for point in path][:-1]
        bb = [point for point in path][1:]
        for point_a, point_b in zip(aa,bb):
            suma += distance(point_a,point_b)
        suma += distance(aa[0],bb[-1])
    return(suma)

def angle(a,b,c):
    '''
    Returns the angle ABC
    
    Parameters
    ----------
    a,b,c : [point,poin,point]
        Point (X,Y)
        
    Returns
    -------
    (float)
        the angle ABC
    '''
    from math import acos
    
    ab = distance(a,b)
    bc = distance(b,c)
    ca = distance(c,a)
    return( acos((ab**2 + bc**2 - ca**2)/(2*ab*bc)) ) 

def angle_ratio(chromosome_set):
    '''
    Returns a ratio of how much does the shape is similar to a perfect polygon
    with the ame number of angles
    
    Parameters
    ----------
    chromosome_set : [chromosome]
        The current iteration of the experiment
        
    Returns
    -------
    (float)
        The total calculated ratio of angles from all individual angles
        compared to an angle from a perfect polygon of n vertex
    '''
    from math import pi
    from functools import reduce
    
    ratio = []
    for path in chromosome_set.copy():
        points = [point for point in path]
        n = len(points)
        perfect_angle = (n-2)*pi/n
        points.insert(0,points[-1])
        points.append(points[0])
        suma = 0
        for i in range(1,n):
            tmp_angle = angle(points[i-1],points[i],points[i+1])
            if(tmp_angle>pi): tmp_angle = pi - tmp_angle
            suma += abs((tmp_angle - pi)/pi)
        ratio.append(1 - suma/n)
    return (reduce((lambda x,y: x+y), ratio)/len(chromosome_set))

def service_ratio(chromosome_set, unserviced_set, threshold):
    from AssignedBS import distance
    from CapacityCal import capacity
    from AssignedBS import assignedUAV
   
    unserviced_num = len(unserviced_set)
   
    diameter = 3
    points = []
    indices_to_remove = set()  # List to store indices of unserviced points to remove
   
    for path_i in chromosome_set:
        path = path_i.copy()
        n = len(path)
        path.append(path[0])
        for i in range(0, n):  # each line
            magnitude = distance(path[i], path[i + 1])
            points.append(path[i])
            div = round(magnitude / diameter - 0.5)
            if div > 3:
                for divisor in range(1, round(magnitude - 0.5), diameter):
                    t = divisor / magnitude
                    xt, yt = [(1 - t) * path[i][0] + t * path[i + 1][0], (1 - t) * path[i][1] + t * path[i + 1][1]]
                    points.append([xt, yt])
            points.append(path[-1])

            capacities = capacity(unserviced_set, points, assignedUAV(unserviced_set, points), threshold)
            for i, cap in enumerate(capacities):
                if cap[1] > 0:
                    indices_to_remove.add(i)  # Store index to remove

#convert the set of unique indices to a list
    indices_to_remove=list(indices_to_remove)

    # Remove unserviced points after iteration is complete
    for index in sorted(indices_to_remove, reverse=True):
        unserviced_set.pop(index)
        return len(unserviced_set) / unserviced_num

def balanced_service_ratio(unserviced_set):
    nearest_neighbour_distances = []
    for i in range(len(unserviced_set)):
        min_distance = float("inf")
        for j in range(len(unserviced_set)):
            if i != j:
                d = distance(unserviced_set[i], unserviced_set[j])
                if d < min_distance:
                    min_distance = d
        nearest_neighbour_distances.append(min_distance)
    return min(nearest_neighbour_distances) / max(nearest_neighbour_distances)  # Minimal ratio out of any 2 points

def path_smoothness_ratio(chromosome_set):
    ratios = []
    for path in chromosome_set:
        path = path.copy() + [path[0]]  # Append the initial point to the end for closed path
        directions = []  # Store direction vectors
        for i in range(len(path) - 1):
            directions.append([path[i + 1][j] - path[i][j] for j in range(2)])  # Vector from point_i to point_i+1
        smoothness_scores = []  # Store smoothness score of each turn (i.e., angle between consecutive direction vectors)
        for i in range(len(directions) - 1):
            cos_angle = dot(directions[i], directions[i + 1]) / (norm(directions[i]) * norm(directions[i + 1]))
            angle = math.acos(
                max(min(cos_angle, 1), -1))  # Clip input to avoid out of range error due to floating point error
            smoothness_scores.append(angle)
        ratios.append(
            1 - sum(smoothness_scores) / (math.pi * (len(smoothness_scores))))  # Math.pi is the maximum angle possible
    return sum(ratios) / len(ratios)  # Assumes all paths are equally important

# Place the rest of your functions above this line

if __name__ == "__main__":
    # Test data
    chromosome_set = [
        [[0, 0], [1, 1], [2, 2]],  # Chromosome 1
        [[0, 2], [1, 1], [2, 0]],  # Chromosome 2 - intersects with Chromosome 1
        [[0, 2], [1, 1], [2, 0]],  # Chromosome 2 - intersects with Chromosome 1
    ]
    unserviced_set = [
        [0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5]
    ]
    threshold = 1.0

    # Test total_distance
    print(f'Test - Total distance: {total_distance(chromosome_set)}')

    # Test line_cross
    print(f'Test - Number of line intersections: {line_cross(chromosome_set)}')

    # Test angle_ratio
    print(f'Test - Angle ratio: {angle_ratio(chromosome_set)}')

    # Test service_ratio
    print(f'Test - Service ratio: {service_ratio(chromosome_set, unserviced_set, threshold)}')

    # Test balance_service_ratio
    print(f'Test - Balanced Service ratio: {balanced_service_ratio(unserviced_set)}')

    # Test path_smoothness_ratio
    print(f'Test - Path Smoothness ratio: {path_smoothness_ratio(chromosome_set)}')

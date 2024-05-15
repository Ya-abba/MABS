# -*- coding: utf-8 -*-
"""
Performs UAV positioning and calculations through GA

@author: Jose Matamoros
"""
import numpy as np
import Mutators
import Fitness

class Child:
    '''
    Each class must be created for each child. 
    Each child will have multiple chromosomes within, which will allow it to mutate.
    '''
    def __init__(self, current, centroids, drone_limit):
        '''
        Parameters
        ----------
        current : (int)
            The current iteration of the experiment
            
        centroids : []
            If current == 0 then it must be a set of cluster centroids
            else
            It must be a set of chromosomes
        '''
        self.born = current
        self.current = current
        if current == 0:
            self.chromosomes = Mutators.initial_build(centroids, drone_limit)
        else:
            self.chromosomes = centroids
    
    def mutate(self, current):
        '''
        Mutates and produces new children from the Child's chromosomes.
        
        Parameters
        ----------
        current : (int)
            The current iteration of the experiment
            
        '''
        self.current = current
        new_childs = []
        
        for new_ch in Mutators.mutator_in_chromo(self.chromosomes):
            new_childs.append(new_ch)
        
        if len(self.chromosomes) > 2:
            for new_ch in Mutators.mutator_cross_chromo(self.chromosomes):
                new_childs.append(new_ch)
        
        return new_childs
    
    def fitness(self, unserviced_set, threshold):
        '''
        Parameters
        ----------
        unserviced_set : [UE]
            A set of UE that are in need of service
        
        threshold : (float)
            The threshold to measure service
        
        Returns
        -------
        [float, float, float, float, float]
            Returns the total distance of the specimen's paths, the angle ratio of all paths,
            the number of intersections, the service ratio, and the path smoothness ratio.
        '''
        distance = Fitness.total_distance(self.chromosomes)
        angle_ratio = Fitness.angle_ratio(self.chromosomes)
        intersections = Fitness.line_cross(self.chromosomes)
        service_ratio = Fitness.service_ratio(self.chromosomes, unserviced_set.copy(), threshold)
        balanced_service_ratio = Fitness.balanced_service_ratio(unserviced_set)
        path_smoothness_ratio = Fitness.path_smoothness_ratio(self.chromosomes)
        
        return [distance, angle_ratio, intersections,balanced_service_ratio, path_smoothness_ratio]

if __name__ == "__main__":
    # Sample test data
    unserviced_set = [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]]
    threshold = 1.0
    current = 1
    centroids = [[0, 0], [1, 1], [2, 2]]
    drone_limit = 3
    
    # Create a sample child instance
    child = Child(current, centroids, drone_limit)
    
    # Test the fitness method
    fitness_metrics = child.fitness(unserviced_set, threshold)
    print("Fitness Metrics:")
    print("Total Distance:", fitness_metrics[0])
    print("Angle Ratio:", fitness_metrics[1])
    print("Intersections:", fitness_metrics[2])
    print("Service Ratio:", fitness_metrics[3])
    print("Balanced Service Ratio:", fitness_metrics[4])
    print("Path Smoothness Ratio:", fitness_metrics[5])
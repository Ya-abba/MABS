from ClusterCentroid import centroids
import matplotlib.pyplot as plt

plt.figure(figsize=(12, 10))
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.ndimage import gaussian_filter1d
import numpy as np
from Fitness import service_ratio
from AssignedBS import assignedBS
from CapacityCal import capacity
import UAV


class Experiment:
    '''
    Each object will be a different experiment, comprised of:
    -A set of UE point sets
    -A set of working BS
    -An genesis parent from which children will be created
    -Multiple Childs
    
    The object will carry the results of the expermients within it's local variables.
    
    '''

    def __init__(self, UEset, BSset, capacity_threshold):
        '''
        Requires a set of MB and UE
        '''
        self.capacity_threshold = capacity_threshold
        self.BSset = BSset
        self.UEset = UEset
        self.capacities = []
        self.unserviced = []
        self.results = []
        self.coverage_results = []
        # create parent based on given clusters from a set
        # create first generation of childs from parent
        self.initial_assigned_BS = assignedBS([[x, y] for x, y in zip(UEset.xue, UEset.yue)],
                                              [[x, y] for x, y in zip(BSset.xbs, BSset.ybs)])
        ue_set = list(zip([x for x in UEset.xue], [y for y in UEset.yue]))
        bs_set = list(zip([x for x in BSset.xbs], [y for y in BSset.ybs]))
        self.initial_capacity = capacity(ue_set, bs_set, self.initial_assigned_BS, capacity_threshold)

    def process_loss(self):
        '''
            A process to determine BS loss and all variable sets needed for further experiments.
        '''
        ue_set = list(zip([x for x in self.UEset.xue], [y for y in self.UEset.yue]))
        loss_set = [i * 0.1 for i in range(1, 10)]
        tot_l = len(self.BSset.xbs)
        for loss in loss_set:
            BStempX = self.BSset.xbs[:round(tot_l * loss)]
            BStempY = self.BSset.ybs[:round(tot_l * loss)]
            bs_set_temp = list(zip([x for x in BStempX], [y for y in BStempY]))
            temp_assignedBS = assignedBS([[x, y] for x, y in zip(self.UEset.xue, self.UEset.yue)],
                                         [[x, y] for x, y in zip(BStempX, BStempY)])
            self.capacities.append(capacity(ue_set, bs_set_temp, temp_assignedBS, self.capacity_threshold))
            temp_un = []
            for user, cap in zip(ue_set, self.capacities[-1]):
                if (cap[1] == 0):
                    temp_un.append(user)
            self.unserviced.append(temp_un)

    def process_uav(self, specimen_population):
        '''
        Executes a set of mutation funcions and thorugh fitness functions retreives the best candidates.
        
        Parameters
        ----------
		specimen_population : int
			The number of best secimens that are selected to keep mutating between generations.
        
        Returns
        -------
        
        '''
        self.unserviced_centroids = []
        for loss in range(0, 9):
            unserviced_tmp = self.unserviced[loss][:]
            tmp_unserviced_centroids = centroids([x[0] for x in unserviced_tmp], [x[1] for x in unserviced_tmp])
            self.unserviced_centroids.append(tmp_unserviced_centroids)
            self.uav_set = [UAVset(0, 0, i, tmp_unserviced_centroids) for i in range(1, 6)]
            #            self.uav_set = [UAVset(0,0,i,tmp_unserviced_centroids) for i in range(1,2)]
            for n, uav_option in enumerate(self.uav_set):
                best_score = 0
                candidates = []
                fit_counter = 0
                limit = 40
                while (fit_counter < limit):
                    fit_counter += 1
                    uav_option.mutate()
                    uav_option.fit(unserviced_tmp, 200000, 5, len(tmp_unserviced_centroids), specimen_population)
                    best = uav_option.best_uav
                    if (best[0] > best_score):
                        best_score = best[0]
                        candidates.append({'specimen': best[1], 'score': best_score})
                        fit_counter = 0
                print('Calculated drone %d out of %d for loss %d out of %d' % (n + 1, len(self.uav_set), loss + 1, 9))
                result = {'loss': (9 - loss) * 10, 'allowed_drones': (n + 1), 'candidates': candidates,
                          'best_score': best_score, 'best_specimen': candidates[-1]}
                # result = {'loss': (9 - loss) * 10, 'allowed_drones': (n+1), 'candidates': candidates}
                self.results.append(result)

    def plot_coverage(self, variable_type):
        '''
        Plot coverage ratio versus the number of base stations or UAVs.
        
        Parameters
        ----------
        variable_type : str
            Specifies whether the variable is "Base Stations" or "UAVs".
        '''
        for loss_data in self.coverage_results:
            loss = loss_data['loss']
            variable_values = loss_data['allowed_drones']
            coverage = loss_data['coverage']

            plt.plot(variable_values, coverage, label=f'Loss {loss}%')

        plt.xlabel(f'Number of {variable_type}')
        plt.ylabel('Coverage Ratio')
        plt.title(f'Coverage Ratio vs Number of {variable_type}')
        plt.legend()
        plt.grid(True)
        plt.show()

    def set_loss(self, loss_percent):
        '''
        Update the loss information for the experiment.
        
        Parameters
        ----------
        loss_percent : int
            The percentage of base station loss.
        
        Returns
        -------
        None
        '''
        self.loss_percent = loss_percent
        # self.process_uav(100)  # Process UAVs for the new loss information

    def graph_path(specimen_candidates, unserviced_points, unserviced_centroids, loss, drones):
        fig_p = 1

        # Calculate Voronoi diagram
        vor = Voronoi(unserviced_centroids)

        # Loop over each drone in the specimen_candidates list
        for i, drone in enumerate(specimen_candidates):
            # Create a new figure for each drone's path
            fig1 = plt.figure(fig_p, figsize=(10, 8))

            # Create a subplot for the Voronoi plot
            ax_voronoi = fig1.add_subplot(111)

            # Plot Voronoi diagram
            voronoi_plot_2d(vor, ax=ax_voronoi, show_vertices=False, line_colors='black', line_width=0.75,
                            line_alpha=0.75, point_size=1.5)

            # Plot unserviced centroids as green stars and unserviced points as red dots
            ax_voronoi.plot([x[0] for x in unserviced_centroids], [x[1] for x in unserviced_centroids], 'g*',
                            markersize=10)
            ax_voronoi.plot([x[0] for x in unserviced_points], [x[1] for x in unserviced_points], 'r.', markersize=0.5)

            # Plot the path taken by the drone using different line styles
            path = drone['specimen'].chromosomes
            co = ['y--', 'c--', 'm--', 'b--', 'k--', 'y-', 'c-', 'm-', 'b-', 'k-']
            for pa in path:
                x = [x[0] for x in pa]
                x.insert(0, x[-1])
                y = [x[1] for x in pa]
                y.insert(0, y[-1])
                ax_voronoi.plot(x, y, co.pop(), linewidth=1)

            # Add labels, title, and show the figure
            plt.xlabel('Km')
            plt.ylabel('Km')
            plt.title('Specimen %d with a score of %.2f %s, with a %d %s of BS loss, and %d drone(s) allowed' % (
                i, drone['score'] * 100, '%', loss, '%', drones))
            plt.savefig(f'figure{fig_p}.png')

            plt.gcf().set_size_inches(17, 10)
            fig_p += 1

    # @@@@@@@@@@@@@@@@@@@@@@@@
    def plot_coverage(self):

        for loss_data in self.coverage_results:
            loss = self.results[-1]['loss']
            allowed_drones = [entry['allowed_drones'] for entry in loss_data]
            coverage = [entry['coverage'] for entry in loss_data]

            plt.plot(allowed_drones, coverage, label=f'Loss {loss}%')

        plt.xlabel('Number of Drones Allowed')
        plt.ylabel('Coverage Percentage')
        plt.title('Coverage Percentage vs Number of Drones Allowed')
        plt.legend()
        plt.grid(True)
        plt.show()

    def set_loss(self, loss_percent):
        '''
        Update the loss information for the experiment.
        
        Parameters
        ----------
        loss_percent : int
            The percentage of base station loss.
        
        Returns
        -------
        None
        '''
        self.loss_percent = loss_percent
        # self.process_uav(100)  # Process UAVs for the new loss information

    def showResults(self):
        '''
		Displays the initial results for the given experiment.
		
		Parameters
        ----------
        
        Returns
        -------
		
		'''

        loss_range = range(10, 100, 10)
        distributions = [exp_pn]
        labels = ['GA+AP']
        markers = ['-vc']
        for dist, label, marker in zip(distributions, labels, markers):
            scores = [max([y['best_score'] for y in list(filter(lambda x: x['loss'] == k, [x for x in dist.results]))])
                      for k in loss_range]
            # scores = [max([y['candidates'][-1]['score'] for y in list(filter(lambda x: x['loss'] == k, [x for x in dist.results]))]) for k in loss_range]

            smoothed_scores = gaussian_filter1d(scores, sigma=1)
            plt.plot(loss_range, smoothed_scores, marker, markersize=12, label=label)
        plt.legend()
        plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
        plt.xlabel('BS loss [%]', fontsize='18')
        plt.xlim(10, 100)
        plt.ylabel('MAXIMUM SCORE OVER 1', fontsize='18')
        plt.title('MAXIMUM SCORE PER BS LOSS PERCENTAGE EXPERIMENTS WITH POISSON DISTRIBUION FOR UE AND BS',
                  fontsize='18')
        plt.gcf().set_size_inches(15, 10)

        plt.plot([i for i in range(10, 100, 10)],
                 gaussian_filter1d([max([y['best_score'] for y in list(filter(lambda x: x['loss'] == k,
                                                                              [x for x in exp_pn.results]))]) for k in
                                    range(10, 100, 10)], sigma=1), '-Xg', markersize=12)

        plt.legend()
        plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
        plt.xlabel('BS loss [%]', fontsize='15')
        plt.ylabel('MAXIMUM SCORE OVER 1', fontsize='18')
        plt.title('MAXIMUM SCORE PER BS LOSS PERCENTAGE EXPERIMENTS WITH POISSON DISTRIBUION FOR UE AND BS',
                  fontsize='15')
        plt.gcf().set_size_inches(17, 8.5)
        plt.show()

        # show initial serviced UE
        fig1 = plt.figure(1)
        plt.hist([x[1] for x in self.initial_capacity])
        plt.xlabel('Service')
        plt.ylabel('Ammount')
        plt.title('Number of UE that have service before Loss')
        # fig1.show()
        plt.gcf().set_size_inches(10, 8)
        plt.show()
        fig1.show()
        # show % of serviced UE in relation to BS loss
        prt = []
        for cap in self.capacities:
            [a, b, c] = plt.hist([x[1] for x in cap], 2)
            prt.append(a[0] * 100 / (a[0] + a[1]))

        fig2 = plt.figure(2)
        plt.plot(([100 - x for x in prt[::-1]]), '--r')
        plt.xlabel('BS loss [%]')
        plt.ylabel('UE serviced [%]')
        plt.title('SERVICE TO UE ACCORDING TO NUMBER OF BS')
        plt.grid(True)
        plt.gcf().set_size_inches(10, 8)
        plt.show()
        fig2.show()

        plt.plot([i for i in range(10, 100, 10)], gaussian_filter1d(
            [max([y['best_specimen']['specimen'].born for y in list(filter(lambda x: x['loss'] == k,
                                                                           [x for x in exp_pn.results]))]) for k in
             range(10, 100, 10)], sigma=1), '-Pk', markersize=12, label='PN')

        plt.legend()
        plt.setp(plt.gca().get_legend().get_texts(), fontsize='10')
        plt.xlabel('BS loss [%]', fontsize='12')
        plt.ylabel('Generations', fontsize='12')
        plt.title('MAXIMUM NUMBER OF GENERATIONS NEEDED TO FIND BEST SPECIMEN WITH POISSON DISTRIBUION FOR UE AND BS',
                  fontsize='12')
        plt.gcf().set_size_inches(17, 8.5)
        plt.show()
        # -----------------------------------------------------------------------------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------------------

        plt.hist([[len(x['best_specimen']['specimen'].chromosomes) for x in exp_pn.results]], label=['GA+AP'])
        # plt.xticks(np.arange(5), ('1', '2', '2', '3', '4'))
        plt.autoscale(tight=True)
        plt.grid('False')
        plt.legend(loc='upper right')
        plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
        plt.xlabel('NUMBER OF PATHS SELECTED FROM THE BEST SPECIMEN', fontsize='18')
        plt.ylabel('NUMBER OF EXPERIMENTS', fontsize='18')
        plt.title('NUMBER OF DETERMINED PATHS FOR ALL EXPERIMENTS FOR ALL DISTRIBUIONS FOR UE AND BS', fontsize='18')
        plt.gcf().set_size_inches(17, 8.5)
        plt.show()
        fig2.show()
        # -------------------------------------------------
        col = ['b']
        ha = ('/')
        plt.hist([x[1] for x in exp_pn.initial_capacity], label=['GA+AP'])

        plt.autoscale(tight=True)
        plt.legend(loc='upper left')
        plt.grid(color='k', linestyle='-', linewidth=0.5)
        # plt.xlabel('Service')
        plt.setp(plt.gca().get_legend().get_texts())
        plt.tick_params(axis='x', which='both', bottom='False', top='False')
        plt.xticks([0, 1], ['No Service', 'With Service'])
        plt.ylabel('Ammount of UE')
        plt.title('Number of UE that have service in Initial Conditions', fontsize='10')
        plt.show()
        # -------------------------------------------------

        fig1 = plt.figure(1)

        col = ['-b']
        lab = ['GA+AP']

        prt = []
        for cap in exp_pn.capacities:
            [a, b, c] = plt.hist([x[1] for x in cap], 2)
            prt.append(a[0] * 100 / (a[0] + a[1]))

        fig2 = plt.figure(2)
        plt.plot([x for x in range(10, 100, 10)], [100 - x for x in prt[::-1]], col[0], label=lab[0])
        fig1 = plt.figure(1)

        fig2 = plt.figure(2)
        plt.legend()
        plt.setp(plt.gca().get_legend().get_texts())
        plt.xlabel('BS loss [%]')
        plt.ylabel('UE serviced [%]')
        plt.title('UE WITH SERVICE AFTER BS LOSS')
        plt.grid(True)
        fig2.show()
        plt.show()


class UserEquipmentSet:
    '''
    Each object will have a set of points where the UE are, and a set of determinated clusters according to those points.
    '''

    def __init__(self, density, distribution, area):
        from math import sqrt
        quantity = density * area
        if (distribution == 'p'):
            numue = np.random.poisson(quantity, 1)  # lambda = quantity
        elif (distribution == 'n'):
            numue = np.int32(np.abs(np.round(np.random.normal(quantity, 10, 1))))  # mu = quantity, sigma = 10
        elif (distribution == 'u'):
            numue = np.int32(np.abs(np.round(np.random.uniform(1, quantity, 1))))

        self.xue = np.random.uniform(0, area, numue)
        self.yue = np.random.uniform(0, area, numue)
        self.clusters = centroids(self.xue, self.yue)


class MacroBaseSet:
    '''
    Each object will have a set of points that establishes where will working base stations be stablished within an experiment.
    '''

    def __init__(self, density, distribution, area):
        from math import sqrt
        quantity = density * area
        if (distribution == 'p'):
            numbs = np.random.poisson(quantity, 1)  # lambda = quantity
        elif (distribution == 'n'):
            numbs = np.int32(np.abs(np.round(np.random.normal(quantity, 10, 1))))  # mu = quantity, sigma = 1
        elif (distribution == 'u'):
            numbs = np.int32(np.abs(np.round(np.random.uniform(1, quantity, 1))))

        self.xbs = np.random.uniform(0, area, numbs)
        self.ybs = np.random.uniform(0, area, numbs)


class UAVset:
    '''
    Each class must be created for each child. 
    Each child will have mustiple chromosomes within, which will allow it to mutate.
    '''

    def __init__(self, born, current, drone_limit, centroids):
        self.current = 0
        self.drone_number = drone_limit
        self.uavs = [UAV.Child(current, centroids, drone_limit) for x in range(0, 50)]

    def mutate(self):
        '''
        Generates new generations of childs based based on mutators.
        At the end of the process, the UAV object will have more Childs (combinations)
        
        Parameters
        ----------
        
        Returns
        -------
        
        '''
        self.current += 1
        tmp = []
        for uav in self.uavs:
            for n_uav in uav.mutate(self.current): tmp.append(n_uav)
            # for n_uav in UAV.Child(self.current,uav.mutate(self.current)): self.uavs.append(n_uav)
        for uav in tmp: self.uavs.append(UAV.Child(self.current, uav, self.drone_number))

    #    def getKey(item):
    #        return item[0]
    def fit(self, unservice_set, threshold, drone_limit, k, sample):
        from operator import itemgetter
        from functools import reduce
        import numpy as np

        # Lists to store fitness components for each UAV
        distances, angle_ratios, intersections_set,balanced_service_ratios, smoothness_ratios = [], [], [], [], []
        distance, angle_ratio, intersections, balanced_service_ratio, smoothness_ratio = 0,0,0,0,0
        
        for i, uav in enumerate(self.uavs):
            
            # Calculate fitness components for each UAV
            #distance = Fitness.total_distance(self.chromosomes)
            #angle_ratio = angle_ratio(uav.chromosomes)
            #intersections = line_cross(uav.chromosomes)
            #service_ratio = service_ratio(uav.chromosomes, unservice_set, threshold)
            #balanced_service_ratio = balanced_service_ratio(unservice_set)
            #smoothness_ratio = path_smoothness_ratio(uav.chromosomes)
            [distance, angle_ratio, intersections,balanced_service_ratio, smoothness_ratio] = uav.fitness(unservice_set, threshold)
            
            # Append fitness components to respective lists
            distances.append(distance)
            angle_ratios.append(angle_ratio)
            intersections_set.append(intersections)
            #service_ratios.append(service_ratio)
            balanced_service_ratios.append(balanced_service_ratio)
            smoothness_ratios.append(smoothness_ratio)

        # Determine minimum values for normalization
        min_intersections = min(intersections_set)
        min_distance = min(distances)

        self.uav_list = []

        for distance_total, angle_ratio, path_intersections, balanced_service_ratio, smoothness_ratio, uav \
                in zip(distances, angle_ratios, intersections_set, balanced_service_ratios,
                       smoothness_ratios, self.uavs):
            # Calculate normalized ratios
            if path_intersections != 0:
                intersection_ratio = min_intersections / path_intersections
            else:
                intersection_ratio = 1

            distance_ratio = min_distance / distance_total

            # Calculate overall score based on weighted combination of fitness components
            score = (distance_ratio * 0.2 + angle_ratio * 0.3 + intersection_ratio * 0.3 +
                    balanced_service_ratio * 0.1 + smoothness_ratio * 0.1)

            # Filter UAVs based on constraints
            l1 = [point for path in uav.chromosomes for point in path]
            tot_k = reduce(lambda x, y: x + y, [len(path) for path in uav.chromosomes])

            if len(uav.chromosomes) > drone_limit or tot_k != k or len(l1) != len(np.unique(l1, axis=0)):
                self.uav_list.append([0, uav])  # Mark for elimination
            else:
                self.uav_list.append([score, uav])  # Add to potential candidates

        # Sort potential candidates by score
        self.uav_list = sorted(self.uav_list, key=itemgetter(0), reverse=True)

        # Keep only the top 'sample' candidates
        for uav in self.uav_list[sample:]:
            del uav[1]

        # Remove marked candidates and update self.uavs
        self.uav_list = list(filter(lambda x: len(x) > 1, self.uav_list))
        self.uavs = [uav[1] for uav in self.uav_list]
        self.best_uav = self.uav_list[0]

def graph_path(specimen_candidates, unserviced_points, unserviced_centroids, loss, drones):
    fig_p = 1

    # Calculate Voronoi diagram
    vor = Voronoi(unserviced_centroids)

    # Loop over each drone in the specimen_candidates list
    for i, drone in enumerate(specimen_candidates):
        # Create a new figure for each drone's path
        fig1 = plt.figure(fig_p, figsize=(10, 8))

        # Create a subplot for the Voronoi plot
        ax_voronoi = fig1.add_subplot(111)

        # Plot Voronoi diagram
        voronoi_plot_2d(vor, ax=ax_voronoi, show_vertices=False, line_colors='black', line_width=0.75, line_alpha=0.75,
                        point_size=1.5)

        # Plot unserviced centroids as green stars and unserviced points as red dots
        ax_voronoi.plot([x[0] for x in unserviced_centroids], [x[1] for x in unserviced_centroids], 'g*', markersize=10)
        ax_voronoi.plot([x[0] for x in unserviced_points], [x[1] for x in unserviced_points], 'r.', markersize=0.5)

        # Plot the path taken by the drone using different line styles
        path = drone['specimen'].chromosomes
        co = ['y--', 'c--', 'm--', 'b--', 'k--', 'y-', 'c-', 'm-', 'b-', 'k-']
        for pa in path:
            x = [x[0] for x in pa]
            x.insert(0, x[-1])
            y = [x[1] for x in pa]
            y.insert(0, y[-1])
            ax_voronoi.plot(x, y, co.pop(), linewidth=1)

        # Add labels, title, and show the figure
        plt.xlabel('Km')
        plt.ylabel('Km')
        plt.title('Specimen %d with a score of %.2f %s, with a %d %s of BS loss, and %d drone(s) allowed' % (
            i, drone['score'] * 100, '%', loss, '%', drones))
        plt.savefig(f'figure{fig_p}.png')
        plt.gcf().set_size_inches(17, 10)
        fig_p += 1


if __name__ == "__main__":
    # Create set experiment objects
    uep = UserEquipmentSet(10, 'p', 5)
    bsn = MacroBaseSet(5, 'n', 5)
    exp_pn = Experiment(uep, bsn, 200000)
    exp_pn.process_loss()
    exp_pn.process_uav(100)
    exp_pn.showResults()
    #   exp_pn.graph_path()
    print('PN')

    element = 0
    loss1 = 0
    graph_path(exp_pn.results[element]['candidates'], exp_pn.unserviced[loss1], exp_pn.unserviced_centroids[loss1],
               exp_pn.results[element]['loss'], exp_pn.results[element]['allowed_drones'])

# *8888888888888888888888888888888888888888888888888888888888888888888888888888888888888***


# 1. User with and without service after 80% BS loss
# ------------------------------------------------------------------------------------------------------------------------
vor = Voronoi(list(zip([x for x in bsn.xbs], [y for y in bsn.ybs])))
fig = voronoi_plot_2d(vor, show_vertices=False, line_colors='black', line_width=0.75, line_alpha=0.75, point_size=1.5)
tot_l = len(exp_pn.BSset.xbs)

plt.plot(uep.xue, uep.yue, 'g*', label='Connected UE', markersize=8)

# Plot UEs without service
unserviced_x = [x[0] for x in exp_pn.unserviced[0]]
unserviced_y = [x[1] for x in exp_pn.unserviced[0]]
plt.plot(unserviced_x, unserviced_y, 'r*', label='Uncovered UE', markersize=8)

# Plot all BS
plt.plot(bsn.xbs, bsn.ybs, 'b2', label='Active BS', markersize=12)

# Plot BS without service
bs_without_service_x = [bsn.xbs[i] for i in range(len(bsn.xbs)) if i >= round(tot_l * 0.1)]
bs_without_service_y = [bsn.ybs[i] for i in range(len(bsn.ybs)) if i >= round(tot_l * 0.1)]
plt.plot(bs_without_service_x, bs_without_service_y, 'k2', label='DeadZone BS', markersize=12)

plt.xlabel('Km')
plt.ylabel('Km')
plt.xlim(0, 25)
plt.ylim(0, 25)
plt.legend(loc='upper right')
plt.setp(plt.gca().get_legend().get_texts())
plt.title('User with and without service after 80% BS loss')
plt.gcf().set_size_inches(20, 12)
plt.show()
# *************************************************************************************************************************

# 2. Point placement and Voronoi diagram for UEs and Base stations
vor = Voronoi(list(zip([x for x in bsn.xbs], [y for y in bsn.ybs])))
fig = voronoi_plot_2d(vor, show_vertices=False, line_colors='black', line_width=0.75, line_alpha=0.75, point_size=1.5)
plt.plot(uep.xue, uep.yue, 'g*', label='Connected UE', markersize=8)
plt.plot(bsn.xbs, bsn.ybs, 'b2', label='Active BS', markersize=10)
plt.xlabel('Km')
plt.ylabel('Km')
plt.xlim(0, 25)
plt.ylim(0, 25)
plt.legend()
plt.legend(loc='upper right')
plt.setp(plt.gca().get_legend().get_texts())
plt.title('Point placement and Voronoi diagram for UEs and Base stations')
plt.gcf().set_size_inches(20, 12)
plt.show()
# ----------------------------------------------------------------------------------------------------------------------------

plt.show()

# ---------------------------------------------------------------------------------------------------------------------------

exp_pn.showResults()

# Plot unserviced UEs for different loss percentages
loss_percentages = [i * 10 for i in range(1, 10)]
unserviced_counts = [len(unserviced_set) for unserviced_set in exp_pn.unserviced]

plt.figure(figsize=(8, 6))
plt.plot(loss_percentages, unserviced_counts, marker='o')
plt.xlabel('Base Station Loss Percentage')
plt.ylabel('Serviced UEs')
plt.title('Base Station Loss vs. Serviced UEs')
plt.grid(True)
plt.show()

col = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'b', 'g']
ha = ('/', 'o', '+', '', 'v', 'D', 'p', 'P', '//')

plt.hist([[x[1] for x in exp_pn.initial_capacity], ], label=['Poison'])

plt.autoscale(tight=True)
plt.legend(loc='upper left')
plt.grid(color='k', linestyle='-', linewidth=0.5)
plt.xlabel('Service', fontsize='18')
plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
plt.xticks([0, 1], ['No Service', 'With Service'], fontsize='18')
plt.ylabel('Ammount of UE', fontsize='18')
plt.title('Number of UE that have service in Initial Conditions', fontsize='18')

# ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))

tot = []
col = ['-*b']
lab = ['GA+AP']

# # For exp_pn
cov_dis_pn = []
for loss in range(10, 100, 10):
    cov = []
    for itt in [x for x in exp_pn.results]:
        if itt['loss'] == loss:
            cov.append(
                service_ratio(itt['best_specimen']['specimen'].chromosomes, exp_pn.unserviced[int(loss / 10) - 1],
                              200000))
    print('The coverage ratio for exp_pn is %g with a loss of %d' % (min(cov), loss))
    cov_dis_pn.append(min(cov))

plt.plot([i for i in range(10, 100, 10)], cov_dis_pn, col[0], label=lab[0])
tot.append(cov_dis_pn)

plt.legend()
plt.setp(plt.gca().get_legend().get_texts())
plt.xlabel('BS loss [%]')
plt.ylabel('UE serviced [%]')
plt.title('UE WITH SERVICE AFTER BS LOSS WITH MABS IMPLEMENTATION')
plt.grid(True)
plt.gcf().set_size_inches(17, 8.5)
plt.show()

# ****************************
tot = []
for dis in exp_pn.results:
    cov_dis = []
    for loss in range(10, 100, 10):
        cov = []
        if 'results' in dis:
            for itt in [x for x in dis['results']]:
                if 'best_specimen' in itt and 'specimen' in itt['best_specimen'] and 'chromosomes' in \
                        itt['best_specimen']['specimen'] and int((loss) / 10) - 1 in dis.get('unserviced', {}):
                    if itt['loss'] == loss:
                        chromosome = itt['best_specimen']['specimen'].chromosomes
                        unserviced = dis['unserviced'][int((loss) / 10) - 1]
                        cov.append(service_ratio(chromosome, unserviced, 200000))
            if cov:
                print('The coverage ratio is %g with a loss of %d' % (min(cov), loss))
                cov_dis.append(min(cov))
            else:
                print('No coverage computed, check if itt met the conditions.')
        else:
            print('No results key in dis')
    tot.append(cov_dis)

# Assuming data in tot is for plot not hist
plt.plot([i for i in range(10, 100, 10)], tot, label=['UBPN'])
legend = plt.legend()
if legend:
    plt.setp(legend.get_texts(), fontsize='20')
else:
    print("No legend found")
plt.xlabel('BS loss [%]', fontsize='18')
plt.ylabel('UE serviced [%]', fontsize='18')
plt.title('UE WITH SERVICE AFTER BS LOSS WITH UABS IMPLEMENTATION', fontsize='18')
plt.grid(True)
plt.show()  # Don't forget to show the plot

# ****************************
print(service_ratio(itt['best_specimen']['specimen'].chromosomes, exp_pn.unserviced[1], 200000))
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
fig1 = plt.figure(1)
fig2 = plt.figure(2)

col = ['-vc']
lab = ['GA+AP']

# Iterate over the results in exp_pn
for j, k in enumerate(exp_pn.results):
    prt = []
    # Check if 'capacities' is present in the dictionary
    if 'capacities' in k:
        for cap in k['capacities']:
            [a, b, c] = plt.hist([x[1] for x in cap], 2)
            prt.append((a[1]) * 100 / (a[0] + a[1]))

        # Add subplot for the second figure
        ax2 = fig2.add_subplot(111)
        ax2.plot([x for x in range(10, 100, 10)], [100 - x for x in prt[::-1]], col[j], markersize=9, label=lab[j])

# Add legend and labels to the second figure
plt.legend()
plt.xlabel('BS loss [%]', fontsize='18')
plt.ylabel('UE serviced after UABS Implementation [%]', fontsize='18')
plt.title('SERVICE DIFFERENCE FOR UE BETWEEN INITIAL LOSS AND AFTER UABS IMPLEMENTATION', fontsize='18')
plt.grid(True)

# Show both figures
plt.show()

# 2. # ... (your existing code for experiments and results)


plt.hist([[len(x['best_specimen']['specimen'].chromosomes) for x in exp_pn.results], ])
# plt.xticks(np.arange(5), ('1', '2', '2', '3', '4'))
plt.autoscale(tight=True)
plt.grid('False')
plt.legend(loc='upper right')
plt.setp(plt.gca().get_legend().get_texts())
plt.xlabel('NUMBER OF PATHS SELECTED FROM THE BEST SPECIMEN')
plt.ylabel('NUMBER OF EXPERIMENTS')
plt.title('NUMBER OF DETERMINED PATHS FOR ALL EXPERIMENTS FOR ALL DISTRIBUIONS FOR UE AND BS')
plt.gcf().set_size_inches(17, 8.5)
plt.show()

plt.plot([i for i in range(10, 100, 10)],
         gaussian_filter1d([max([len(y['candidates']) for y in list(filter(lambda x: x['loss'] == k,
                                                                           [x for x in exp_pn.results]))]) for k in
                            range(10, 100, 10)], sigma=1), '-^r', markersize=12, label='UBPU')
plt.show()

iterations = range(1, len(exp_pn.results[element]['candidates']) + 1)
coverage_ratios = [(100 - candidate['best_score']) for candidate in exp_pn.results[element]['candidates']]

# Plot the coverage ratio over iterations
plt.plot(iterations, coverage_ratios, marker='o')
plt.xlabel('Iterations')
plt.ylabel('Coverage Ratio [%]')
plt.title('Coverage Ratio Over Iterations for Loss %d' % (exp_pn.results[element]['loss']))
plt.grid(True)
plt.show()

loss_percentages = [i for i in range(10, 100, 10)]
max_scores = [0.75, 0.85, 0.92, 0.95, 0.97, 0.98, 0.99, 1.0]

# Plotting the maximum scores
plt.plot(loss_percentages, max_scores, marker='o', linestyle='-', color='b')
plt.xlabel('Base Station Loss Percentage')
plt.ylabel('Maximum Score')
plt.title('Maximum Score vs. Base Station Loss Percentage')
plt.grid(True)
plt.show()

# ***********************************************************************

# -------------------------------------------------
fig1 = plt.figure(1)
col = ['-*b', '-og', '-^r', '-vc', '-Dm', '-sy', '-Pk', '-pb', '-Xg']
lab = ['UBPP', 'UBPN', 'UBPU', 'UBNP', 'UBNN', 'UBNU', 'UBUP', 'UBUN', 'UBUU']
for j, k in enumerate((exp_pn)):
    prt = []
for cap in k.capacities:
    [a, b, c] = plt.hist([x[1] for x in cap], 2)
prt.append((a[1]) * 100 / (a[0] + a[1]))
fig2 = plt.figure(2)
plt.plot([x for x in range(10, 100, 10)], [100 - x for x in prt[::-1]], col[j], markersize=9, label=lab[j])
fig1 = plt.figure(1)
fig2 = plt.figure(2)
plt.legend()
plt.xlabel('BS loss [%]', fontsize='18')
plt.ylabel('UE serviced after UABS Implementation [%]', fontsize='18')
plt.title('SERVICE DIFFERENCE FOR UE BETWEEN INITIAL LOSS AND AFTER UABS IMPLEMENTATION', fontsize='18')
plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
plt.grid(True)
fig2.show()
print(service_ratio(exp_pn.results[5]['best_specimen']['specimen'].chromosomes, exp_pn.unserviced[1], 200000))
print([list(filter(lambda x: x == 90, [x['loss'] for x in k.results])) for k in (exp_pn)])

tot = []
#
col = ['-og']
lab = ['UBPN']
#
for dis in exp_pn:
    cov_dis = []
    for loss in range(10, 100, 10):
        cov = []
        for itt in [x for x in dis['results']]:
            if (itt['loss'] == loss):
                cov.append(
                    service_ratio(itt['best_specimen']['specimen'].chromosomes, dis.unserviced[int((loss) / 10) - 1],
                                  200000))
        print('The coverage ratio is %g with a loss of %d' % (min(cov), loss))
        cov_dis.append(min(cov))
    plt.plot([i for i in range(10, 100, 10)], cov_dis)
    tot.append(cov_dis)
#
plt.hist(tot, label=['UBPN'])
plt.legend()
plt.setp(plt.gca().get_legend().get_texts(), fontsize='20')
plt.xlabel('BS loss [%]', fontsize='18')
plt.ylabel('UE serviced [%]', fontsize='18')
plt.title('UE WITH SERVICE AFTER BS LOSS WITH UABS IMPLEMENTATION', fontsize='18')
plt.grid(True)

print(service_ratio(itt['best_specimen']['specimen'].chromosomes, exp_pn.unserviced[1], 200000))

print('Ready!')

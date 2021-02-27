from deap.tools.support import Statistics
import pandas as pd
import numpy as np
import random
from deap import base
from deap import creator
from deap import tools
import numpy
from bitstring import BitArray

import itertools
import copy
import pickle
import time
import matplotlib.pyplot as plt

import multiprocessing

from modular.URDF_writer import UrdfWriter
from modular.optimization.test import iacobelli_IK
from modular.optimization.test_class import test
from modular.optimization.pickle_utilities import load_pickle

from romity_collides import pyromiti_collision as pyrom

# Create grid
# x = list(numpy.linspace(0.0, 1.4, 8))
x = list(numpy.linspace(0.0, 0.6, 4))
y = list(numpy.linspace(0.0, 0.6, 4))
coordinates = list(itertools.product(x, y))
angles =  [0.0, 1.57] #, 3.14, -1.57]

TYPES_OF_MODULES = 4
MIN_NUM_OF_MODULES = 4
MAX_NUM_OF_MODULES = 8

INVALID_RESULT = 99999999

n_DOFs = 5
n_modules = 6 #5
max_modules_after_links = 2
module_encoding_length = 2
final_index = n_modules*module_encoding_length
bits_for_angle_offset = 1
bits_for_xy = 4

def from_bitlist_to_int(bitlist):
    out = 0
    for bit in bitlist:
        out = (out << 1) | bit
        return out

def from_int_to_bitlist(n):
    return [int(x) for x in '{:02b}'.format(n)] # {:02b} to represent the number in 2-bits string

def generate_robot_set(n):
    # 3 Yaws, 3 Elbows, 1 straight link, 1 elbow link
    elements = [0, 1, 1, 1, 2, 2]#, 0, 0] #[1, 1, 0, 0, 0, 2, 3]
    modules_combinations = set(itertools.permutations(elements, n))
    # We remove from the robot set all the combinations that have the link modules (0 or 3) in the first (n-2) positions
    # Equivalent to a constraint on the max weight the link can withstand of ~6kg  
    elements_to_remove = []
    for comb in modules_combinations:
        if comb.index(0) < (n-max_modules_after_links-1):
            elements_to_remove.append(comb)
    for comb in elements_to_remove:    
        modules_combinations.remove(comb)
    # we make a copy of the set to have the same combinations also with module 3 (substituting 0)
    # TODO: This is ugly. Generate all and remove the ones with both 0 and 3
    modules_combinations_2 = set()
    for module_comb in modules_combinations:
        module_comb2 = tuple([i if i != 0 else 3 for i in module_comb])
        modules_combinations_2.add(module_comb2)
    modules_combinations = modules_combinations.union(modules_combinations_2)
    # Convert integer representing modules to bit strings
    expanded_modules_combinations = []
    for comb in modules_combinations:
        expanded_comb =[]
        for module in comb:
            expanded_comb += from_int_to_bitlist(module)
        expanded_modules_combinations.append(expanded_comb)
    return expanded_modules_combinations

# the random initialization of the genetic algorithm is done here
def generate_random_robot():
    # n_modules = random.randint(MIN_NUM_OF_MODULES,MAX_NUM_OF_MODULES)
    angle_offset = [random.randint(0,1) for i in range(bits_for_angle_offset)]
    xy = [random.randint(0,1) for i in range(bits_for_xy)] #, random.randint(0,1)]
    robot_allowed = False
    while not robot_allowed:    
        robot = list(random.choice(generate_robot_set(n_modules)))
        # flat_robot_list = [item for sublist in robot_list for item in sublist]
        robot_config = robot + angle_offset + xy
        robot_allowed = check_robot_is_allowed(robot_config)
        if not robot_allowed:
            print("There must be a bug in the robot generation!!!!!!!!!!!!")
    return robot_config

# this is the function used by the algorithm for evaluation
# I chose it to be the absolute difference of the number of calories in the planning and the goal of calories
def evaluate(individual):
    individual = individual[0]
    robot, xy, angle = chunk_individual(individual)
    fitness, q, pose, n_dofs = evaluate_robot(robot, xy, angle)
    return fitness, q, pose, n_dofs

def check_robot_is_allowed(individual):
    robot_structure, xy, angle = chunk_individual(individual)
    chunked_robot_structure = [robot_structure[i:i + module_encoding_length] for i in range(0, len(robot_structure), module_encoding_length)]  
    robot_structure = [BitArray(l).uint for l in chunked_robot_structure]
    # 0 and 3 can appear only once
    contains_duplicates = (robot_structure.count(0) + robot_structure.count(3))  > 2
    # constraint on max weight holdable by links (max two modules after)
    violates_weight_constraint = False
    if 0 in robot_structure:
        violates_weight_constraint = robot_structure.index(0) < (n_modules-max_modules_after_links-1) or violates_weight_constraint
    if 3 in robot_structure:
        violates_weight_constraint = robot_structure.index(3) < (n_modules-max_modules_after_links-1) or violates_weight_constraint
    # check there is not too many dofs
    contains_too_many_DOFs = (robot_structure.count(1) + robot_structure.count(2))  > n_DOFs
    # contains_too_many_DOFs = False
    # check we don't go over the limit of 3 module for each type
    # too_many_elbows = robot_structure.count(1) > 3
    too_many_elbows = False
    too_many_yaws = robot_structure.count(2) > 3

    is_allowed = not too_many_elbows and not too_many_yaws and not contains_duplicates and not violates_weight_constraint and not contains_too_many_DOFs

    return is_allowed

def MYmutFlipBit(individual, indpb):
    """Flip the value of the attributes of the input individual and return the
    mutant. The *individual* is expected to be a :term:`sequence` and the values of the
    attributes shall stay valid after the ``not`` operator is called on them.
    The *indpb* argument is the probability of each attribute to be
    flipped. This mutation is usually applied on boolean individuals.

    :param individual: Individual to be mutated.
    :param indpb: Independent probability for each attribute to be flipped.
    :returns: A tuple of one individual.

    This function uses the :func:`~random.random` function from the python base
    :mod:`random` module.
    """
    for i in xrange(len(individual)):
        if random.random() < indpb:
            test = copy.deepcopy(individual)
            test[i] = type(test[i])(not test[i])
            robot_allowed = check_robot_is_allowed(test)
            if robot_allowed:
                individual[i] = test[i]

    return individual,


def MYcxTwoPoint(ind1, ind2):
    """Executes a two-point crossover on the input :term:`sequence`
    individuals. The two individuals are modified in place and both keep
    their original length.

    :param ind1: The first individual participating in the crossover.
    :param ind2: The second individual participating in the crossover.
    :returns: A tuple of two individuals.

    This function uses the :func:`~random.randint` function from the Python
    base :mod:`random` module.
    """
    copy1 = copy.deepcopy(ind1)
    copy2 = copy.deepcopy(ind2)
    size = min(len(copy1), len(copy2))
    cxpoint1 = random.randint(1, size)
    cxpoint2 = random.randint(1, size - 1)
    if cxpoint2 >= cxpoint1:
        cxpoint2 += 1
    else:  # Swap the two cx points
        cxpoint1, cxpoint2 = cxpoint2, cxpoint1

    copy1[cxpoint1:cxpoint2], copy2[cxpoint1:cxpoint2] \
        = copy2[cxpoint1:cxpoint2], copy1[cxpoint1:cxpoint2]

    if check_robot_is_allowed(copy1) and check_robot_is_allowed(copy2):
        ind1[:] = copy1
        ind2[:] = copy2
        return ind1, ind2
    else:
        return MYcxTwoPoint(ind1, ind2)


def evaluate_robot(robot_structure, xy, angle_offset):
    # print("Evaluating:")
    # print(robot_structure)
    # print(angle_offset)
    # print(xy)
    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    # urdf_writer.add_table()

    # Add 1st chain base
    urdf_writer.add_socket(xy[0], xy[1], 0.0, angle_offset)

    chunked_robot_structure = [robot_structure[i:i + module_encoding_length] for i in range(0, len(robot_structure), module_encoding_length)]  
    robot_structure = [BitArray(l).uint for l in chunked_robot_structure]

    # Add 1st chain
    for module in robot_structure:
        if module == 1:
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 2:
            data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 3:
            # data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            # homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
            data = urdf_writer.add_module('module_link_elbow_90_B.yaml', 3.14, False)
            # data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)
        else:
            # continue
            data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)

    # Add a simple virtual end-effector
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)

    urdf = urdf_writer.process_urdf()
    # urdf = urdf_writer.write_urdf()
    srdf = urdf_writer.write_srdf(homing_joint_map)
    # joint_map = urdf_writer.write_joint_map()
    # lowlevel_config = urdf_writer.write_lowlevel_config()
    # probdesc = urdf_writer.write_hardcodedproblem_description("ee_B", "ee_A")

    # urdf_writer.deploy_robot("pino_moveit")

    # This robot breaks Opensot for some reason. skip it
    if angle_offset == -1.57 and robot_structure == ((0, 1, 0), (0, 1)):
        return

    # x_target, y_target, z_target = 0.3, 0.0, 0.0
    # result, q, pose, n_dofs = test(string, x_target, y_target, z_target)
    # if result is None:
        # result = INVALID_RESULT
    # return result, q, pose, n_dofs

    tot_result = 0.0

    poses_to_reach = [  [0.5,         0.0,        0.1],
                        [0.5,         0.0-0.1,    0.1],
                        [0.5+0.12,    0.0,        0.1],
                        [0.5+0.12,    0.0-0.1,    0.1],
                        [0.5+0.12,    0.0,        0.1+0.12],
                        [0.5+0.12,    0.0-0.1,    0.1+0.12],
                        [0.5,         0.0,        0.1+0.12],
                        [0.5,         0.0-0.1,    0.1+0.12]]
    q_sol = []
    final_poses = []

    cm = pyrom.CollisionManager()
    cm.load(urdf, srdf)

    # result, q, pose, n_dofs = iacobelli_IK()
    for pose in poses_to_reach:
        result, q, pose, n_dofs = test(urdf, pose[0], pose[1], pose[2])
        if result is not None:
            links_in_collision = cm.collides(q.tolist())
            if links_in_collision:    
                print(links_in_collision)
                tot_result = INVALID_RESULT
                break
            tot_result+= result
            q_sol.append(q)
            final_poses.append(pose)
        else:    
            tot_result = INVALID_RESULT
            break

    return tot_result, q_sol, final_poses, n_dofs


def chunk_individual(individual):
    # individual = individual[0]
    robot_structure = individual[:final_index]
    angle_offset = angles[BitArray(individual[final_index:final_index+bits_for_angle_offset]).uint]
    xy = coordinates[BitArray(individual[final_index+bits_for_angle_offset:final_index+bits_for_angle_offset+bits_for_xy]).uint] # 7
    
    return robot_structure, xy, angle_offset

def test_robot(individual):
    individual = individual[0]
    robot_structure, xy, angle_offset = chunk_individual(individual)
    print("Testing:")
    print(robot_structure)
    print(angle_offset)
    print(xy)

    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    # urdf_writer.add_table()

    # Add 1st chain base
    urdf_writer.add_socket(xy[0], xy[1], 0.0, angle_offset)

    chunked_robot_structure = [robot_structure[i:i + module_encoding_length] for i in range(0, len(robot_structure), module_encoding_length)]  
    robot_structure = [BitArray(l).uint for l in chunked_robot_structure]

    # Add 1st chain
    for module in robot_structure:
        if module == 1:
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 2:
            data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 3:
            # data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            # homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
            data = urdf_writer.add_module('module_link_elbow_90_B.yaml', 3.14, False)
            # data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)
        else:
            # continue
            data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)

    # Add a simple virtual end-effector
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)

    string = urdf_writer.process_urdf()
    urdf = urdf_writer.write_urdf()
    srdf = urdf_writer.write_srdf(homing_joint_map)
    joint_map = urdf_writer.write_joint_map()
    lowlevel_config = urdf_writer.write_lowlevel_config()

    urdf_writer.deploy_robot("pino_moveit")

    # This robot breaks Opensot for some reason. skip it
    if angle_offset == -1.57 and robot_structure == ((0, 1, 0), (0, 1)):
        return

    tot_result = 0.0

    poses_to_reach = [  [0.45,         0.0,        0.15],
                        [0.45,         0.0-0.1,    0.15],
                        [0.45+0.12,    0.0,        0.15],
                        [0.45+0.12,    0.0-0.1,    0.15],
                        [0.45+0.12,    0.0,        0.15+0.12],
                        [0.45+0.12,    0.0-0.1,    0.15+0.12],
                        [0.45,         0.0,        0.15+0.12],
                        [0.45,         0.0-0.1,    0.15+0.12]]
    q_sol = []
    final_poses = []

    cm = pyrom.CollisionManager()
    cm.load(urdf, srdf)

    # result, q, pose, n_dofs = iacobelli_IK()
    for pose in poses_to_reach:
        result, q, pose, n_dofs = test(string, pose[0], pose[1], pose[2])
        if result is not None:
            links_in_collision = cm.collides(q.tolist())
            if links_in_collision:
                print(links_in_collision)
                tot_result = INVALID_RESULT
                break
            tot_result+= result
            q_sol.append(q)
            final_poses.append(pose)
        else:    
            tot_result = INVALID_RESULT
            break

    return tot_result, q_sol, final_poses, n_dofs


# # this is the definition of the total genetic algorithm is executed, it is almost literally copied from the deap library
# def main_2objectives(verbose):
#     # this is the setup of the deap library: registering the different function into the toolbox
#     creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
#     creator.create("Individual", list, fitness=creator.FitnessMin, ik_solution=None, pose=None, dofs=None)

#     toolbox = base.Toolbox()

#     toolbox.register("generate_random_robot", generate_random_robot)

#     toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.generate_random_robot, n=1)
#     toolbox.register("population", tools.initRepeat, list, toolbox.individual)

#     toolbox.register("evaluate", evaluate)
#     toolbox.register("mate", MYcxTwoPoint)
#     toolbox.register("mutate", MYmutFlipBit, indpb=0.1)
#     toolbox.register("select", tools.selTournament, tournsize=2)

#     # as an example, this is what a population of 10 shopping lists looks like
#     # toolbox.population(n=10)

#     stats_fit = tools.Statistics(lambda ind: ind.fitness.values)
#     stats_size = tools.Statistics(key=lambda ind: ind.dofs)
#     mstats = tools.MultiStatistics(fitness=stats_fit, size=stats_size)

#     mstats.register("avg", np.mean)
#     mstats.register("std", np.std)
#     mstats.register("min", np.min)
#     mstats.register("max", np.max)

#     logbook = tools.Logbook()
#     hof = tools.HallOfFame(1)

#     pool = multiprocessing.Pool(processes=10)
#     toolbox.register("map", pool.map)

#     pop_size = 20 # 20
#     pop = toolbox.population(n=pop_size) 
    
#     # Evaluate the entire population
#     fitnesses = list(toolbox.map(toolbox.evaluate, pop))
#     for ind, fit in zip(pop, fitnesses):
#         ind.fitness.values = fit[0],
#         ind.ik_solution = fit[1]
#         ind.pose = fit[2]
#         ind.dofs = fit[3]

#     hof.update(pop)

#     # CXPB  is the probability with which two individuals
#     #       are crossed
#     #
#     # MUTPB is the probability for mutating an individual
#     CXPB, MUTPB = 0.5, 0.25

#     # Extracting all the fitnesses of
#     fits = [ind.fitness.values[0] for ind in pop]

#     # Variable keeping track of the number of generations
#     g = 0

#     record = mstats.compile(pop)
#     logbook.record(gen=g, evals=pop_size, **record) #pop=pop, 
#     if verbose:
#         print("Statistics")
#         # print(record)
#         print(logbook.stream)

#     # Begin the evolution
#     while g < 50: # 100
#         # A new generation
#         g = g + 1
#         print("-- Generation %i --" % g)
#         for ind in pop:
#             ind = ind[0]
#             print(ind)
#             # r, xy, angle = chunk_individual(ind)
#             # print(r, xy, angle)

#         # Select the next generation individuals
#         offspring = toolbox.select(pop, len(pop))
#         # Clone the selected individuals
#         offspring = list(toolbox.map(toolbox.clone, offspring))

#         # Apply crossover and mutation on the offspring
#         for child1, child2 in zip(offspring[::2], offspring[1::2]):
#             if random.random() < CXPB:
#                 toolbox.mate(child1[0], child2[0])
#                 del child1.fitness.values
#                 del child2.fitness.values

#         for mutant in offspring:
#             if random.random() < MUTPB:
#                 toolbox.mutate(mutant[0])
#                 del mutant.fitness.values

#         # Evaluate the individuals with an invalid fitness
#         invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
#         fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
#         for ind, fit in zip(invalid_ind, fitnesses):
#             ind.fitness.values = fit[0],
#             ind.ik_solution = fit[1]
#             ind.pose = fit[2]
#             ind.dofs = fit[3]

#         hof.update(offspring)

#         pop[:] = offspring

#         record = mstats.compile(pop)
#         logbook.record(gen=g, evals=pop_size, **record) #pop=pop, 
#         if verbose:
#             print("Statistics")
#             # print(record)
#             print(logbook.stream)

#         # Gather all the fitnesses in one list and print the stats
#         # fits = [ind.fitness.values[0] for ind in pop]

#         # length = len(pop)
#         # mean = sum(fits) / length
#         # sum2 = sum(x * x for x in fits)
#         # std = abs(sum2 / length - mean ** 2) ** 0.5

#         # print(min(fits), max(fits), mean, std)

#     best = pop[np.argmin([toolbox.evaluate(x)[0] for x in pop])]

#     print(logbook)

#     # gen = logbook.select("gen")
#     # fit_mins = logbook.chapters["fitness"].select("min")
#     # size_avgs = logbook.chapters["size"].select("avg")

#     # fig, ax1 = plt.subplots()
#     # line1 = ax1.plot(gen, fit_mins, "b-", label="Minimum Fitness")
#     # ax1.set_xlabel("Generation")
#     # ax1.set_ylabel("Fitness", color="b")
#     # for tl in ax1.get_yticklabels():
#     #     tl.set_color("b")

#     # ax2 = ax1.twinx()
#     # line2 = ax2.plot(gen, size_avgs, "r-", label="Average Size")
#     # ax2.set_ylabel("Size", color="r")
#     # for tl in ax2.get_yticklabels():
#     #     tl.set_color("r")

#     # lns = line1 + line2
#     # labs = [l.get_label() for l in lns]
#     # ax1.legend(lns, labs, loc="center right")

#     # plt.show()

#     print("ZIO CAGNACCIO!!!!!!!!!!!!!!!!!!!")
#     print(best)
#     print("IK solution:")
#     print(best.ik_solution)
#     print("final pose:")
#     print(best.pose)
#     print("DOFs:")
#     print(best.dofs)
#     print("best fitness value:")
#     print(best.fitness.values[0])
#     print("best_population:")
#     for p in pop:
#         p = p[0]
#         print(p)
#         # r, xy, angle = chunk_individual(p)
#         # print(r, xy, angle)

#     pickle_path = "/home/tree/pickles/logbooks/"
#     date = time.strftime("%Y%m%d-%H%M%S")
#     filename = pickle_path + date + '_logbook.pkl'
#     with open(filename, 'wb') as output:
#         pickle.dump(logbook, output)
#     filename = pickle_path + date + '_population.pkl'
#     with open(filename, 'wb') as output:
#         pickle.dump(pop, output)
#     filename = pickle_path + date + '_hof.pkl'
#     with open(filename, 'wb') as output:
#         pickle.dump(hof, output)

#     return best


# this is the definition of the total genetic algorithm is executed, it is almost literally copied from the deap library
def main(verbose):
    # this is the setup of the deap library: registering the different function into the toolbox
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin, ik_solution=None, pose=None, dofs=None)

    toolbox = base.Toolbox()

    toolbox.register("generate_random_robot", generate_random_robot)

    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.generate_random_robot, n=1)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("evaluate", evaluate)
    toolbox.register("mate", MYcxTwoPoint)
    toolbox.register("mutate", MYmutFlipBit, indpb=0.2)
    toolbox.register("select", tools.selTournament, tournsize=5)

    # as an example, this is what a population of 10 shopping lists looks like
    # toolbox.population(n=10)

    stats_fit = tools.Statistics(lambda ind: ind.fitness.values)
    stats_size = tools.Statistics(key=lambda ind: ind.dofs)
    mstats = tools.MultiStatistics(fitness=stats_fit, size=stats_size)

    mstats.register("avg", np.mean)
    mstats.register("std", np.std)
    mstats.register("min", np.min)
    mstats.register("max", np.max)

    logbook = tools.Logbook()
    hof = tools.HallOfFame(20)

    pool = multiprocessing.Pool(processes=10)
    toolbox.register("map", pool.map)

    pop_size = 60 # 50
    pop = toolbox.population(n=pop_size) 
    
    # Evaluate the entire population
    fitnesses = list(toolbox.map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit[0],
        ind.ik_solution = fit[1]
        ind.pose = fit[2]
        ind.dofs = fit[3]

    hof.update(pop)

    # CXPB  is the probability with which two individuals
    #       are crossed
    #
    # MUTPB is the probability for mutating an individual
    CXPB, MUTPB = 0.5, 0.25

    # Extracting all the fitnesses of
    fits = [ind.fitness.values[0] for ind in pop]

    # Variable keeping track of the number of generations
    g = 0

    record = mstats.compile(pop)
    logbook.record(gen=g, evals=pop_size, **record) #pop=pop, 
    if verbose:
        # print("Statistics")
        # print(record)
        print(logbook.stream)

    # Begin the evolution
    while g < 100: # 100
        # A new generation
        g = g + 1
        print("-- Generation %i --" % g)
        for ind in pop:
            ind = ind[0]
            print(ind)
            # r, xy, angle = chunk_individual(ind)
            # print(r, xy, angle)

        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(toolbox.map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1[0], child2[0])
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant[0])
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit[0],
            ind.ik_solution = fit[1]
            ind.pose = fit[2]
            ind.dofs = fit[3]

        hof.update(offspring)

        pop[:] = offspring

        record = mstats.compile(pop)
        logbook.record(gen=g, evals=pop_size, **record) #pop=pop, 
        if verbose:
            # print("Statistics")
            # print(record)
            print(logbook.stream)

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        # length = len(pop)
        # mean = sum(fits) / length
        # sum2 = sum(x * x for x in fits)
        # std = abs(sum2 / length - mean ** 2) ** 0.5

        # print(min(fits), max(fits), mean, std)

    best = pop[np.argmin([toolbox.evaluate(x)[0] for x in pop])]

    print(logbook)
    print(hof)

    # gen = logbook.select("gen")
    # fit_mins = logbook.chapters["fitness"].select("min")
    # size_avgs = logbook.chapters["size"].select("avg")

    # fig, ax1 = plt.subplots()
    # line1 = ax1.plot(gen, fit_mins, "b-", label="Minimum Fitness")
    # ax1.set_xlabel("Generation")
    # ax1.set_ylabel("Fitness", color="b")
    # for tl in ax1.get_yticklabels():
    #     tl.set_color("b")

    # ax2 = ax1.twinx()
    # line2 = ax2.plot(gen, size_avgs, "r-", label="Average Size")
    # ax2.set_ylabel("Size", color="r")
    # for tl in ax2.get_yticklabels():
    #     tl.set_color("r")

    # lns = line1 + line2
    # labs = [l.get_label() for l in lns]
    # ax1.legend(lns, labs, loc="center right")

    # plt.show()

    print("ZIO CAGNACCIO!!!!!!!!!!!!!!!!!!!")
    print(best)
    print("IK solution:")
    print(best.ik_solution)
    print("final pose:")
    print(best.pose)
    print("DOFs:")
    print(best.dofs)
    print("best fitness value:")
    print(best.fitness.values[0])
    print("best_population:")
    for p in pop:
        p = p[0]
        print(p)
        # r, xy, angle = chunk_individual(p)
        # print(r, xy, angle)

    pickle_path = "/home/tree/pickles/logbooks/"
    date = time.strftime("%Y%m%d-%H%M%S")
    filename = pickle_path + date + '_logbook.pkl'
    with open(filename, 'wb') as output:
        pickle.dump(logbook, output)
    filename = pickle_path + date + '_population.pkl'
    with open(filename, 'wb') as output:
        pickle.dump(pop, output)
    filename = pickle_path + date + '_hof.pkl'
    with open(filename, 'wb') as output:
        pickle.dump(hof, output)

    return best


if __name__ == '__main__':

    best_solution = main(True) 
    # check_robot_is_allowed([1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0])
    # a, b, c, d = test_robot([[1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0]])
    # check_robot_is_allowed([1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0])
    # a, b, c, d = test_robot([[1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0]])
    # print("Fitness:")
    # print a
    # print("IK Solutions:")
    # print b
    # print("Final poses:")
    # print c
    # print("# of DOFs:")
    # print d
    # a, b, c, d = test_robot([[1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1]]) #[[0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1]]
    # a = load_pickle("/home/tree/pickles/logbooks/4-5DOFs_02_02_no_link/20210224-152627_population.pkl")
    # print(a[0][0])
    # test_robot(a[0][0])
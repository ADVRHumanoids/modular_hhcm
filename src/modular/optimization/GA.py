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

from modular.URDF_writer import UrdfWriter
from modular.optimization.test import iacobelli_IK

# Create grid
x = list(numpy.linspace(0.0, 1.4, 8))
y = list(numpy.linspace(0.0, 0.6, 4))
coordinates = list(itertools.product(x, y))
angles =  [0.0, 1.57, 3.14, -1.57]

TYPES_OF_MODULES = 4
MIN_NUM_OF_MODULES = 4
MAX_NUM_OF_MODULES = 8

n_dofs = 5
module_encoding_length = 2
final_index = n_dofs*module_encoding_length

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
    # n_dofs = random.randint(MIN_NUM_OF_MODULES,MAX_NUM_OF_MODULES)
    angle_offset = [random.randint(0,1), random.randint(0,1)]
    xy = [random.randint(0,1), random.randint(0,1), random.randint(0,1), random.randint(0,1), random.randint(0,1)]
    robot = list(random.choice(generate_robot_set(n_dofs)))
    # flat_robot_list = [item for sublist in robot_list for item in sublist]
    robot_config = robot + angle_offset + xy
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
    # 0 can appear only once
    contains_duplicates = robot_structure.count(0) > 1
    return not contains_duplicates

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
    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    urdf_writer.add_table()

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
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
            # data = urdf_writer.add_module('module_link_elbow_90_B.yaml', 0, False)
        else:
            continue
        # elif module == 3:
        #     data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)

    # Add a simple virtual end-effector
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)

    # string = urdf_writer.process_urdf()
    urdf = urdf_writer.write_urdf()
    # srdf = urdf_writer.write_srdf(homing_joint_map)
    # joint_map = urdf_writer.write_joint_map()
    # lowlevel_config = urdf_writer.write_lowlevel_config()
    # probdesc = urdf_writer.write_hardcodedproblem_description("ee_B", "ee_A")

    # urdf_writer.deploy_robot("pino_moveit")

    # This robot breaks Opensot for some reason. skip it
    if angle_offset == -1.57 and robot_structure == ((0, 1, 0), (0, 1)):
        return

    result, q, pose, n_dofs = iacobelli_IK()

    if result is None:
        result = 999999999999999999

    return result, q, pose, n_dofs

def chunk_individual(individual):
    # individual = individual[0]
    robot_structure = individual[:final_index]
    print(robot_structure)
    angle_offset = angles[BitArray(individual[final_index:final_index+2]).uint]
    print(angle_offset)
    xy = coordinates[BitArray(individual[final_index+2:final_index+4]).uint]
    print(xy)
    return robot_structure, xy, angle_offset

def test_robot(individual):
    individual = individual[0]
    robot_structure, xy, angle_offset = chunk_individual(individual)

    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    urdf_writer.add_table()

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
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
            # data = urdf_writer.add_module('module_link_elbow_90_B.yaml', 0, False)
        else:
            continue
        # elif module == 3:
        #     data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)

    # Add a simple virtual end-effector
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)

    # string = urdf_writer.process_urdf()
    urdf = urdf_writer.write_urdf()
    srdf = urdf_writer.write_srdf(homing_joint_map)
    joint_map = urdf_writer.write_joint_map()
    lowlevel_config = urdf_writer.write_lowlevel_config()
    # probdesc = urdf_writer.write_hardcodedproblem_description("ee_B", "ee_A")

    urdf_writer.deploy_robot("pino_moveit")

    # This robot breaks Opensot for some reason. skip it
    if angle_offset == -1.57 and robot_structure == ((0, 1, 0), (0, 1)):
        return

    result, q, pose, n_dofs = iacobelli_IK()

    if result is None:
        result = 999999999999999999

    return result, q, pose, n_dofs

# this is the setup of the deap library: registering the different function into the toolbox
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin, ik_solution=None, pose=None, dofs=None)

toolbox = base.Toolbox()

toolbox.register("generate_random_robot", generate_random_robot)

toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.generate_random_robot, n=1)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("evaluate", evaluate)
toolbox.register("mate", MYcxTwoPoint)
toolbox.register("mutate", MYmutFlipBit, indpb=0.1)
toolbox.register("select", tools.selTournament, tournsize=2)

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
hof = tools.HallOfFame(1)

# this is the definition of the total genetic algorithm is executed, it is almost literally copied from the deap library
def main():
    pop = toolbox.population(n=20)
    
    # Evaluate the entire population
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit[0],
        ind.ik_solution = fit[1]
        ind.pose = fit[2]
        ind.dofs = fit[3]

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
    print("Statistics")
    print(record)
    logbook.record(gen=g, evals=30, **record)

    # Begin the evolution
    while g < 100:
        # A new generation
        g = g + 1
        print("-- Generation %i --" % g)
        for ind in pop:
            print(ind)

        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1[0], child2[0])
                if not check_robot_is_allowed(child1[0]) or not check_robot_is_allowed(child2[0]):
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant[0])
                if not check_robot_is_allowed(mutant[0]):
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                    print("LA MERDAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!!")
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit[0],
            ind.ik_solution = fit[1]
            ind.pose = fit[2]
            ind.dofs = fit[3]

        pop[:] = offspring

        record = mstats.compile(pop)
        print("Statistics")
        print(record)
        logbook.record(gen=g, evals=30, **record)


        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5

        print(min(fits), max(fits), mean, std)

    best = pop[np.argmin([toolbox.evaluate(x)[0] for x in pop])]

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
        print(p)
    return best

if __name__ == '__main__':

    # best_solution = main()
    test_robot([[0,0, 1,0, 0,1, 1,1, 0,1, 0,0, 0,0,0,1,1]])

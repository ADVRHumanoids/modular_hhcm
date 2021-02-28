from deap.tools.support import Statistics
import pandas as pd
import numpy as np
import random
from deap import base
from deap import creator
from deap import tools
import numpy
from bitstring import BitArray

from modular.optimization.pickle_utilities import load_pickle2


INVALID_RESULT = 99999999


# this is the setup of the deap library: registering the different function into the toolbox
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin, ik_solution=None, pose=None, dofs=None)

hof = load_pickle2("/home/tree/pickles/logbooks/9.buona/20210228-033204_hof.pkl")
print(hof)

i=0
for ind in hof:
    i+=1
    if ind.fitness.values[0] != INVALID_RESULT:
        print("##################################")
        print("Individual nr.{}: {}".format(i, ind))
        print("fitness: {}".format(ind.fitness))
        print("joint_positions: {}".format(ind.ik_solution))
        print("poses: {}".format(ind.pose))
        print("dofs: {}".format(ind.dofs))
    
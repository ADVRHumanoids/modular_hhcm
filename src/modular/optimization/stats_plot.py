from deap.tools.support import Statistics
import pandas as pd
import numpy as np
import random
from deap import base
from deap import creator
from deap import tools
import numpy
from bitstring import BitArray
import matplotlib.pyplot as plt

from modular.optimization.pickle_utilities import load_pickle2


INVALID_RESULT = 99999999


# this is the setup of the deap library: registering the different function into the toolbox
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin, ik_solution=None, pose=None, dofs=None)

logbook = load_pickle2("/home/tree/pickles/logbooks/9.buona/20210228-033204_logbook.pkl")
print(logbook)

# font = {'family' : 'normal',
#         'weight' : 'normal',
#         'size'   : 18}
# plt.rc('font', **font)

SMALL_SIZE = 14
MEDIUM_SIZE = 16
BIGGER_SIZE = 18

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

gen = logbook.select("gen")
# for i in range(len(gen)):
#     gen[i] = gen[i]-20
fit_mins = logbook.chapters["fitness"].select("min")
# for i in range(len(fit_mins)):
#     fit_mins[i] = fit_mins[i]-2200
size_avgs = logbook.chapters["size"].select("avg")
# for i in range(len(size_avgs)):
#     size_avgs[i] = size_avgs[i]*1.05

start_index = 0 #21

fig, ax1 = plt.subplots()
line1 = ax1.plot(gen[start_index:], fit_mins[start_index:], "b-", label="Minimum Fitness")
ax1.set_xlabel("Generation")
ax1.set_ylabel("Fitness", color="b")
for tl in ax1.get_yticklabels():
    tl.set_color("b")

ax2 = ax1.twinx()
line2 = ax2.plot(gen[start_index:], size_avgs[start_index:], "r-", label="Average Size")
ax2.set_ylabel("Size", color="r")
for tl in ax2.get_yticklabels():
    tl.set_color("r")

lns = line1 + line2
labs = [l.get_label() for l in lns]
ax1.legend(lns, labs, loc="center right")
ax1.xaxis.grid(True, which='both')
ax1.yaxis.grid(True, which='both')
# plt.grid(b=None, which='major', axis='both')
plt.show()
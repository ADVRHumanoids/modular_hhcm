import rbdl
import math
import numpy as np

import operator

from cartesian_interface.pyci_all import *

import rospy
from sensor_msgs.msg import JointState
import timeit
import subprocess
import shlex
import roslaunch

from deap import base
from deap import creator
from deap import gp
from deap import tools
from deap import algorithms

import sys
sys.path.insert(0, '/home/edoardo/advr-superbuild/external')
import src.modular.URDF_writer

# Instance of UrdfWriter class
urdf_writer = src.modular.URDF_writer.UrdfWriter()


def Joint1(child):
    data = urdf_writer.add_module('module_joint_elbow_B.yaml', 0, False)
    return data


def Joint2(child):
    data = urdf_writer.add_module('module_joint_elbow_B.yaml', 1.57, False)
    return data


def Joint3(child):
    data = urdf_writer.add_module('module_joint_elbow_B.yaml', 3.14, False)
    return data


def Joint4(child):
    data = urdf_writer.add_module('module_joint_elbow_B.yaml', -1.57, False)
    return data


# def Joint5(child):
#     return
#
#
# def Joint6(child):
#     return
#
#
# def Joint7(child):
#     return
#
#
# def Joint8(child):
#     return


def evaluate_manipulability(individual, cartesian_points):
    urdf_writer.__init__()
    data = urdf_writer.add_slave_cube(0)
    # Transform the tree expression in a callable function
    func = toolbox.compile(expr=individual)

    data = func(1)
    string = urdf_writer.process_urdf()
    # print(string)

    urdf = urdf_writer.write_urdf()
    srdf, cartesio_stack = urdf_writer.write_srdf()
    joint_map = urdf_writer.write_joint_map()

    # Evaluate

    # Create a new model
    # model = rbdl.Model()
    # model = rbdl.loadModel('/home/edoardo/advr-superbuild/configs/ADVR_shared/ModularBot/urdf/ModularBot.urdf')
    model = rbdl.loadModel('/home/edoardo/advr-superbuild/external/modular/urdf/ModularBot.urdf')

    # print(model)
    # print(model.q_size)
    # print(model.previously_added_body_id)
    # id = model.previously_added_body_id - 7
    # print(model.GetBody(id))
    # print(model.GetBodyName(2147483647))
    # print(model.GetBodyName(2147483648))
    # print(model.GetBodyName(2147483649))
    # print(model.GetBodyName(2147483650))
    # print(model.GetBodyName(2147483651))
    # print(model.GetBodyName(2147483652))
    # print(model.GetBodyName(2147483653))
    # print(model.GetBodyName(2147483654))
    # print(model.GetBodyName(2147483655))
    # print(model.GetBodyName(2147483656))
    # print(model.GetBodyName(2147483657))
    # print(model.GetBodyName(0))

    print(len(model.mBodies))
    i = 0
    while i < len(model.mBodies):
        print(model.GetBodyName(i))
        i += 1

    last_id = len(model.mBodies) - 1
    print(model.GetBodyName(last_id))

    # body = model.GetBody(model.previously_added_body_id)
    # body = rbdl.Body.fromPointer(model.GetBody(model.previously_added_body_id))
    # print(body.mMass)

    # Solve inverse kinematics for cartesian points

    # command_line = "roslaunch /home/edoardo/advr-superbuild/external/modular/launch/cartesio.launch"
    # args = shlex.split(command_line)
    #
    # cartesIO_server = subprocess.call(args)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cartesio_launch = "/home/edoardo/advr-superbuild/external/modular/launch/cartesio_local.launch"
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [cartesio_launch])
    launch.start()
    rospy.loginfo("started")

    #
    ci = pyci.CartesianInterfaceRos()

    target_pose = Affine3(pos=[0.3, 0.4, 0], rot=[1.0, .0, .0, .0])

    ee_name = model.GetBodyName(last_id)

    ci.setVelocityLimits(ee_name, 1000.0, 1000.0)

    ci.setAccelerationLimits(ee_name, 1000.0, 1000.0)

    ci.setTargetPose(ee_name, target_pose, 0.5)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(ee_name)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', '0.5'
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)

    print 'Exiting...'
    #cartesIO_server.terminate()
    launch.shutdown()

    # get q
    # compute manipulability
    # until IK is not ready, initialize all q to 1
    q = np.asarray(solution_msg.position)
    point_local = np.array([0., 0., 0.2])
    # Compute and print the jacobian (note: the output parameter
    # of the Jacobian G must have proper size!)
    G = np.zeros([3, model.qdot_size])
    rbdl.CalcPointJacobian(model, q, last_id, point_local, G)

    print(G)

    GGtrans = np.matmul(G, np.transpose(G))
    det = np.linalg.det(GGtrans)
    manipulability = math.sqrt(det) if det > 0 else 0

    print(manipulability)

    coord = rbdl.CalcBodyToBaseCoordinates(model, q, last_id, point_local)

    print(coord)

    return manipulability, model.q_size


pset = gp.PrimitiveSet("MAIN", arity=1)
pset.addPrimitive(Joint1, 1)
pset.addPrimitive(Joint2, 1)
pset.addPrimitive(Joint3, 1)
pset.addPrimitive(Joint4, 1)
# pset.addPrimitive(Joint5, 1)
# pset.addPrimitive(Joint6, 1)
# pset.addPrimitive(Joint7, 1)
# pset.addPrimitive(Joint8, 1)

# pset.addPrimitive(operator.sub, 2)
# pset.addPrimitive(operator.mul, 2)

creator.create("FitnessMulti", base.Fitness, weights=(1.0, -0.1))
creator.create("Individual", gp.PrimitiveTree, fitness=creator.FitnessMulti,
               pset=pset)

toolbox = base.Toolbox()
toolbox.register("expr", gp.genHalfAndHalf, pset=pset, min_=4, max_=7)
toolbox.register("individual", tools.initIterate, creator.Individual,
                 toolbox.expr)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("compile", gp.compile, pset=pset)

toolbox.register("evaluate", evaluate_manipulability, cartesian_points=[[1, 2, 3], [3, 2, 1]])
toolbox.register("select", tools.selNSGA2) # change this change a lot the results
toolbox.register("mate", gp.cxOnePoint)
toolbox.register("expr_mut", gp.genFull, min_=0, max_=2)
toolbox.register("mutate", gp.mutUniform, expr=toolbox.expr_mut, pset=pset)

toolbox.decorate("mate", gp.staticLimit(key=operator.attrgetter("height"), max_value=7))
toolbox.decorate("mutate", gp.staticLimit(key=operator.attrgetter("height"), max_value=7))


def main():
    # initialize ROS node
    rospy.init_node('cartesIO_client', anonymous=True)

    # random.seed(318)

    pop = toolbox.population(n=10)
    print pop
    hof = tools.HallOfFame(1)

    stats_fit = tools.Statistics(lambda ind: ind.fitness.values)
    stats_size = tools.Statistics(len)
    mstats = tools.MultiStatistics(fitness=stats_fit, size=stats_size)
    mstats.register("avg_manipul", np.mean, axis=0)
    mstats.register("std_manipul", np.std, axis=0)
    mstats.register("min_manipul", np.min, axis=0)
    mstats.register("max_manipul", np.max, axis=0)

    pop, log = algorithms.eaSimple(pop, toolbox, 0.5, 0.1, 10, stats=mstats,
                                   halloffame=hof, verbose=True)
    champ = 0
    for champ in hof:
        urdf_writer.__init__()
        data = urdf_writer.add_slave_cube(0)
        func = toolbox.compile(champ)
        data = func(1)
        string = urdf_writer.process_urdf()
        # print(string)

        urdf_writer.write_urdf()
        urdf_writer.write_srdf()
        urdf_writer.write_joint_map()

        print champ.fitness

    print(str(champ)+'\n\n')

    # nodes, edges, labels = gp.graph(champ)
    #
    # ### Graphviz Section ###
    # import pygraphviz as pgv
    #
    # g = pgv.AGraph()
    # g.add_nodes_from(nodes)
    # g.add_edges_from(edges)
    # g.layout(prog="dot")
    #
    # for i in nodes:
    #     n = g.get_node(i)
    #     n.attr["label"] = labels[i]
    #
    # g.draw("tree.pdf")

    print log
    return pop, log, hof


if __name__ == "__main__":
    main()

# tree = toolbox.individual()
# print(tree)
# print(tree.fitness)
# print(tree.root)
#
# data = urdf_writer.add_slave_cube(0)
#
# function = gp.compile(tree, pset)
# data = function(1)
# string = urdf_writer.process_urdf()
# # print(string)
#
# urdf_writer.write_urdf()
#
#
# # Create a new model
# model = rbdl.Model()
# # Create a joint from joint type
# joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")
#
# xtrans= rbdl.SpatialTransform()
# xtrans.r = np.array([0., 1., 0.])
# # You can print all types
# print (joint_rot_y)
# print (model)
# print (xtrans)
#
# model = rbdl.loadModel('/home/edoardo/advr-superbuild/configs/ADVR_shared/ModularBot/urdf/ModularBot.urdf')
#
# print(model)
# print(model.q_size)
# print(model.previously_added_body_id)
# id = model.previously_added_body_id - 7
# print(model.GetBody(id))
# print(model.GetBodyName(2147483647))
# print(model.GetBodyName(2147483648))
# print(model.GetBodyName(2147483649))
# print(model.GetBodyName(2147483650))
# print(model.GetBodyName(2147483651))
# print(model.GetBodyName(2147483652))
# print(model.GetBodyName(2147483653))
# print(model.GetBodyName(2147483654))
# print(model.GetBodyName(2147483655))
# print(model.GetBodyName(2147483656))
# print(model.GetBodyName(2147483657))
# print(model.GetBodyName(0))
#
#
#
#
#
# print(len(model.mBodies))
# i=0
# while i < len(model.mBodies):
#     print(model.GetBodyName(i))
#     i += 1
#
# last_id = len(model.mBodies) - 1
# print(model.GetBodyName(last_id))
#
# # body = model.GetBody(model.previously_added_body_id)
# # body = rbdl.Body.fromPointer(model.GetBody(model.previously_added_body_id))
# # print(body.mMass)
#
# q = np.ones(model.q_size)
# point_local = np.array([0., 0., 0.2])
# # Compute and print the jacobian (note: the output parameter
# # of the Jacobian G must have proper size!)
# G = np.zeros([3, model.qdot_size])
# rbdl.CalcPointJacobian(model, q, last_id, point_local, G)
#
# print(G)
#
# GGtrans = np.matmul(G, np.transpose(G))
# manipulability = math.sqrt(np.linalg.det(GGtrans))
#
# print(manipulability)
#
# coord = rbdl.CalcBodyToBaseCoordinates(model, q, last_id, point_local)
#
# print(coord)







# expr = gp.genFull(pset, min_=1, max_=3)
# tree = gp.PrimitiveTree(expr)
# print(str(tree))
#
# nodes, edges, labels = gp.graph(expr)
#
# ### Graphviz Section ###
# import pygraphviz as pgv
#
# g = pgv.AGraph()
# g.add_nodes_from(nodes)
# g.add_edges_from(edges)
# g.layout(prog="dot")
#
# for i in nodes:
#     n = g.get_node(i)
#     n.attr["label"] = labels[i]
#
# g.draw("tree.pdf")


# import matplotlib.pyplot as plt
# import networkx as nx
# from networkx.drawing.nx_agraph import graphviz_layout
#
# g = nx.Graph()
# g.add_nodes_from(nodes)
# g.add_edges_from(edges)
# pos = graphviz_layout(g, prog="dot")
#
# nx.draw_networkx_nodes(g, pos)
# nx.draw_networkx_edges(g, pos)
# nx.draw_networkx_labels(g, pos, labels)
# plt.show()

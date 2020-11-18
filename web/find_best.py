import pickle_utilities
from ik import get_manipulability
import numpy as np
import math
from dual_arm_optimization import Candidate, evaluate_robot


# def get_manipulabilty(jacobian):  # TODO: specify indexes to discard for manip. computation. Now z rot. is discarded
#     j = jacobian[:-1, :]
#     jT = j.T
#     jjT = j*j.T
#     det = np.linalg.det(jjT)
#     manip = np.sqrt(det)
#
#     eigs = np.linalg.eig(j)
#
#     # Consider the 5 - dof reduced vector
#     vector = [1, 0, 0, 0, 0]
#
#     # minimising this index correspond to maximize force tranmissibility
#     # We want to minimise the one on z direction and maximize the others
#     force_transmission_ratio_indexes = []
#     for i in vector:  # range(0, len(vector)-1):
#         u = np.array(vector)
#         ft_ratio = u.dot(jjT).dot(u)
#         force_transmission_ratio_indexes.append(ft_ratio)
#         vector.insert(0, vector.pop())
#
#     return manip, force_transmission_ratio_indexes


def evaluate_second(candidates):
    tau_list = []
    dist_list = []
    manip_list = []
    ratios_list = []
    for c in candidates:
        skip = False
        for chain in c.robot:
            for i in range(len(chain)):
                try:
                    if chain[i] == 0 and chain[i] == chain[i + 1]:
                        skip = True
                        break
                except IndexError:
                    pass
            else:
                continue  # only executed i[f the inner loop did NOT break
            break  # only executed if the inner loop DID break

        print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
        print "Robot:", c.robot
        print "X-Y 2nd base position:", c.xy
        print "Angle offset:", c.angle_offset

        if not skip:
            tau_norm = sum([np.linalg.norm(tau) for tau in c.result['tau']])
            distance = np.linalg.norm(c.xy)
            # m1, fr1 = get_manipulability(c.result['J1'], 3)
            # m2, fr2 = get_manipulability(c.result['J2'], 3)
            # manipulability = [m1, m2]
            # force_ratios = [fr1, fr2]
            manipulability = min(c.result['manip'])
            force_ratios = c.result['force_ratios']
        else:
            tau_norm = 1000
            distance = 1000
            manipulability = [0.0]
            force_ratios = []

        setattr(c, "tau_norm", tau_norm)
        setattr(c, "distance", distance)
        setattr(c, "manipulability", manipulability)
        setattr(c, "force_ratios", force_ratios)

        tau_list.append(tau_norm)
        dist_list.append(distance)
        manip_list.append(manipulability)
        ratios_list.append(force_ratios)
        print "Tau norm: ", tau_norm

    return tau_list, dist_list, manip_list, ratios_list


def evaluate(candidates):
    tau_list = []
    dist_list = []
    manip_list = []
    ratios_list = []
    for c in candidates:
        skip = False
        for chain in c.robot:
            for i in range(len(chain)):
                try:
                    if chain[i] == 0 and chain[i] == chain[i + 1]:
                        skip = True
                        break
                except IndexError:
                    pass
            else:
                continue  # only executed i[f the inner loop did NOT break
            break  # only executed if the inner loop DID break

        print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
        print "Robot:", c.robot
        print "X-Y 2nd base position:", c.xy
        print "Angle offset:", c.angle_offset
        print "Trasl. err. norm. 1:", c.result['tr_norm_1']
        print "Rot. err. norm. 1:", c.result['rot_norm_1']
        print "Trasl. err. norm. 2:", c.result['tr_norm_2']
        print "Rot. err. norm. 2:", c.result['rot_norm_2']
        if not skip:
            tau_norm = np.linalg.norm(c.result['tau_1']) + np.linalg.norm(c.result['tau_2'])
            distance = np.linalg.norm(c.xy)
            m1, fr1 = get_manipulability(c.result['J1'], 1)
            m2, fr2 = get_manipulability(c.result['J2'], 1)
            manipulability = [m1, m2]
            force_ratios = [fr1, fr2]
        else:
            tau_norm = 1000
            distance = 1000
            manipulability = [0.0, 0.0]
            force_ratios = [[], []]

        setattr(c, "tau_norm", tau_norm)
        setattr(c, "distance", distance)
        setattr(c, "manipulability", manipulability)
        setattr(c, "force_ratios", force_ratios)

        tau_list.append(tau_norm)
        dist_list.append(distance)
        manip_list.append(manipulability)
        ratios_list.append(force_ratios)
        print "Tau norm: ", tau_norm

    return tau_list, dist_list, manip_list, ratios_list

def filter_manipulability(candidates, manip_list):
    nan = 0.0
    idx_to_rmv = []
    for i in xrange(len(manip_list)):
        if manip_list[i][0] < 0.02 or manip_list[i][0] < 0.02:
            idx_to_rmv.append(i)
    print idx_to_rmv

def plot_bars2(tau_list, manip_list):
    import matplotlib.pyplot as plt
    # X = np.arange(3)
    fig, axs = plt.subplots(2)
    # ax = fig.add_axes([0,0,1,1])
    idxs = range(len(tau_list))
    axs[0].bar(idxs, tau_list, color='#1f77b4')
    axs[1].bar(idxs, manip_list, color='#ff7f0e')

    axs[0].grid(b=True, which='major', axis='y', linestyle=':')
    axs[1].grid(b=True, which='major', axis='y', linestyle=':')

    axs[0].set_ylim(0, max(tau_list) * 1.1)
    axs[0].set_xlim(-0.5, len(tau_list) - 0.5)
    axs[1].set_xlim(-0.5, len(tau_list) - 0.5)

    axs[0].spines['top'].set_visible(False)
    axs[0].spines['right'].set_visible(False)
    axs[0].spines['left'].set_visible(False)
    axs[1].spines['top'].set_visible(False)
    axs[1].spines['right'].set_visible(False)
    axs[1].spines['left'].set_visible(False)

    plt.show()


def plot_bars(tau_list, manip_list, manip_list_2):
    import matplotlib.pyplot as plt
    # X = np.arange(3)
    fig, axs = plt.subplots(3)
    # ax = fig.add_axes([0,0,1,1])
    idxs = range(len(tau_list))
    axs[0].bar(idxs, tau_list, color='#1f77b4')
    axs[1].bar(idxs, manip_list, color='#ff7f0e')
    axs[2].bar(idxs, manip_list_2, color='#2ca02c')# , color = 'b', width = 0.25)
    # ax.bar(X + 0.25, dists, color = 'g', width = 0.25)
    # ax.bar(X + 0.50, manip_list, color = 'r', width = 0.25)
    axs[0].grid(b=True, which='major', axis='y', linestyle=':')
    axs[1].grid(b=True, which='major', axis='y', linestyle=':')
    axs[2].grid(b=True, which='major', axis='y', linestyle=':')
    # axs.grid(b=True, which='minor', axis='y', linestyle=':')
    axs[0].set_ylim(0, max(tau_list)*1.1)
    axs[0].set_xlim(-0.5, len(tau_list)-0.5)
    axs[1].set_xlim(-0.5, len(tau_list) - 0.5)
    axs[2].set_xlim(-0.5, len(tau_list) - 0.5)
    axs[0].spines['top'].set_visible(False)
    axs[0].spines['right'].set_visible(False)
    axs[0].spines['left'].set_visible(False)
    axs[1].spines['top'].set_visible(False)
    axs[1].spines['right'].set_visible(False)
    axs[1].spines['left'].set_visible(False)
    axs[2].spines['top'].set_visible(False)
    axs[2].spines['right'].set_visible(False)
    axs[2].spines['left'].set_visible(False)

    plt.show()


def plot_force_transmission_ratios(ft1, ft2, ft3, ft4, ft5):
    import matplotlib.pyplot as plt
    # X = np.arange(3)
    fig, axs = plt.subplots(5)
    # ax = fig.add_axes([0,0,1,1])
    idxs = range(len(ft1))
    axs[0].bar(idxs, ft1, color='#1f77b4')
    axs[1].bar(idxs, ft2, color='#ff7f0e')
    axs[2].bar(idxs, ft3, color='#2ca02c')
    axs[3].bar(idxs, ft4, color='#d62728')
    axs[4].bar(idxs, ft5, color='#9467bd')# , color = 'b', width = 0.25)
    # ax.bar(X + 0.25, dists, color = 'g', width = 0.25)
    # ax.bar(X + 0.50, manip_list, color = 'r', width = 0.25)
    axs[0].grid(b=True, which='major', axis='y', linestyle=':')
    axs[1].grid(b=True, which='major', axis='y', linestyle=':')
    axs[2].grid(b=True, which='major', axis='y', linestyle=':')
    axs[3].grid(b=True, which='major', axis='y', linestyle=':')
    axs[4].grid(b=True, which='major', axis='y', linestyle=':')
    # axs.grid(b=True, which='minor', axis='y', linestyle=':')
    # axs[0].set_ylim(0, max(ft1)*1.1)
    axs[0].set_xlim(-0.5, len(ft1)-0.5)
    axs[1].set_xlim(-0.5, len(ft1) - 0.5)
    axs[2].set_xlim(-0.5, len(ft1) - 0.5)
    axs[3].set_xlim(-0.5, len(ft1) - 0.5)
    axs[4].set_xlim(-0.5, len(ft1) - 0.5)
    axs[0].spines['top'].set_visible(False)
    axs[0].spines['right'].set_visible(False)
    axs[0].spines['left'].set_visible(False)
    axs[1].spines['top'].set_visible(False)
    axs[1].spines['right'].set_visible(False)
    axs[1].spines['left'].set_visible(False)
    axs[2].spines['top'].set_visible(False)
    axs[2].spines['right'].set_visible(False)
    axs[2].spines['left'].set_visible(False)
    axs[3].spines['top'].set_visible(False)
    axs[3].spines['right'].set_visible(False)
    axs[3].spines['left'].set_visible(False)
    axs[4].spines['top'].set_visible(False)
    axs[4].spines['right'].set_visible(False)
    axs[4].spines['left'].set_visible(False)

    plt.show()


if __name__ == "__main__":
    candidates = pickle_utilities.load_pickle('/home/edoardo/MultiDoF-superbuild/external/modular/web/20201016-231348.pkl')

    tau_list, dist_list, manip_list, ratios_list = eval(candidates)

    print tau_list
    min_tau = min(tau_list)
    min_tau_index = tau_list.index(min_tau)
    min_torque_candidate = candidates[min_tau_index]
    print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    print min_tau, min_tau_index
    print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    print "Robot:", min_torque_candidate.robot
    print "X-Y 2nd base position:", min_torque_candidate.xy
    print "Angle offset:", min_torque_candidate.angle_offset
    print "Trasl. err. norm. 1:", min_torque_candidate.result['tr_norm_1']
    print "Rot. err. norm. 1:", min_torque_candidate.result['rot_norm_1']
    print "Trasl. err. norm. 2:", min_torque_candidate.result['tr_norm_2']
    print "Rot. err. norm. 2:", min_torque_candidate.result['rot_norm_2']
    print "q1:", min_torque_candidate.result['q_1']
    print "q1:", min_torque_candidate.result['q_2']
    print "tau1:", min_torque_candidate.result['tau_1']
    print "tau2:", min_torque_candidate.result['tau_2']

    # evaluate_robot(min_torque_candidate.robot, min_torque_candidate.xy, min_torque_candidate.angle_offset)

    print manip_list
    # max_man = max(manip_list)
    # max_man_index = tau_list.index(max_man)
    # max_man_candidate = candidates[max_man_index]
    # print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    # print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    # print max_man, max_man_index
    # print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
    # print "Robot:", min_torque_candidate.robot
    # print "X-Y 2nd base position:", max_man_candidate.xy
    # print "Angle offset:", max_man_candidate.angle_offset
    # print "Trasl. err. norm. 1:", max_man_candidate.err_trasl_1
    # print "Rot. err. norm. 1:", max_man_candidate.err_rot_1
    # print "Trasl. err. norm. 2:", max_man_candidate.err_trasl_2
    # print "Rot. err. norm. 2:", max_man_candidate.err_rot_2
    # print "q1:", max_man_candidate.q1
    # print "q1:", max_man_candidate.q2
    # print "tau1:", max_man_candidate.tau1
    # print "tau2:", max_man_candidate.tau2


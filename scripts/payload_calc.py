import pinocchio
import numpy as np
import scipy
import rospkg
import os

rospack = rospkg.RosPack()
pkgpath = rospack.get_path('ModularBot')
urdfpath = os.path.join(pkgpath, 'urdf', 'ModularBot.urdf')

urdf = open(urdfpath, 'r').read()

# Load the urdf model
model = pinocchio.buildModelFromUrdf(urdfpath)

# Create data required by the algorithms
data = model.createData()

# Get the frame index for the end-effector (hard-coded for now)
frame_idx = model.getFrameId('TCP_gripper_A')

lower_limits = model.lowerPositionLimit
upper_limits = model.upperPositionLimit

nq = model.nq

# for i in range(nq):
#     print(model.names[i+1], lower_limits[i], upper_limits[i])

## LinSpace Sampling
# n_q_samples = 10
# q_samples = np.empty((nq, n_q_samples))
# # q_list = []

# for idx, lims in enumerate(zip(lower_limits, upper_limits)):
#     q_idx_samples = np.linspace(lims[0], lims[1], n_q_samples)
#     q_samples[idx] = q_idx_samples
#     # q_list.append(q_idx_samples)
# print(q_samples)
# q_configurations = np.array(list(itertools.product(*q_samples)))
# n_samples = len(q_configurations)

## Random Sampling
n_samples = 100000
q_configurations = [pinocchio.randomConfiguration(model) for i in range(n_samples)]

c = np.array([0, 0, -1, 0, 0, 0])

lb = np.zeros((nq))
ub = np.array([0, 0, np.inf, 0, 0, 0])

worst_case_force = np.array([0, 0, np.inf, 0, 0, 0])
worst_case_q = np.zeros((nq))
worst_case_tau = np.zeros((nq))
worst_case_b = np.zeros((nq))
worst_case_A = np.zeros((nq, 6))

zeros = np.zeros((nq))
q = np.zeros((nq))
A = np.zeros((nq, 6))
b = np.zeros((nq))
tau_rated = 162.0
tau_max = np.ones((nq))*tau_rated
tau_min = - tau_max

# Compute the jacobian for all the samples
for idx, q in enumerate(q_configurations):
    # q = np.array(q)
    pinocchio.computeJointJacobians(model, data, q)
    # pinocchio.framesForwardKinematics(model, data, q)
    A = pinocchio.getFrameJacobian(model, data, frame_idx, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:6].T * 1  # 9.81
    b = tau_max - np.abs(pinocchio.nonLinearEffects(model, data, q, zeros))
    tau = pinocchio.nonLinearEffects(model, data, q, zeros)
    tau_available = []
    sign_J_z = np.sign(A[:,2])
    sign_tau = np.sign(tau)
    ## method 1
    # for idx, torque in enumerate(tau):
    #     if torque <= tau_rated and sign_tau[idx] in [1]:
    #         tau_tmp = tau_rated - torque
    #         # if sign_J_z[idx] != sign_tau[idx] and sign_J_z[idx] != 0:
    #         #     tau_tmp *= -1
    #         if sign_J_z[idx] != 1:
    #             A[idx, 2] *= -1
    #         tau_available.append(tau_tmp)
    #     elif torque >= -tau_rated and sign_tau[idx] in [-1]:
    #         tau_tmp = -tau_rated - torque
    #         # if sign_J_z[idx] != sign_tau[idx] and sign_J_z[idx] != 0:
    #         #     tau_tmp *= -1
    #         tau_tmp *= -1
    #         if sign_J_z[idx] != 1:
    #             A[idx, 2] *= -1
    #         tau_available.append(tau_tmp)
    #     elif sign_tau[idx] == 0:
    #         tau_available.append(tau_rated)
    #     else:
    #         tau_available.append(0.0)
    # b = np.array(tau_available)

    ## method 2
    b1 = tau_max - tau
    b2 = - tau_min + tau
    b = np.concatenate((b1, b2))
    A1 = A
    A2 = -A
    A = np.vstack((A1, A2))
    

    sol = scipy.optimize.linprog(c, A_ub=A, b_ub=b, bounds=list(zip(lb, ub)))

    # print (idx)
    
    if (sol.success):
        # print (sol.x / 9.81)
        if sol.x[2]/9.81 < worst_case_force[2]/9.81:
            worst_case_force = sol.x
            worst_case_q = q
            worst_case_tau = tau
            worst_case_b = b
            worst_case_A = A

print (('Payload: ', worst_case_force[2]/9.81))
print (('q: ', worst_case_q))


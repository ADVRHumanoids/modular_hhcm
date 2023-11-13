import pinocchio
import itertools
import numpy
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

n_q_samples = 2

q_samples = numpy.empty((nq, n_q_samples))
# q_list = []

# for i in range(nq):
#     print(model.names[i+1], lower_limits[i], upper_limits[i])

for idx, lims in enumerate(zip(lower_limits, upper_limits)):
    
    q_idx_samples = numpy.linspace(lims[0], lims[1], n_q_samples)
    q_samples[idx] = q_idx_samples
    # q_list.append(q_idx_samples)

print(q_samples)

q_configurations = list(itertools.product(*q_samples))
n_samples = len(q_configurations)

# other_qs = numpy.meshgrid(q_samples[0], q_samples[1], q_samples[2], q_samples[3], q_samples[4], q_samples[5])
# print(other_qs)

# print(n_samples)

# Compute the jacobian for all the samples
jac_samples = numpy.empty((nq * n_samples, 6))
tau_samples = numpy.empty((nq* n_samples))
# jac_samples = numpy.zeros(nq*n_samples*6).reshape((nq * n_samples, 6))
# tau_samples = numpy.zeros(nq*n_samples).reshape(nq* n_samples)
for idx, q in enumerate(q_configurations):
    q = numpy.array(q)
    pinocchio.computeJointJacobians(model, data, q)
    pinocchio.framesForwardKinematics(model, data, q)
    jac_samples[idx*nq:idx*nq+nq, :] = pinocchio.getFrameJacobian(model, data, frame_idx, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:6].T * -9.81
    tau_samples[idx*nq:idx*nq+nq] = numpy.ones((nq))*162 - pinocchio.nonLinearEffects(model, data, q, numpy.zeros(nq))

    # A = jac_samples[2, :, idx]*9.81
    # b = 162 - tau_samples[:, idx]
    # x = b/A
    # print(x)
    # print(numpy.linalg.norm(A*x - b))

# A_eq = -jac_samples[:, :, 0].T * 9.81
# for idx in range(1, n_samples):
#     A_eq = numpy.concatenate((A_eq, -jac_samples[:, :, idx].T * 9.81))
A_eq = jac_samples

# b_eq = numpy.ones((nq))*162 - tau_samples[:, 0]
# for idx in range(1, n_samples):
#     b_eq = numpy.concatenate((b_eq, numpy.ones((nq))*162 - tau_samples[:, idx]))
b_eq = tau_samples

c = numpy.array([0, 0, -1, 0, 0, 0])

lb = numpy.zeros((nq))
ub = numpy.array([0, 0, numpy.inf, 0, 0, 0])

sol = scipy.optimize.linprog(c, A_ub=A_eq, b_ub=b_eq, bounds=list(zip(lb, ub)))
# sol = numpy.linalg.solve(A_eq.T, b_eq.T)

print (sol.status)
print (sol.message)
print (sol.success)
print (sol.x)

print (sol.x / 9.81)
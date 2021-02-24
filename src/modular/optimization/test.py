from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from PyKDL import ChainDynParam, Vector, JntArray
import nlopt
import numpy as np


def transform_matrix(*matrices):
    transformed = []

    for matrix in matrices:
        transformed.append(np.array(matrix))

    return transformed


def joint_list_to_kdl(q):

    if q is None:
        return None

    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]

    q_kdl = JntArray(len(q))

    for i, q_i in enumerate(q):
        q_kdl[i] = q_i

    return q_kdl


def parallel_cf(q, gradient, kinematic, local_axis, parallel_to):

    assert len(parallel_to) == 3 and len(local_axis) == 3, 'axis must be 3d vector'

    # extract rotation matrix from pose:
    rot_matrix = np.array(kinematic.forward(q)[:3, :3])

    local_axis, parallel_to = transform_matrix(local_axis, parallel_to)

    # transform axis in global frame
    axis = rot_matrix.dot(local_axis)

    # dot product:
    dot_product = parallel_to.dot(axis)

    # rotational part of Jacobian
    j_rot = np.array(kinematic.jacobian(q)[3:6])

    s = np.zeros((3, 3))
    s[1, 2] = axis[0]
    s[2, 1] = - axis[0]
    s[0, 2] = - axis[1]
    s[2, 0] = axis[1]
    s[0, 1] = axis[2]
    s[1, 0] = - axis[2]

    j_axis = s.dot(j_rot)

    temp = (dot_product - 1.0) * parallel_to.dot(j_axis)

    if gradient.size > 0:
        for i in range(len(q)):
            gradient[i] = temp[i]

    return 0.5 * np.square(dot_product - 1.0)


def check(tolerance, solution, *constraints):

    for constraint in constraints:
        if abs(constraint(solution, np.array([]))) > abs(tolerance):
            constraint(solution, np.array([]))
            return False

    return True


def position_constraint(q, gradient, kinematic, global_axis, value, sign):

    assert len(global_axis) == 3, 'axis must be 3d vector'
    global_axis, = transform_matrix(global_axis)

    sign = 1.0 if sign is '+' else -1.0

    # translational part of Jacobian
    j_pos = np.array(kinematic.jacobian(q)[0:3])
    position = np.array(kinematic.forward(q)[0:3, 3])
    position = np.squeeze(np.asarray(position))

    temp = global_axis.dot(j_pos)

    if gradient.size > 0:
        for i in range(len(q)):
            gradient[i] = sign * temp[i]

    return sign * (position.dot(global_axis) - value)


def orientation_constraint(q, gradient, kinematic, local_axis, global_axis, value, sign):

    assert not abs(value) > 1.0, 'projection cannot be bigger than 1'
    assert len(global_axis) == 3 and len(local_axis) == 3, 'axis must be 3d vector'

    local_axis, global_axis = transform_matrix(local_axis, global_axis)

    sign = 1.0 if sign is '+' else -1.0

    # transform axis in global frame
    rot_matrix = np.array(kinematic.forward(q)[:3, :3])
    local_axis = rot_matrix.dot(local_axis)

    # dot product:
    dot_product = global_axis.dot(local_axis)

    # rotational part of Jacobian
    s = np.array([[0.0, -local_axis[2], local_axis[1]],
                  [local_axis[2], 0.0, -local_axis[0]],
                  [-local_axis[1], local_axis[0], 0.0]])

    j_rot = np.array(kinematic.jacobian(q)[3:6])
    j_axis = - s.dot(j_rot)

    temp = global_axis.dot(j_axis)

    if gradient.size > 0:
        for i in range(len(q)):
            gradient[i] = sign*temp[i]

    return sign * (dot_product - value)


def gravity_cost(q, gradient, dynamic):

    def get_gravity(q):

        q_kdl = joint_list_to_kdl(q)
        g_kdl = joint_list_to_kdl(q)
        dynamic.JntToGravity(q_kdl, g_kdl)

        return np.array([el for el in g_kdl])

    def get_g_gradient(q):

        increment = 0.001
        n = len(q)

        grad = np.empty((n, n))
        g0 = get_gravity(q)

        for i in range(n):
            delta = np.zeros(n)
            delta[i] = increment
            grad[:, i] = (get_gravity(q + delta) - g0) / increment

        return grad

    g = get_gravity(q)
    dg = get_g_gradient(q)

    dp = g.dot(g)
    grad = g.dot(dg)

    if gradient.size > 0:
        for i in range(len(gradient)):
            gradient[i] = grad[i]

    return 0.5 * dp


def solve(opt, n, ub, lb, check, max_time=0.2, max_efforts=50):

    best_x, best_val = None, None

    failures = 0
    successes = 0

    while successes < max_efforts and failures < max_efforts:

        #print 'effort: ', successes + failures

        rd = np.random.rand(n)
        seed = lb + np.diag(rd).dot(ub - lb)

        try:
            opt.set_maxtime(max_time)
            temp_x = opt.optimize(seed)
            temp_min = opt.last_optimum_value()

            ret = opt.last_optimize_result()

            if ret is nlopt.MAXTIME_REACHED:
                #print 'timeout! :('
                failures += 1
                continue

            if ret < 0:
                #print 'miserably failed'
                failures += 1
                continue

            if not check(temp_x):
                #print 'porcalanonna! :('
                failures += 1
                continue

        except Exception as e:
            #print 'failed with seed :( ', seed
            failures += 1
            continue

        print 'kinda solution found!'
        successes += 1

        if temp_min < best_val or best_val is None:
            best_val = temp_min
            best_x = temp_x
            break

    return best_x, best_val

def iacobelli_IK():
    # get_chain()
    robot = URDF.from_xml_file("/tmp/modular/urdf/ModularBot.urdf")

    print 'HERE WE START!'

    base = 'base_link'
    target = 'ee_A'
    max_position_error = 0.0005
    x_target, y_target, z_target = 0.3, 0.0, 0.0
    tree = kdl_tree_from_urdf_model(robot)
    chain = tree.getChain(base, target)

    # gravity vector
    g = Vector(0, 0, -9.81)

    kdl_kin = KDLKinematics(robot, base, target)
    kdl_dyn = ChainDynParam(chain, g)

    # qq = joint_list_to_kdl([0,1.5,1.5,0,0])
    # pp = joint_list_to_kdl([0,0,0,0,0])
    # kdl_dyn.JntToGravity(qq, pp)

    print 'number of joints:', tree.getNrOfJoints()

    # initialize the optimization problem: we need a global optimization with equality constraints
    # https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
    n = tree.getNrOfJoints()
    opt = nlopt.opt(nlopt.LD_SLSQP, n)

    # set the joints limit as upper and lower bounds:
    ub = np.array([2.0]*n)
    lb = np.array([-2.0]*n)

    opt.set_lower_bounds(lb)
    opt.set_upper_bounds(ub)

    # set the target function to minimize:
    opt.set_min_objective(lambda q, grad: gravity_cost(q, grad, kdl_dyn))

    # set the constraints:

    x_constraint = lambda q, grad: position_constraint(q, grad, kdl_kin, (1, 0, 0), x_target, '+')
    y_constraint = lambda q, grad: position_constraint(q, grad, kdl_kin, (0, 1, 0), y_target, '+')
    z_constraint = lambda q, grad: position_constraint(q, grad, kdl_kin, (0, 0, 1), z_target, '+')
    n_constraint = lambda q, grad: orientation_constraint(q, grad, kdl_kin, (0, 0, 1), (0, 0, -1), 1.0, '+')

    opt.add_equality_constraint(x_constraint, 1e-8)
    opt.add_equality_constraint(y_constraint, 1e-8)
    opt.add_equality_constraint(z_constraint, 1e-8)
    opt.add_equality_constraint(n_constraint, 1e-8)

    # set stop criteria:
    # opt.set_xtol_rel(1e-9)  # -> nlopt.XTOL_REACHED
    # opt.set_stopval(1e-9)  # -> nlopt.STOPVAL_REACHED
    opt.set_ftol_rel(1e-8)  # -> nlopt.FTOL_REACHED

    # in this solve the black magic... random seed and timeout to keep limited the computation time
    xf, min_found = solve(opt, n, ub, lb, lambda solution: check(max_position_error, solution, x_constraint, y_constraint, z_constraint))

    if min_found is not None:

        # do forward kinematics with the optimal solution:
        pose = kdl_kin.forward(xf)

        print 'LIFE IS BEAUTIFUL!'

        print "optimum at:", xf
        print "optimized pose:\n", pose
        print "minimum value:", min_found

        g_f = joint_list_to_kdl(xf)
        kdl_dyn.JntToGravity(joint_list_to_kdl(xf), g_f)
        print "joint torques:", g_f

        return min_found, xf, pose, n

    else:

        print 'LAMERDA: no solution found'

        return None, None, None, n
    
if __name__ == '__main__':

    iacobelli_IK()


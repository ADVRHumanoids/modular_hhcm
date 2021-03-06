from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from PyKDL import ChainDynParam, Vector, JntArray
import nlopt
import numpy as np
from collections import Sized, namedtuple
import fiacobelli_rbdl.rbdl as rbdl


def joint_list_to_kdl(q):

    if q is None:
        return None

    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]

    q_kdl = JntArray(len(q))

    for i, q_i in enumerate(q):
        q_kdl[i] = q_i

    return q_kdl


########################################################################################################################

class MyFunction(object):

    def __init__(self):
        pass

    def get_value(self, q):
        pass

    def get_gradient(self, q):
        pass


########################################################################################################################


class MyConstraint(MyFunction):
    GREATER = '>'
    SMALLER = '<'
    EQUAL = '='

    def __init__(self, function, relation, value, tolerance):
        MyFunction.__init__(self)

        assert relation in (MyConstraint.GREATER, MyConstraint.SMALLER, MyConstraint.EQUAL), 'invalid relation'

        self._function = function
        self._relation = relation
        self._sign = - 1.0 if relation is MyConstraint.GREATER else 1.0
        self._tolerance = tolerance
        self._value = value

    def get_value(self, q):
        return self._sign * (self._function.get_value(q) - self._value)

    def get_gradient(self, q):
        return self._sign * self._function.get_gradient(q)

    def check(self, q):
        value = self.get_value(q)

        if self._relation is MyConstraint.EQUAL:
            return abs(value) < self.get_tolerance()

        return value < self.get_tolerance()

    def get_tolerance(self):
        return self._tolerance

    def get_type(self):
        return self._relation


class Quadratic(MyFunction):

    def __init__(self, function, value):

        # min(0.5 * ||f - b||^2)

        MyFunction.__init__(self)
        self._function = function
        self._value = np.array(value)

    def get_value(self, q):

        error = self._function.get_value(q) - self._value
        norm = np.linalg.norm(error)
        return 0.5 * np.square(norm)

    def get_gradient(self, q):

        f = self._function.get_value(q)
        return self._function.get_gradient(q).dot(f)


########################################################################################################################


class Position(MyFunction):

    def __init__(self, axis, kinematic):

        MyFunction.__init__(self)

        assert len(axis) == 3, 'axis must be 3d vector'

        self._axis = np.array(axis)
        self._kinematic = kinematic

    def get_value(self, q):

        position = np.array(self._kinematic.forward(q)[0:3, 3])
        position = np.squeeze(np.asarray(position))

        return position.dot(self._axis)

    def get_gradient(self, q):

        j_pos = np.array(self._kinematic.jacobian(q)[0:3])
        return self._axis.dot(j_pos)


class Projection(MyFunction):

    def __init__(self, local_axis, global_axis, kinematic):
        MyFunction.__init__(self)

        assert len(local_axis) == 3 and len(global_axis) == 3, 'axis must be 3d vector'

        self._laxis = np.array(local_axis)
        self._gaxis = np.array(global_axis)
        self._kinematic = kinematic

    def get_value(self, q):

        # transform axis in global frame
        rot_matrix = np.array(self._kinematic.forward(q)[:3, :3])
        axis = rot_matrix.dot(self._laxis)

        return self._gaxis.dot(axis)

    def get_gradient(self, q):

        rot_matrix = np.array(self._kinematic.forward(q)[:3, :3])
        axis = rot_matrix.dot(self._laxis)

        s = np.array([[0.0, -axis[2], axis[1]],
                      [axis[2], 0.0, -axis[0]],
                      [-axis[1], axis[0], 0.0]])

        j_rot = np.array(self._kinematic.jacobian(q)[3:6])
        j_axis = - s.dot(j_rot)

        return self._gaxis.dot(j_axis)


class Gravity(MyFunction):

    def __init__(self, dynamic):

        MyFunction.__init__(self)
        self._dynamic = dynamic

    def get_value(self, q):

        q_kdl = joint_list_to_kdl(q)
        g_kdl = joint_list_to_kdl(q)

        self._dynamic.JntToGravity(q_kdl, g_kdl)
        return np.array([el for el in g_kdl])

    def get_gradient(self, q):

        increment, n = 0.001, len(q)
        initial = self.get_value(q)
        gradient = np.empty((n, n))

        for i in range(n):
            delta = np.zeros(n)
            delta[i] = increment
            gradient[:, i] = (self.get_value(q + delta) - initial) / increment

        return gradient


class Gravity2(MyFunction):

    def __init__(self, gravity_getter):

        MyFunction.__init__(self)
        self._getter = gravity_getter

    def get_value(self, q):

        return self._getter(q)

    def get_gradient(self, q):

        increment, n = 0.001, len(q)
        initial = self.get_value(q)
        gradient = np.empty((n, n))

        for i in range(n):
            delta = np.zeros(n)
            delta[i] = increment
            gradient[:, i] = (self.get_value(q + delta) - initial) / increment

        return gradient

class Manipulability(MyFunction):

    ROTATION = 'rotation'
    TRANSLATION = 'translation'

    def __init__(self, kinematic, motion_type):

        MyFunction.__init__(self)
        self._kinematic = kinematic
        self._type = motion_type

    def get_value(self, q):

        j = np.array(self._kinematic.jacobian(q)[3:6]) if self._type is Manipulability.ROTATION else np.array(self._kinematic.jacobian(q)[0:3])
        
        jjt = j.dot(j.transpose())
        m = np.linalg.det(jjt)

        return np.sqrt(m)

    def get_gradient(self, q):

        increment, n = 0.001, len(q)
        initial = self.get_value(q)
        gradient = np.empty((n, n))

        for i in range(n):
            delta = np.zeros(n)
            delta[i] = increment
            gradient[:, i] = (self.get_value(q + delta) - initial) / increment

        return gradient


class ForceTransmission(MyFunction):

    ROTATION = 'rotation'
    TRANSLATION = 'translation'

    def __init__(self, kinematic, axis, motion_type):

        MyFunction.__init__(self)
        self._kinematic = kinematic
        self._axis = np.array(axis)
        self._type = motion_type

    def get_value(self, q):

        j = np.array(self._kinematic.jacobian(q)[3:6]) if self._type is Manipulability.ROTATION else np.array(self._kinematic.jacobian(q)[0:3])
        jtu = j.transpose().dot(self._axis)

        return np.sqrt(jtu.dot(jtu))

    def get_gradient(self, q):

        increment, n = 0.001, len(q)
        initial = self.get_value(q)
        gradient = np.empty((n, n))

        for i in range(n):
            delta = np.zeros(n)
            delta[i] = increment
            gradient[:, i] = (self.get_value(q + delta) - initial) / increment

        return gradient


########################################################################################################################


class Bounds(object):

    def __init__(self, n, ub, lb):

        if not isinstance(ub, Sized):
            ub = [ub] * n
            lb = [lb] * n

        assert len(ub) == n and len(lb) == n, 'invalid bounds'

        self._ub = np.array(ub)
        self._lb = np.array(lb)

    def get_lb(self):
        return self._lb

    def get_ub(self):
        return self._ub


########################################################################################################################

class Problem(object):

    def __init__(self, dimension):

        self._optimizer = nlopt.opt(nlopt.LD_SLSQP, dimension)
        self._constraints = list()
        self._target = None

        self._optimizer.set_ftol_rel(1e-8)  # -> nlopt.FTOL_REACHED

    @staticmethod
    def _compute(q, gradient, function):

        if len(gradient) > 0:
            temp = function.get_gradient(q)
            for i in range(len(q)):
                gradient[i] = temp[i]

        return function.get_value(q)

    def set_bounds(self, bounds):

        self._optimizer.set_lower_bounds(bounds.get_lb())
        self._optimizer.set_upper_bounds(bounds.get_ub())

    def add_constraint(self, *constraints):

        for constraint in constraints:
            self._constraints.append(constraint)
            if constraint.get_type() is MyConstraint.EQUAL:
                self._optimizer.add_equality_constraint(lambda q, g, c=constraint: Problem._compute(q, g, c), 1e-8)
            else:
                self._optimizer.add_inequality_constraint(lambda q, g, c=constraint: Problem._compute(q, g, c), 1e-8)

    def set_target(self, target):

        self._optimizer.set_min_objective(lambda q, g, t=target: Problem._compute(q, g, t))

    def solve(self, initial_guess=None, max_efforts=300, max_effort_time=0.1, check_cb=None):

        # This function tries to solve the problem for `max_effort` times.
        # If solutions are found, the best one (i.e. the one with the least cost)
        # is returned as a tuple (q, f). In case no solution is found, the returned tuple is (None, None).

        output = namedtuple('output', ('q', 'cost'))
        self._optimizer.set_maxtime(max_effort_time)

        failures = 0
        check_cb = check_cb or (lambda t: True)
        best = output(None, None)

        lb = self._optimizer.get_lower_bounds()
        ub = self._optimizer.get_upper_bounds()

        accepted = {nlopt.SUCCESS, nlopt.STOPVAL_REACHED, nlopt.FTOL_REACHED, nlopt.XTOL_REACHED}

        while failures < max_efforts:

            if initial_guess is None:
                rd = np.random.rand(len(ub))
                initial_guess = lb + np.diag(rd).dot(ub - lb)

            try:
                q = self._optimizer.optimize(initial_guess)
                f = self._optimizer.last_optimum_value()

                retval = self._optimizer.last_optimize_result()

                if retval in accepted and all([c.check(q) for c in self._constraints]) and check_cb(q):
                    if best.cost > f or best.cost is None:
                        print 'solution found:\n', q
                        print 'optimum value:\n', f
                        best = output(q, f)

            except nlopt.RoundoffLimited as e:
                pass

            except RuntimeError:
                pass

            failures += 1
            initial_guess = None

        return best


########################################################################################################################

def test2(urdf_string, x_target, y_target, z_target, check_cb=None):

    # get_chain()

    robot = URDF.from_xml_string(urdf_string)
    # robot = URDF.from_xml_file("/tmp/modular/urdf/ModularBot.urdf")

    # print 'HERE WE START!'

    base = 'base_link'
    target = 'pen_A'
    max_position_error = 0.0005
    max_orientation_error = 0.0005
    # x_target, y_target, z_target = 0.3, 0.0, 0.0
    n_target = 1.0

    tree = kdl_tree_from_urdf_model(robot)
    chain = tree.getChain(base, target)

    # gravity vector
    gravity = Vector(0, 0, -9.81)

    kdl_kin = KDLKinematics(robot, base, target)
    kdl_dyn = ChainDynParam(chain, gravity)

    # print 'number of joints:', tree.getNrOfJoints()

    dof = tree.getNrOfJoints()

    x = Position((1, 0, 0), kdl_kin)
    y = Position((0, 1, 0), kdl_kin)
    z = Position((0, 0, 1), kdl_kin)
    n = Projection((0, 0, 1), (0, -1, 0), kdl_kin)
    # n = Projection((0, 0, 1), (0, 0, -1), kdl_kin)

    bounds = Bounds(dof, 2.0, -2.0)

    constraint_x = MyConstraint(x, MyConstraint.EQUAL, x_target, max_position_error)
    constraint_y = MyConstraint(y, MyConstraint.EQUAL, y_target, max_position_error)
    constraint_z = MyConstraint(z, MyConstraint.EQUAL, z_target, max_position_error)
    constraint_n = MyConstraint(n, MyConstraint.EQUAL, n_target, max_orientation_error)

    trg = Quadratic(Gravity(kdl_dyn), np.zeros((dof,)))

    problem = Problem(dof)

    problem.add_constraint(constraint_x, constraint_y, constraint_z, constraint_n)
    problem.set_bounds(bounds)
    problem.set_target(trg)

    pose_f = None
    q_opt, f_opt = problem.solve(check_cb=check_cb)

    if q_opt is not None:
        pose_f = kdl_kin.forward(q_opt)
        print 'here is the optimal pose:\n', pose_f

    return f_opt, q_opt, pose_f, dof


def test(urdf_string, x_target, y_target, z_target, check_cb=None, base_link=None, distal_link=None):

    def get_gravity(mdl, q):

        zeros, tau = np.zeros(len(q)), np.zeros(len(q))
        rbdl.InverseDynamics(mdl, q, zeros, zeros, tau)
        return tau

    # get_chain()

    robot = URDF.from_xml_string(urdf_string)
    
    # use rbdl for computing the inverse dynamic
    rbdlm = rbdl.loadModelFromString(urdf_string)

    # print 'HERE WE START!'

    base = base_link or 'base_link'
    target = distal_link or 'pen_A'
    max_position_error = 0.0005
    max_orientation_error = 0.0005
    # x_target, y_target, z_target = 0.3, 0.0, 0.0
    n_target = 1. #.996

    tree = kdl_tree_from_urdf_model(robot)
    chain = tree.getChain(base, target)

    # gravity vector
    gravity = Vector(0, 0, -9.81)

    kdl_kin = KDLKinematics(robot, base, target)
    #kdl_dyn = ChainDynParam(chain, gravity)

    # print 'number of joints:', tree.getNrOfJoints()

    dof = tree.getNrOfJoints()

    x = Position((1, 0, 0), kdl_kin)
    y = Position((0, 1, 0), kdl_kin)
    z = Position((0, 0, 1), kdl_kin)
    n = Projection((0, 0, 1), (0, -1, 0), kdl_kin)
    # n = Projection((0, 0, 1), (0, 0, -1), kdl_kin)

    bounds = Bounds(dof, 2., -2.)

    constraint_x = MyConstraint(x, MyConstraint.EQUAL, x_target, max_position_error)
    constraint_y = MyConstraint(y, MyConstraint.EQUAL, y_target, max_position_error)
    constraint_z = MyConstraint(z, MyConstraint.EQUAL, z_target, max_position_error)
    constraint_n = MyConstraint(n, MyConstraint.EQUAL, n_target, max_orientation_error)

    trg = Quadratic(Gravity2(lambda q, mdl=rbdlm: get_gravity(mdl, q)), np.zeros((dof,)))

    problem = Problem(dof)

    problem.add_constraint(constraint_x, constraint_y, constraint_z, constraint_n)
    problem.set_bounds(bounds)
    problem.set_target(trg)

    pose_f = None
    q_opt, f_opt = problem.solve(check_cb=check_cb)

    if q_opt is not None:
        pose_f = kdl_kin.forward(q_opt)
        print 'here is the optimal pose:\n', pose_f

        kman = Manipulability(kdl_kin, Manipulability.TRANSLATION).get_value(q_opt)
        fman = ForceTransmission(kdl_kin, [0., 1., 0.], ForceTransmission.TRANSLATION).get_value(q_opt)

        print("manipulability: {}".format(kman))
        print("force transmission: {}".format(fman))

    return f_opt, q_opt, pose_f, dof


from romity_collides import pyromiti_collision as pyrom

if __name__ == '__main__':
    # robot = URDF.from_xml_file("/home/tree/moveit_ws/src/pino_moveit/urdf/ModularBot.urdf")
    # base = 'base_link'
    # target = 'pen_A'
    # max_position_error = 0.0005
    # max_orientation_error = 0.0005
    # # x_target, y_target, z_target = 0.3, 0.0, 0.0
    # n_target = 1.0

    # tree = kdl_tree_from_urdf_model(robot)
    # chain = tree.getChain(base, target)

    # # gravity vector
    # gravity = Vector(0, 0, -9.81)

    # kdl_kin = KDLKinematics(robot, base, target)
    # kdl_dyn = ChainDynParam(chain, gravity)

    # dof = tree.getNrOfJoints()

    # # [ 0.12357965  0.99749001  1.24962237 -1.49476812  1.66825236]
    # # optimum value:
    # # 1036.334954
    # #q = [ 0.135, 0.978,  1.19,  -1.39,  1.66]
    # #q = [-0.19221036,  0.52081521,  0.88275572,  1.7252368,  -0.19054714]
    
    # q = [0.11,  -1.11 ,  2.09,  0.61, 0.39]
    
    # q_kdl = joint_list_to_kdl(q)
    # g_kdl = joint_list_to_kdl(q)
    # kdl_dyn.JntToGravity(q_kdl, g_kdl)
    # print(g_kdl)
    # print (kdl_kin.forward(q))

    with open("/home/tree/moveit_ws/src/pino_moveit/urdf/ModularBot.urdf", "r") as f:
        urdf = f.read()
        #rbdlm = rbdl.loadModelFromString(urdf)
        #zeros = np.zeros(5)
        #q = np.array(q)
        #tau = q*0.
        #rbdl.InverseDynamics(rbdlm, q, zeros, zeros, tau)
        #print(tau)
    
    with open("/home/tree/moveit_ws/src/pino_moveit/srdf/ModularBot.srdf", "r") as s:
        srdf = s.read()
    
    cm = pyrom.CollisionManager()
    cm.load(urdf, srdf)

    x = [3.92999991e-01, 3.95999993e-01, 2.72999997e-01, 2.71999999e-01]
    y = [-3.77999997e-01, -3.77999998e-01, -3.78999995e-01, -3.79999995e-01]
    z = [1.85999994e-01, 6.49999994e-02, 6.49999975e-02, 1.85999995e-01]

    for i in range(len(x)):
        base = "J1_A_stator"
        target = 'pen_A'
        
        f_opt, q_opt, pose_f, dof = test(urdf, x[i], y[i] + 0.1, z[i], check_cb=lambda q: not cm.collides(q), base_link=base, distal_link=target)

        robot = URDF.from_xml_string(urdf)
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain(base, target)

        kdl_kin = KDLKinematics(robot, base, target)
        
        kman = Manipulability(kdl_kin, Manipulability.TRANSLATION).get_value(q_opt)
        fman = ForceTransmission(kdl_kin, [0., 1., 0.], ForceTransmission.TRANSLATION).get_value(q_opt)

        print("manipulability: {}".format(kman))
        print("force transmission: {}".format(fman))
        print("################")
    
    

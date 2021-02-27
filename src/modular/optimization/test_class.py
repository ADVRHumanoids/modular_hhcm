from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from PyKDL import ChainDynParam, Vector, JntArray
import nlopt
import numpy as np
from collections import Sized


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


class Manipulability(MyFunction):

    ROTATION = 'rotation'
    TRANSLATION = 'rotation'

    def __init__(self, kinematic, axis, motion_type):

        MyFunction.__init__(self)
        self._kinematic = kinematic
        self._axis = np.array(axis)
        self._type = motion_type

    def get_value(self, q):

        j = np.array(self._kinematic.jacobian(q)[3:6]) if self._type is Manipulability.ROTATION else np.array(self._kinematic.jacobian(q)[0:3])

        j_axis = j.dot(self._axis)

        return np.sqrt(j_axis.dot(j_axis))

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

    def solve(self, initial_guess=None, max_efforts=300, max_effort_time=0.1):

        self._optimizer.set_maxtime(max_effort_time)

        failures = 0

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

                if retval in accepted and all([c.check(q) for c in self._constraints]):
                    print 'solution found:\n', q
                    print 'optimum value:\n', f
                    return q, f

            except nlopt.RoundoffLimited as e:
                pass

            except RuntimeError:
                pass

            failures += 1
            initial_guess = None

        print 'solution not found'


########################################################################################################################

def test(urdf_string, x_target, y_target, z_target):

    # get_chain()

    robot = URDF.from_xml_string(urdf_string)
    # robot = URDF.from_xml_file("/tmp/modular/urdf/ModularBot.urdf")

    # print 'HERE WE START!'

    base = 'base_link'
    target = 'ee_A'
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

    rst = problem.solve()

    if rst is not None:
        q_opt, f_opt  = rst
        pose_f = kdl_kin.forward(q_opt)
        print 'here is the optimal pose:\n', pose_f
        return f_opt, q_opt, pose_f, dof
    else:
        return None, None, None, dof

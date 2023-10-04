import pinocchio
import numpy as np
import scipy
import rospkg
import os

class ModelStats:
    def __init__(self, urdf_writer):
        self.urdf_writer = urdf_writer

        # ensure the urdf string has been already generated
        if getattr(self.urdf_writer, 'urdf_string', None) is None:
            self.urdf_writer.write_urdf()
        self.urdf = self.urdf_writer.urdf_string

        self.update_model()

    def update_model(self):
        # Load the urdf model
        self.model = pinocchio.buildModelFromUrdf(self.urdf)

        # Create data required by the algorithms
        self.data = self.model.createData()

        # Get the tip link name
        # HACK: we take the last chain in the list of chains. This should be changed to be the one selected by the user
        tip_link_name = self.urdf_writer.get_chain_tip_link(self.listofchains[-1])
        
        # Get the frame index for the end-effector
        self.frame_idx = self.model.getFrameId(tip_link_name)  # 'TCP_gripper_A'

        self.lower_limits = self.model.lowerPositionLimit
        self.upper_limits = self.model.upperPositionLimit

        self.nq = self.model.nq

    def compute_payload(self, n_samples=10000):
        self.update_model()
        
        ## Random Sampling
        samples = n_samples
        q_configurations = [pinocchio.randomConfiguration(self.model) for i in range(samples)]

        # Solve a linear programming problem to get the worst case force and therefore the worst case payload
        #    minimize:
        #        c @ x
        #    such that:
        #        A_ub @ x <= b_ub
        #        A_eq @ x == b_eq
        #        lb <= x <= ub

        c = np.array([0, 0, -1, 0, 0, 0])

        lb = np.zeros((self.nq))
        ub = np.array([0, 0, np.inf, 0, 0, 0])

        self.worst_case_force = np.array([0, 0, np.inf, 0, 0, 0])
        self.worst_case_q = np.zeros((self.nq))
        self.worst_case_tau = np.zeros((self.nq))
        self.worst_case_b = np.zeros((self.nq))
        self.worst_case_A = np.zeros((self.nq, 6))

        zeros = np.zeros((self.nq))
        q = np.zeros((self.nq))
        A = np.zeros((self.nq, 6))
        b = np.zeros((self.nq))
        tau_rated = 162.0
        tau_max = np.ones((self.nq))*tau_rated
        tau_min = - tau_max

        # Compute the jacobian and gravity torques for all the samples. Then, compute the worst case force -> overall payload
        for idx, q in enumerate(q_configurations):
            
            pinocchio.computeJointJacobians(self.model, self.data, q)
            # pinocchio.framesForwardKinematics(model, data, q)
            A = pinocchio.getFrameJacobian(self.model, self.data, self.frame_idx, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:6].T * 1  # 9.81
            tau = pinocchio.nonLinearEffects(self.model, self.data, q, zeros)

            b1 = tau_max - tau
            b2 = - tau_min + tau
            b = np.concatenate((b1, b2))
            A1 = A
            A2 = -A
            A = np.vstack((A1, A2))

            sol = scipy.optimize.linprog(c, A_ub=A, b_ub=b, bounds=list(zip(lb, ub)))

            if (sol.success):
                # print (sol.x / 9.81)
                if sol.x[2]/9.81 < self.worst_case_force[2]/9.81:
                    self.worst_case_force = sol.x
                    self.worst_case_q = q
                    self.worst_case_tau = tau
                    self.worst_case_b = b
                    self.worst_case_A = A

        self.payload = self.worst_case_force[2]/9.81

        return self.payload
    
    def get_payload(self):
        return self.payload

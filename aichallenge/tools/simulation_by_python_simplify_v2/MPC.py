import numpy as np
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

# Colors
PREDICTION = '#BA4A00'

##################
# MPC Controller #
##################


class MPC:
    def __init__(self, model, N, Q, R, QN, StateConstraints, InputConstraints,
                 ay_max):
        self.N = N
        self.Q = Q
        self.R = R
        self.QN = QN
        self.model = model
        self.nx = self.model.n_states
        self.nu = 2
        self.state_constraints = StateConstraints
        self.input_constraints = InputConstraints
        self.ay_max = ay_max
        self.current_prediction = None
        self.infeasibility_counter = 0
        self.current_control = np.zeros((self.nu*self.N))
        self.optimizer = osqp.OSQP()

    def _init_problem(self):
        umin = self.input_constraints['umin']
        umax = self.input_constraints['umax']
        xmin = self.state_constraints['xmin']
        xmax = self.state_constraints['xmax']

        A = np.zeros((self.nx * (self.N + 1), self.nx * (self.N + 1)))
        B = np.zeros((self.nx * (self.N + 1), self.nu * (self.N)))
        ur = np.zeros(self.nu*self.N)
        xr = np.zeros(self.nx*(self.N+1))
        uq = np.zeros(self.N * self.nx)
        xmin_dyn = np.kron(np.ones(self.N + 1), xmin)
        xmax_dyn = np.kron(np.ones(self.N + 1), xmax)
        umax_dyn = np.kron(np.ones(self.N), umax)
        kappa_pred = np.tan(np.array(self.current_control[3::] +
                                     self.current_control[-1:])) / self.model.length

        for n in range(self.N):
            current_waypoint = self.model.reference_path.get_waypoint(self.model.wp_id + n)
            next_waypoint = self.model.reference_path.get_waypoint(self.model.wp_id + n + 1)
            delta_s = next_waypoint - current_waypoint
            kappa_ref = current_waypoint.kappa
            v_ref = current_waypoint.v_ref

            f, A_lin, B_lin = self.model.linearize(v_ref, kappa_ref, delta_s)
            A[(n+1) * self.nx: (n+2)*self.nx, n * self.nx:(n+1)*self.nx] = A_lin
            B[(n+1) * self.nx: (n+2)*self.nx, n * self.nu:(n+1)*self.nu] = B_lin
            ur[n*self.nu:(n+1)*self.nu] = np.array([v_ref, kappa_ref])
            uq[n * self.nx:(n+1)*self.nx] = B_lin.dot(np.array([v_ref, kappa_ref])) - f

            vmax_dyn = np.sqrt(self.ay_max / (np.abs(kappa_pred[n]) + 1e-12))
            if vmax_dyn < umax_dyn[self.nu*n]:
                umax_dyn[self.nu*n] = vmax_dyn

        ub, lb, _ = self.model.reference_path.update_path_constraints(
                    self.model.wp_id+1, self.N, 2*self.model.safety_margin,
            self.model.safety_margin)
        xmin_dyn[0] = self.model.spatial_state.e_y
        xmax_dyn[0] = self.model.spatial_state.e_y
        xmin_dyn[self.nx::self.nx] = lb
        xmax_dyn[self.nx::self.nx] = ub
        xr[self.nx::self.nx] = (lb + ub) / 2

        Ax = sparse.kron(sparse.eye(self.N + 1),
                         -sparse.eye(self.nx)) + sparse.csc_matrix(A)
        Bu = sparse.csc_matrix(B)
        Aeq = sparse.hstack([Ax, Bu])
        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)
        A = sparse.vstack([Aeq, Aineq], format='csc')

        lineq = np.hstack([xmin_dyn, np.kron(np.ones(self.N), umin)])
        uineq = np.hstack([xmax_dyn, umax_dyn])
        x0 = np.array(self.model.spatial_state[:])
        leq = np.hstack([-x0, uq])
        ueq = leq
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])

        q = np.hstack([
            -np.tile(self.Q.diagonal(), self.N) * xr[:-self.nx],
            -self.QN.dot(xr[-self.nx:]),
            -np.tile(self.R.diagonal(), self.N) * ur
        ])
        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), self.Q),
            self.QN,
            sparse.kron(sparse.eye(self.N), self.R)
        ], format='csc')

        self.optimizer = osqp.OSQP()
        self.optimizer.setup(P=P, q=q, A=A, l=l, u=u, verbose=False)

    def get_control(self):
        nx = self.model.n_states
        nu = 2
        self.model.get_current_waypoint()
        self.model.spatial_state = self.model.t2s(reference_state=
            self.model.temporal_state, reference_waypoint=
            self.model.current_waypoint)
        self._init_problem()
        dec = self.optimizer.solve()

        try:
            control_signals = np.array(dec.x[-self.N*nu:])
            control_signals[1::2] = np.arctan(control_signals[1::2] * self.model.length)
            v = control_signals[0]
            delta = control_signals[1]
            self.current_control = control_signals
            x = np.reshape(dec.x[:(self.N+1)*nx], (self.N+1, nx))
            self.current_prediction = self.update_prediction(x)
            u = np.array([v, delta])
            self.infeasibility_counter = 0
        except:
            print('Infeasible problem. Previously predicted control signal used!')
            id = nu * (self.infeasibility_counter + 1)
            u = np.array(self.current_control[id:id+2])
            self.infeasibility_counter += 1

        if self.infeasibility_counter == (self.N - 1):
            print('No control signal computed!')
            exit(1)

        return u

    def update_prediction(self, spatial_state_prediction):
        x_pred, y_pred = [], []
        for n in range(2, self.N):
            associated_waypoint = self.model.reference_path.get_waypoint(self.model.wp_id+n)
            predicted_temporal_state = self.model.s2t(associated_waypoint,
                                            spatial_state_prediction[n, :])
            x_pred.append(predicted_temporal_state.x)
            y_pred.append(predicted_temporal_state.y)
        return x_pred, y_pred
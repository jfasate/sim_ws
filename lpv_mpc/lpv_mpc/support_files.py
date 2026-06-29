"""
LPV-MPC support functions for F1Tenth.
Adapted from Mark Misin's autonomous360 dynamic bicycle model MPC.

Vehicle model: Dynamic bicycle model with 6 states, 2 inputs, 4 outputs.
  States:  [x_dot, y_dot, psi, psi_dot, X, Y]  (body-frame velocities + global pose)
  Inputs:  [delta, a]  (steering angle, longitudinal acceleration)
  Outputs: [x_dot, psi, X, Y]  (tracked by MPC)

The MPC optimizes over du (change in inputs), using an augmented state that
includes the previous inputs. The system is re-linearized at each predicted
step in the horizon (LPV approach).
"""

import numpy as np


class SupportFilesF1Tenth:

    def __init__(self, params=None):
        """Initialize with F1Tenth vehicle parameters.

        Args:
            params: dict of overrides. Any key not provided uses the default.
        """
        p = params or {}

        # Vehicle constants (F1Tenth scale)
        g = p.get('g', 9.81)
        m = p.get('m', 3.47)               # vehicle mass [kg]
        Iz = p.get('Iz', 0.04712)           # yaw inertia [kg*m^2]
        Cf = p.get('Cf', 90.0)              # front cornering stiffness [N/rad]
        Cr = p.get('Cr', 110.0)             # rear cornering stiffness [N/rad]
        lf = p.get('lf', 0.15875)           # CG to front axle [m]
        lr = p.get('lr', 0.17145)           # CG to rear axle [m]
        Ts = p.get('Ts', 0.02)              # sample time [s]
        mju = p.get('mju', 0.015)           # rolling resistance coefficient

        # MPC tuning
        outputs = 4   # [x_dot, psi, X, Y]
        inputs = 2    # [delta, a]
        hz = p.get('hz', 10)  # prediction horizon

        # Cost matrices (diagonal)
        Q = np.diag(p.get('Q_diag', [10.0, 500.0, 100.0, 100.0]))
        S = np.diag(p.get('S_diag', [10.0, 500.0, 100.0, 100.0]))
        R = np.diag(p.get('R_diag', [50.0, 5.0]))

        self.constants = {
            'g': g, 'm': m, 'Iz': Iz, 'Cf': Cf, 'Cr': Cr,
            'lf': lf, 'lr': lr, 'Ts': Ts, 'mju': mju,
            'Q': Q, 'S': S, 'R': R,
            'outputs': outputs, 'inputs': inputs, 'hz': hz,
        }

        # Precompute all matrices that do NOT depend on the vehicle state or
        # inputs. In the LPV-MPC only the per-step linearization (A_aug, B_aug)
        # and the resulting prediction matrices (Cdb, Adc) change each tick;
        # the cost blocks (Qdb, Tdb, Rdb), the constraint-selection matrix
        # (C_asterisk_global), the rate-limit bounds and the identity blocks
        # are constant for a fixed horizon. Building them once removes a large
        # chunk of per-tick Python allocation and block-assignment work.
        self._precompute_constant_matrices()

    def _precompute_constant_matrices(self):
        """Build the state-independent QP matrices a single time.

        The output matrix C (and hence the augmented C_aug) is constant, so
        the cost-weight blocks CQC = C_aug.T Q C_aug etc. are constant too.
        """
        Q = self.constants['Q']
        S = self.constants['S']
        R = self.constants['R']
        hz = self.constants['hz']
        inputs = self.constants['inputs']
        n_out = self.constants['outputs']

        n_states = 6
        n_aug = n_states + inputs        # 8
        n_constr = 4

        # Constant output selection C and its augmented form C_aug = [C, 0].
        C = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ], dtype=float)
        C_aug = np.hstack([C, np.zeros((n_out, inputs))])

        CQC = C_aug.T @ Q @ C_aug
        CSC = C_aug.T @ S @ C_aug
        QC = Q @ C_aug
        SC = S @ C_aug

        # Block-diagonal cost matrices (terminal step uses S instead of Q).
        Qdb = np.zeros((n_aug * hz, n_aug * hz))
        Tdb = np.zeros((n_out * hz, n_aug * hz))
        Rdb = np.zeros((inputs * hz, inputs * hz))
        for i in range(hz):
            r0, c0 = n_aug * i, n_aug * i
            Qdb[r0:r0 + n_aug, c0:c0 + n_aug] = CSC if i == hz - 1 else CQC
            r0t = n_out * i
            Tdb[r0t:r0t + n_out, c0:c0 + n_aug] = SC if i == hz - 1 else QC
            r0r = inputs * i
            Rdb[r0r:r0r + inputs, r0r:r0r + inputs] = R

        # Constraint selection: picks [x_dot, y_dot, delta, a] from x_aug.
        C_asterisk = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1],
        ], dtype=float)
        C_asterisk_global = np.zeros((n_constr * hz, n_aug * hz))
        for i in range(hz):
            C_asterisk_global[n_constr * i:n_constr * i + n_constr,
                              n_aug * i:n_aug * i + n_aug] = C_asterisk

        # Input-rate limits (constant per sample): [d_delta, d_a].
        d_delta_max = np.pi / 90     # ~0.035 rad/step
        d_a_max = 0.5                # m/s^2 per step
        ub_global = np.empty(inputs * hz)
        ub_global[0::2] = d_delta_max
        ub_global[1::2] = d_a_max
        lb_global = ub_global.copy()
        ublb_global = np.concatenate((ub_global, lb_global))

        I_global = np.eye(inputs * hz)
        I_mega_global = np.vstack((I_global, -I_global))

        # Stash for use each tick.
        self._n_aug = n_aug
        self._n_constr = n_constr
        self._Qdb = Qdb
        self._Tdb = Tdb
        self._Rdb = Rdb
        self._C_asterisk = C_asterisk
        self._C_asterisk_global = C_asterisk_global
        self._ublb_global = ublb_global
        self._I_mega_global = I_mega_global

    def state_space(self, states, delta, a):
        """Linearize the dynamic bicycle model around the current operating point.

        Returns discrete-time (Ad, Bd, Cd, Dd) via the bilinear (Tustin)
        transform — i.e. trapezoidal integration of the continuous model.
        """
        g = self.constants['g']
        m = self.constants['m']
        Iz = self.constants['Iz']
        Cf = self.constants['Cf']
        Cr = self.constants['Cr']
        lf = self.constants['lf']
        lr = self.constants['lr']
        Ts = self.constants['Ts']
        mju = self.constants['mju']

        x_dot = states[0]
        y_dot = states[1]
        psi = states[2]

        # Protect against low longitudinal velocity.
        # The continuous A matrix has ~1/x_dot terms that blow up as
        # x_dot -> 0, independent of the discretization scheme. The Tustin
        # transform below is A-stable (so it no longer imposes a stability
        # floor the way forward Euler did), but this clamp is still needed
        # to keep the continuous model itself well-conditioned at low speed.
        x_dot = max(x_dot, 1.5)

        # Continuous-time A matrix entries
        A11 = -mju * g / x_dot
        A12 = Cf * np.sin(delta) / (m * x_dot)
        A14 = Cf * lf * np.sin(delta) / (m * x_dot) + y_dot
        A22 = -(Cr + Cf * np.cos(delta)) / (m * x_dot)
        A24 = -(Cf * lf * np.cos(delta) - Cr * lr) / (m * x_dot) - x_dot
        A34 = 1.0
        A42 = -(Cf * lf * np.cos(delta) - lr * Cr) / (Iz * x_dot)
        A44 = -(Cf * lf**2 * np.cos(delta) + lr**2 * Cr) / (Iz * x_dot)
        A51 = np.cos(psi)
        A52 = -np.sin(psi)
        A61 = np.sin(psi)
        A62 = np.cos(psi)

        # Continuous-time B matrix entries
        B11 = -1.0 / m * np.sin(delta) * Cf
        B12 = 1.0
        B21 = 1.0 / m * np.cos(delta) * Cf
        B41 = 1.0 / Iz * np.cos(delta) * Cf * lf

        A = np.array([
            [A11, A12, 0, A14, 0, 0],
            [0,   A22, 0, A24, 0, 0],
            [0,   0,   0, A34, 0, 0],
            [0,   A42, 0, A44, 0, 0],
            [A51, A52, 0, 0,   0, 0],
            [A61, A62, 0, 0,   0, 0],
        ])
        B = np.array([
            [B11, B12],
            [B21, 0],
            [0,   0],
            [B41, 0],
            [0,   0],
            [0,   0],
        ])
        C = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])
        D = np.zeros((4, 2))

        # Bilinear (Tustin) discretization via trapezoidal integration:
        #   Ad = (I - Ts/2 A)^-1 (I + Ts/2 A)
        #   Bd = (I - Ts/2 A)^-1 (Ts B)
        # Solve against the same (I - Ts/2 A) factor rather than forming an
        # explicit inverse (more numerically stable).
        I6 = np.eye(6)
        half = 0.5 * Ts
        M_lhs = I6 - half * A
        Ad = np.linalg.solve(M_lhs, I6 + half * A)
        Bd = np.linalg.solve(M_lhs, Ts * B)
        Cd = C
        Dd = D

        return Ad, Bd, Cd, Dd

    def augmented_matrices(self, Ad, Bd, Cd, Dd):
        """Build augmented system that includes previous inputs in the state.

        Augmented state: x_aug = [x_dot, y_dot, psi, psi_dot, X, Y, delta, a]
        """
        n_states = Ad.shape[0]  # 6
        n_inputs = Bd.shape[1]  # 2

        A_aug = np.block([
            [Ad, Bd],
            [np.zeros((n_inputs, n_states)), np.eye(n_inputs)],
        ])
        B_aug = np.vstack([Bd, np.eye(n_inputs)])
        C_aug = np.hstack([Cd, np.zeros((Cd.shape[0], n_inputs))])
        D_aug = Dd

        return A_aug, B_aug, C_aug, D_aug

    def mpc_simplification(self, Ad, Bd, Cd, Dd, hz, x_aug_t, du):
        """Build the QP matrices for the LPV-MPC.

        The LPV approach re-linearizes the system at each predicted step in the
        horizon, using the previous du solution for warm-starting.

        Returns:
            Hdb:  Hessian of QP (inputs*hz x inputs*hz)
            Fdbt: Linear term matrix
            Cdb:  Prediction matrix for du -> x_aug
            Adc:  Prediction matrix for x_aug_0 -> x_aug
            G:    Inequality constraint matrix
            ht:   Inequality constraint vector
        """
        A_aug, B_aug, C_aug, D_aug = self.augmented_matrices(Ad, Bd, Cd, Dd)

        Cf = self.constants['Cf']
        g = self.constants['g']
        m = self.constants['m']
        mju = self.constants['mju']
        lf = self.constants['lf']
        inputs = self.constants['inputs']

        # Constant matrices precomputed once in __init__.
        n_aug = self._n_aug          # 8
        n_constr = self._n_constr    # 4
        Qdb = self._Qdb
        Tdb = self._Tdb
        Rdb = self._Rdb
        C_asterisk_global = self._C_asterisk_global

        # ======================== LPV prediction ========================
        # Per-tick state-dependent work: store the step linearizations, the
        # homogeneous propagation chain (Adc), and the per-step state-constraint
        # bounds. The cost / selection / rate-limit matrices are constant and
        # already built in __init__.
        Adc = np.zeros((n_aug * hz, n_aug))
        A_aug_collection = np.zeros((hz, n_aug, n_aug))
        B_aug_collection = np.zeros((hz, n_aug, inputs))
        y_asterisk_max_global = np.zeros(n_constr * hz)
        y_asterisk_min_global = np.zeros(n_constr * hz)

        A_product = A_aug.copy()
        states_predicted_aug = x_aug_t.copy()

        for i in range(hz):
            # LPV: store current step linearization and predicted propagation
            Adc[n_aug * i:n_aug * i + n_aug, :] = A_product
            A_aug_collection[i] = A_aug
            B_aug_collection[i] = B_aug

            # ==================== State constraints ====================
            x_dot_pred = max(states_predicted_aug[0][0], 1.5)

            x_dot_max = 12.0
            y_dot_max = min(0.17 * x_dot_pred, 2.0)
            delta_max = 0.4189   # ~24 deg
            Fyf = Cf * (states_predicted_aug[6][0]
                        - states_predicted_aug[1][0] / x_dot_pred
                        - lf * states_predicted_aug[3][0] / x_dot_pred)
            a_bias = (Fyf * np.sin(states_predicted_aug[6][0]) + mju * m * g) / m \
                - states_predicted_aug[3][0] * states_predicted_aug[1][0]
            a_max = 3.0 + a_bias

            x_dot_min = 0.5
            y_dot_min = max(-0.17 * x_dot_pred, -2.0)
            delta_min = -0.4189
            a_min = -3.0 + a_bias

            j0 = n_constr * i
            y_asterisk_max_global[j0:j0 + n_constr] = (x_dot_max, y_dot_max, delta_max, a_max)
            y_asterisk_min_global[j0:j0 + n_constr] = (x_dot_min, y_dot_min, delta_min, a_min)

            # ==================== LPV: predict next step ====================
            if i < hz - 1:
                du1 = du[inputs * (i + 1)][0]
                du2 = du[inputs * (i + 1) + inputs - 1][0]
                states_predicted_aug = (A_aug @ states_predicted_aug
                                        + B_aug @ np.array([[du1], [du2]]))
                states_pred = states_predicted_aug[0:6, 0]
                delta_pred = states_predicted_aug[6][0]
                a_pred = states_predicted_aug[7][0]
                Ad_i, Bd_i, Cd_i, Dd_i = self.state_space(states_pred, delta_pred, a_pred)
                A_aug, B_aug, C_aug, D_aug = self.augmented_matrices(Ad_i, Bd_i, Cd_i, Dd_i)
                A_product = A_aug @ A_product

        # ======================== Prediction matrix Cdb ========================
        # Block recursion (same math, far fewer Python-level matmuls than the
        # original O(hz^2) double loop):
        #   Cdb[i, i] = B_i
        #   Cdb[i, j] = A_i @ Cdb[i-1, j]   for j < i
        # so the whole filled portion of each row is one matmul against the
        # previous row instead of an inner loop over j.
        Cdb = np.zeros((n_aug * hz, inputs * hz))
        for i in range(hz):
            ri = n_aug * i
            ci = inputs * i
            Cdb[ri:ri + n_aug, ci:ci + inputs] = B_aug_collection[i]
            if i > 0:
                width = inputs * i
                Cdb[ri:ri + n_aug, 0:width] = (
                    A_aug_collection[i] @ Cdb[ri - n_aug:ri, 0:width])

        # ======================== Constraint assembly ========================
        Cdb_constraints = C_asterisk_global @ Cdb
        Cdb_constraints_global = np.vstack((Cdb_constraints, -Cdb_constraints))

        Adc_constraints = C_asterisk_global @ Adc
        Adc_constraints_x0 = (Adc_constraints @ x_aug_t).flatten()
        y_max_diff = y_asterisk_max_global - Adc_constraints_x0
        y_min_diff = -y_asterisk_min_global + Adc_constraints_x0
        y_diff_global = np.concatenate((y_max_diff, y_min_diff))

        G = np.vstack((self._I_mega_global, Cdb_constraints_global))
        ht = np.concatenate((self._ublb_global, y_diff_global))

        # ======================== QP cost ========================
        QdbCdb = Qdb @ Cdb
        Hdb = Cdb.T @ QdbCdb + Rdb
        temp = Adc.T @ QdbCdb
        temp2 = -Tdb @ Cdb
        Fdbt = np.vstack((temp, temp2))

        return Hdb, Fdbt, Cdb, Adc, G, ht

    def open_loop_new_states(self, states, delta, a):
        """Simulate one sample period forward using sub-stepping.

        Used only if you want to simulate the plant (not needed when the
        f1tenth sim provides state feedback, but useful for debugging).
        """
        g = self.constants['g']
        m = self.constants['m']
        Iz = self.constants['Iz']
        Cf = self.constants['Cf']
        Cr = self.constants['Cr']
        lf = self.constants['lf']
        lr = self.constants['lr']
        Ts = self.constants['Ts']
        mju = self.constants['mju']

        x_dot = states[0]
        y_dot = states[1]
        psi = states[2]
        psi_dot = states[3]
        X = states[4]
        Y = states[5]

        sub_loop = 30
        for _ in range(sub_loop):
            x_dot = max(x_dot, 1.5)
            Fyf = Cf * (delta - y_dot / x_dot - lf * psi_dot / x_dot)
            Fyr = Cr * (-y_dot / x_dot + lr * psi_dot / x_dot)

            x_dot_dot = a + (-Fyf * np.sin(delta) - mju * m * g) / m + psi_dot * y_dot
            y_dot_dot = (Fyf * np.cos(delta) + Fyr) / m - psi_dot * x_dot
            psi_dot_dot = (Fyf * lf * np.cos(delta) - Fyr * lr) / Iz
            X_dot = x_dot * np.cos(psi) - y_dot * np.sin(psi)
            Y_dot = x_dot * np.sin(psi) + y_dot * np.cos(psi)

            dt = Ts / sub_loop
            x_dot += x_dot_dot * dt
            y_dot += y_dot_dot * dt
            psi += psi_dot * dt
            psi_dot += psi_dot_dot * dt
            X += X_dot * dt
            Y += Y_dot * dt

        new_states = np.array([x_dot, y_dot, psi, psi_dot, X, Y])
        return new_states, x_dot_dot, y_dot_dot, psi_dot_dot

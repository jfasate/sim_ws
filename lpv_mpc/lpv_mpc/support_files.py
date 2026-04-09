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

    def state_space(self, states, delta, a):
        """Linearize the dynamic bicycle model around the current operating point.

        Returns discrete-time (Ad, Bd, Cd, Dd) via forward Euler.
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
        # At low speed the terms ~1/x_dot make the A matrix stiff;
        # forward-Euler discretization becomes unstable when
        # |A44| * Ts > 2, i.e. x_dot < (Cf*lf^2+Cr*lr^2)/(Iz * 2/Ts).
        # For F1Tenth params this critical speed is ~1.2 m/s.
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

        # Forward Euler discretization
        Ad = np.eye(6) + Ts * A
        Bd = Ts * B
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

        Q = self.constants['Q']
        S = self.constants['S']
        R = self.constants['R']
        Cf = self.constants['Cf']
        g = self.constants['g']
        m = self.constants['m']
        mju = self.constants['mju']
        lf = self.constants['lf']
        inputs = self.constants['inputs']

        n_aug = A_aug.shape[0]  # 8
        n_out = C_aug.shape[0]  # 4

        # ======================== Constraints ========================
        # Input rate limits (per sample)
        d_delta_max = np.pi / 90     # ~0.035 rad/step  (~1.75 rad/s)
        d_a_max = 0.5                # m/s^2 per step

        ub_global = np.zeros(inputs * hz)
        lb_global = np.zeros(inputs * hz)
        for i in range(inputs * hz):
            if i % 2 == 0:  # steering rate
                ub_global[i] = d_delta_max
                lb_global[i] = d_delta_max
            else:            # accel rate
                ub_global[i] = d_a_max
                lb_global[i] = d_a_max

        ublb_global = np.concatenate((ub_global, lb_global))

        I_global = np.eye(inputs * hz)
        I_mega_global = np.vstack((I_global, -I_global))

        # State/input constraint selection matrix
        # Constrains: [x_dot, y_dot, delta, a] from augmented state
        C_asterisk = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1],
        ], dtype=float)

        n_constr = C_asterisk.shape[0]  # 4
        C_asterisk_global = np.zeros((n_constr * hz, n_aug * hz))

        y_asterisk_max_global = []
        y_asterisk_min_global = []

        # ======================== Cost matrices ========================
        CQC = C_aug.T @ Q @ C_aug
        CSC = C_aug.T @ S @ C_aug
        QC = Q @ C_aug
        SC = S @ C_aug

        Qdb = np.zeros((n_aug * hz, n_aug * hz))
        Tdb = np.zeros((n_out * hz, n_aug * hz))
        Rdb = np.zeros((inputs * hz, inputs * hz))
        Cdb = np.zeros((n_aug * hz, inputs * hz))
        Adc = np.zeros((n_aug * hz, n_aug))

        # ======================== LPV prediction ========================
        A_product = A_aug.copy()
        states_predicted_aug = x_aug_t.copy()
        A_aug_collection = np.zeros((hz, n_aug, n_aug))
        B_aug_collection = np.zeros((hz, n_aug, inputs))

        for i in range(hz):
            # Cost matrices
            if i == hz - 1:
                r0, c0 = n_aug * i, n_aug * i
                Qdb[r0:r0 + n_aug, c0:c0 + n_aug] = CSC
                r0t, c0t = n_out * i, n_aug * i
                Tdb[r0t:r0t + n_out, c0t:c0t + n_aug] = SC
            else:
                r0, c0 = n_aug * i, n_aug * i
                Qdb[r0:r0 + n_aug, c0:c0 + n_aug] = CQC
                r0t, c0t = n_out * i, n_aug * i
                Tdb[r0t:r0t + n_out, c0t:c0t + n_aug] = QC

            r0r, c0r = inputs * i, inputs * i
            Rdb[r0r:r0r + inputs, c0r:c0r + inputs] = R

            # LPV: store current matrices and predicted propagation
            Adc[n_aug * i:n_aug * i + n_aug, :] = A_product
            A_aug_collection[i] = A_aug
            B_aug_collection[i] = B_aug

            # ==================== State constraints ====================
            x_dot_pred = max(states_predicted_aug[0][0], 1.5)

            x_dot_max = 8.0
            y_dot_max = min(0.17 * x_dot_pred, 2.0)
            delta_max = 0.4189   # ~24 deg
            Fyf = Cf * (states_predicted_aug[6][0]
                        - states_predicted_aug[1][0] / x_dot_pred
                        - lf * states_predicted_aug[3][0] / x_dot_pred)
            a_max = 3.0 + (Fyf * np.sin(states_predicted_aug[6][0]) + mju * m * g) / m \
                    - states_predicted_aug[3][0] * states_predicted_aug[1][0]

            x_dot_min = 0.5
            y_dot_min = max(-0.17 * x_dot_pred, -2.0)
            delta_min = -0.4189
            a_min = -3.0 + (Fyf * np.sin(states_predicted_aug[6][0]) + mju * m * g) / m \
                    - states_predicted_aug[3][0] * states_predicted_aug[1][0]

            y_asterisk_max = np.array([x_dot_max, y_dot_max, delta_max, a_max])
            y_asterisk_min = np.array([x_dot_min, y_dot_min, delta_min, a_min])
            y_asterisk_max_global = np.concatenate((y_asterisk_max_global, y_asterisk_max))
            y_asterisk_min_global = np.concatenate((y_asterisk_min_global, y_asterisk_min))

            C_asterisk_global[n_constr * i:n_constr * i + n_constr,
                              n_aug * i:n_aug * i + n_aug] = C_asterisk

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

        # Build Cdb (lower-triangular block matrix of A^(i-j) * B products)
        for i in range(hz):
            for j in range(i + 1):
                AB_product = np.eye(n_aug)
                for ii in range(i, j - 1, -1):
                    if ii > j:
                        AB_product = AB_product @ A_aug_collection[ii]
                    else:
                        AB_product = AB_product @ B_aug_collection[ii]
                Cdb[n_aug * i:n_aug * i + n_aug,
                    inputs * j:inputs * j + inputs] = AB_product

        # ======================== Constraint assembly ========================
        Cdb_constraints = C_asterisk_global @ Cdb
        Cdb_constraints_global = np.vstack((Cdb_constraints, -Cdb_constraints))

        Adc_constraints = C_asterisk_global @ Adc
        Adc_constraints_x0 = (Adc_constraints @ x_aug_t).flatten()
        y_max_diff = y_asterisk_max_global - Adc_constraints_x0
        y_min_diff = -y_asterisk_min_global + Adc_constraints_x0
        y_diff_global = np.concatenate((y_max_diff, y_min_diff))

        G = np.vstack((I_mega_global, Cdb_constraints_global))
        ht = np.concatenate((ublb_global, y_diff_global))

        # ======================== QP cost ========================
        Hdb = Cdb.T @ Qdb @ Cdb + Rdb
        temp = Adc.T @ Qdb @ Cdb
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

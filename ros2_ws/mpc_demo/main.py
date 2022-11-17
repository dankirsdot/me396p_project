import numpy as np
import matplotlib.pyplot as plt

# CasADi is an open-source tool for nonlinear optimization and algorithmic differentiation
from casadi import *

from plotter import *
from reference_trajectory import *

# System parameters
dt = 0.2  # time step [s]
L = 2.0  # weelbase length [m]
l_r = 1.2  # length from the rear wheel to the center of mass [m]
model_parameters = L, l_r

# Control limits
omega_max = 1.22  # [rad/s]
omega_min = -1.22  # [rad/s]

# Obstacles
obs_x = [10.0, 5.0]  # [m]
obs_y = [10.0, 25.0]  # [m]
obs_radius = [1.0, 1.0]  # [m]
n_O = len(obs_x)  # number of obstacles

###########################################
# Set up NLP problem
###########################################

N = 8  # length of prediction horizon

# State vector
x = SX.sym('x')
y = SX.sym('y')
theta = SX.sym('theta')
delta = SX.sym('delta')
X = vertcat(x, y, theta, delta)
n_X = X.shape[0]

# Control vector
v = SX.sym('v')
omega = SX.sym('omega')
U = vertcat(v, omega)
n_U = U.shape[0]

# System ODE
beta = atan2(l_r*tan(delta), L)

dx = v*cos(theta + beta)
dy = v*sin(theta + beta)
dtheta = (v*cos(beta)*tan(delta))/L
ddelta = omega

ODE = vertcat(dx, dy, dtheta, ddelta)

dX = Function('ODE', [X, U], [ODE], ['X', 'U'], ['ODE'])

# Decision variables (controls)
h_U = SX.sym('h_U', n_U, N)

# A vector that represents the states over the optimization problem.
h_X = SX.sym('h_X', n_X, N+1)

# Parameters (which include the initial state and the reference along the
# predicted trajectory (reference states and reference controls))
P = SX.sym('P', n_X + N*(n_X+n_U))

obj = 0  # objective function
g = []  # constraints vector

Q = np.diag([1.0, 1.0, 0.1, 0.0])  # weighing matrices (state)
R = np.diag([0.01, 0.01])  # weighing matrices (control activity)
R_hat = np.diag([1.0, 1.0])  # weighing matrices (control changing rate)

X0 = h_X[:, 0]  # initial state
g.append(X0 - P[0:n_X])  # initial condition constraints
# print(g)

for i in range(N):
    Xi = h_X[:, i]  # current state
    Ui = h_U[:, i]  # current control

    state_obj = Xi - P[n_X + i*(n_X + n_U): 2*n_X + i*(n_X + n_U)]
    # print(state_obj)
    obj += mtimes(mtimes(state_obj.T, Q), state_obj)

    # control_obj = Ui - P[2*n_X + i*(n_X + n_U): n_U + 2*n_X + i*(n_X + n_U)]
    # print(control_obj)
    # obj += mtimes(mtimes(control_obj.T, R), control_obj)

    if i < N-1:
        # penalize the control activity
        obj += mtimes(mtimes(Ui.T, R), Ui)

        # penalize control changing rate
        Ui_next = h_U[:, i+1]  # next control
        control_obj = Ui - Ui_next
        # print(control_obj)
        obj += mtimes(mtimes(control_obj.T, R_hat), control_obj)

    Xi_next = h_X[:, i+1]  # next state
    dXi = dX(Xi, Ui)  # state variables update
    Xi_next_euler = Xi + dXi*dt  # using the Euler approach
    g.append(Xi_next - Xi_next_euler)  # compute constraints

# Add constraints for collision avoidance
obstacles = obs_x, obs_y, obs_radius, n_O
for i in range(n_O):
    for j in range(N+1):
        g.append(sqrt((h_X[0, j]-obs_x[i])**2+(h_X[1, j]-obs_y[i])**2))

# Form the NLP
opt_vars = [reshape(h_X, n_X*(N+1), 1), reshape(h_U, n_U*N, 1)]
opt_vars = vertcat(*opt_vars)

g = vertcat(*g)

nlp = {'f': obj, 'x': opt_vars, 'g': g, 'p': P}

# Solver options
opts = {}
opts["ipopt"] = {"max_iter": 2000, "print_level": 0,
                 "acceptable_tol": 1e-8, "acceptable_obj_change_tol": 1e-6}
opts["print_time"] = 0

# Allocate a solver
solver = nlpsol("solver", "ipopt", nlp, opts)

# Set up constraints bounds
# lb - lower bound
# ub - upper bound
lbg = np.zeros(g.shape[0])  # equality constraints
ubg = np.zeros(g.shape[0])  # equality constraints

# collision avoidance constraints
obs_margin = 2.0  # [m]
for i in range(n_O):
    for j in range(N+1):
        lbg[n_X*(N+1)*(i+1):n_X*(N+1)*(i+2)] = obs_radius[i] + obs_margin
        ubg[n_X*(N+1)*(i+1):n_X*(N+1)*(i+2)] = inf

lbx = np.zeros(opt_vars.shape[0])
ubx = np.zeros(opt_vars.shape[0])

# x constraints
lbx[0:n_X*(N+1):n_X] = -inf
ubx[0:n_X*(N+1):n_X] = inf

# y constraints
lbx[1:n_X*(N+1):n_X] = -inf
ubx[1:n_X*(N+1):n_X] = inf

# theta constraints
lbx[2:n_X*(N+1):n_X] = -inf
ubx[2:n_X*(N+1):n_X] = inf

# delta constraints
lbx[3:n_X*(N+1):n_X] = -inf
ubx[3:n_X*(N+1):n_X] = inf

# v constraints
lbx[n_X*(N+1):n_X*(N+1)+n_U*N:n_U] = -inf  # -inf
ubx[n_X*(N+1):n_X*(N+1)+n_U*N:n_U] = inf  # inf

# omega constraints
lbx[n_X*(N+1)+1:n_X*(N+1)+n_U*N:n_U] = omega_min
ubx[n_X*(N+1)+1:n_X*(N+1)+n_U*N:n_U] = omega_max


###########################################
# Start the simulation loop
###########################################

first_frame = True

t0 = 0  # initial time
X0 = np.array([0, 0, 0, 0], dtype=np.float64)  # initial state

X_list = []  # [X0]
t_list = []  # [t0]

h_X_num = np.zeros((N+1)*n_X)  # numerical state vectors
h_U_num = np.zeros((N, n_U))  # numerical control inputs

MPC_iter = 0
U_list = []  # list for the controls U
X_sol_list = []  # list for the solution trajectories

# Get reference trajectory
dl = 1.0  # trajectory step
cx, cy, ctheta, _ = get_switch_back_course(dl)
for i in range(N-1):
    cx.append(cx[-1])
    cy.append(cy[-1])
    ctheta.append(ctheta[-1])

# Start MPC
for j in range(len(cx) - N + 1):
    cur_time = MPC_iter*dt  # get the current time

    p = np.zeros(P.shape[0])
    p[:n_X] = X0  # set initial position

    for i in range(N):
        pred_time = cur_time + i*dt  # predicted time instant

        x_ref = cx[i + j]
        y_ref = cy[i + j]
        theta_ref = ctheta[i + j]
        delta_ref = 0
        X_ref = np.array(
            [x_ref, y_ref, theta_ref, delta_ref], dtype=np.float64)

        v_ref = 0.0
        omega_ref = 0.0
        U_ref = np.array([v_ref, omega_ref], dtype=np.float64)

        # State parameters
        p[n_X + i*(n_X + n_U): 2*n_X + i*(n_X + n_U)] = X_ref
        # Control parameters
        p[2*n_X + i*(n_X + n_U): n_U + 2*n_X + i*(n_X + n_U)] = U_ref

    # Initial value of the optimization variables
    opt_vars_ini = [reshape(h_X_num, n_X*(N+1), 1), reshape(h_U_num, n_U*N, 1)]
    opt_vars_ini = vertcat(*opt_vars_ini)

    # Solve the NLP
    sol = solver(x0=opt_vars_ini, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

    # Get the solution
    opt_vars_sol = sol['x']

    # Get controls only from the solution
    h_U_num = opt_vars_sol[n_X*(N+1):].full()
    U0 = h_U_num[:n_U].ravel()  # take the current control
    U_list.append(U0)

    # Plot the solution step
    plot_step(j, N, X0, U0, t0, h_X_num, cx, cy, model_parameters, obstacles)
    if first_frame == True:
        input()
        first_frame = False

    # Append time
    t_list.append(t0)

    # Apply the control and shift the solution
    dX0 = dX(X0, U0).full()
    X0 = X0 + (dX0*dt).ravel()
    X_list.append(X0)

    # Update time
    t0 = t0 + dt

    # Get states only from the solution
    h_X_num = opt_vars_sol[:n_X*(N+1)].full()
    X_sol_list.append(h_X_num)

    # Shift state solution
    h_X_num = np.roll(h_X_num, -n_X)
    h_X_num[-n_X:] = h_X_num[-2*n_X:-n_X]

    # Shift control solution
    h_U_num = np.roll(h_U_num, -n_U)
    h_U_num[-n_U:] = h_U_num[-2*n_U:-n_U]

    MPC_iter += 1

X = numpy.asarray(X_list)
x = X[:, 0]
y = X[:, 1]
theta = X[:, 2]
delta = X[:, 3]

U = numpy.asarray(U_list)
v = U[:, 0]
omega = U[:, 1]

t = numpy.asarray(t_list)

plt.close("all")
plt.subplots()
plt.plot(cx[:-N+1], cy[:-N+1], "-g", label="Reference path")
plt.plot(x, y, "-r", label="Actual path")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()

# Plot obstacles
for i in range(n_O):
    circle_i = plt.Circle((obs_x[i], obs_y[i]), obs_radius[i], color='black')
    plt.gcf().gca().add_artist(circle_i)

plt.subplots()
plt.plot(t, v, "-r", label="Linear speed")
plt.grid(True)
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")

plt.show()

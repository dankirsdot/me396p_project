import numpy as np
import matplotlib.pyplot as plt


def plot_car(x, y, theta, delta, model_parameters, cabcolor="-r", truckcolor="-k"):
    # Extract model parameters
    L, l_r = model_parameters

    # Vehicle parameters
    car_width = 2.0  # [m]
    wheel_length = 0.3  # [m]
    wheel_width = 0.2  # [m]
    tread = 0.5  # [m]

    backtowheel = 0.8  # [m]
    facetowheel = 0.8  # [m]

    outline = np.array([[- backtowheel - l_r, (L - l_r) + facetowheel, (L - l_r) + facetowheel, - backtowheel - l_r, -backtowheel - l_r],
                        [car_width / 2, car_width / 2, - car_width / 2, -car_width / 2, car_width / 2]])

    fr_wheel = np.array([[wheel_length, -wheel_length, -wheel_length, wheel_length, wheel_length],
                         [-wheel_width - tread, -wheel_width - tread, wheel_width - tread, wheel_width - tread, -wheel_width - tread]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1
    rr_wheel[0, :] -= l_r
    rl_wheel[0, :] -= l_r

    Rot1 = np.array([[np.cos(theta), np.sin(theta)],
                     [-np.sin(theta), np.cos(theta)]])
    Rot2 = np.array([[np.cos(delta), np.sin(delta)],
                     [-np.sin(delta), np.cos(delta)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += (L - l_r)
    fl_wheel[0, :] += (L - l_r)

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)


def plot_step(step, N, X, U, t, h_X_num, cx, cy, model_parameters, obstacles):
    # Extrack state variables
    x, y, theta, delta = X[0], X[1], X[2], X[3]
    v, omega = U[0], U[1]

    # Extract horizon solution
    n_X = X.shape[0]
    x_sol = h_X_num[0::n_X]
    y_sol = h_X_num[1::n_X]

    # Extract obstacles
    obs_x, obs_y, obs_radius, n_O = obstacles

    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])

    # Plot obstacles
    for i in range(n_O):
        circle_i = plt.Circle((obs_x[i], obs_y[i]),
                              obs_radius[i], color='black')
        plt.gcf().gca().add_artist(circle_i)

    plt.plot(cx, cy, "-g", label="Reference path")
    plt.plot(x_sol, y_sol, "xk", label="MPC solution")
    plt.plot(cx[step:step+N], cy[step:step+N], "xr", label="Reference horizon")
    plot_car(x, y, theta, delta, model_parameters,
             cabcolor="-r", truckcolor="-k")

    plt.axis("equal")
    plt.grid(True)
    plt.title("Time[s]:" + str(round(t, 2))
              + ", speed[m/s]:" + str(round(v, 2)))
    plt.legend()
    plt.pause(0.001)

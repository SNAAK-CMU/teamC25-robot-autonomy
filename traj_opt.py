import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def softmin(x, y, k=10):
    """ Smooth minimum function. """
    return -1/k * np.log(np.exp(-k*x) + np.exp(-k*y))

def dynamics(params, x, u):
    """ System dynamics function. """
    A = 0.05
    B = 100

    u_smooth = softmin(u[0], 0, 50)  # Smooth min function
    xdot = A * x[0] * B * -u_smooth**2  # Compute xdot
    return np.array([xdot])

def hermite_simpson(params, x1, x2, u, dt):
    """ Hermite-Simpson implicit integration residual. """
    x1dot = dynamics(params, x1, u)
    x2dot = dynamics(params, x2, u)
    x_k12 = 0.5 * (x1 + x2) + (dt / 8) * (x1dot - x2dot)
    
    return x1 + (dt / 6) * (x1dot + 4 * dynamics(params, x_k12, u) + x2dot) - x2

def create_idx(nx, nu, N):
    nz = (N - 1) * nu + N * nx
    x = [list(range((i - 1) * (nx + nu), (i - 1) * (nx + nu) + nx)) for i in range(1, N + 1)]
    u = [list(range((i - 1) * (nx + nu) + nx, (i - 1) * (nx + nu) + nx + nu)) for i in range(1, N)]
    c = [list(range((i - 1) * nx, (i - 1) * nx + nx)) for i in range(1, N)]
    nc = (N - 1) * nx

    return {"nx": nx, "nu": nu, "N": N, "nz": nz, "nc": nc, "x": x, "u": u, "c": c}

def cost(Z, params):
    idx, N, xg = params["idx"], params["N"], params["xg"]
    Q, R, Qf = params["Q"], params["R"], params["Qf"]
    
    X = np.array([Z[idx["x"][i]] for i in range(N)]).T
    U = np.array([Z[idx["u"][i]] for i in range(N-1)]).T
    
    J = 0.5 * np.sum(np.diag((X[:, :-1] - xg).T @ Q @ (X[:, :-1] - xg)))
    J += 0.5 * np.sum(np.diag((U - 0.25).T @ R @ (U - 0.25)))
    J += 0.5 * np.dot((X[:, -1] - xg).T, Qf @ (X[:, -1] - xg))
    
    return J

def dynamics_constraints(Z, params):
    idx, N, dt = params["idx"], params["N"], params["dt"]
    c = np.zeros(idx["nc"])
    
    for i in range(N - 1):
        xi = Z[idx["x"][i]]
        ui = Z[idx["u"][i]]
        xip1 = Z[idx["x"][i + 1]]
        
        c[idx["c"][i]] = hermite_simpson(params, xi, xip1, ui, dt)
    
    return c

def equality_constraint(Z, params):
    idx, xic, xg = params["idx"], params["xic"], params["xg"]
    eq_con = [
        Z[idx["x"][0]] - xic,
        Z[idx["x"][-1]] - xg,
        Z[idx["u"][0]] - 0.25,
        Z[idx["u"][-1]] - 0.25,
        dynamics_constraints(Z, params)
    ]
    #print(equality_constraint)
    return np.concatenate(eq_con)

def inequality_constraint(Z, params):
    idx, N = params["idx"], params["N"]
    c = np.zeros(N - 2)
    
    for i in range(N - 2):
        ui = Z[idx["u"][i + 1]][0]
        uim1 = Z[idx["u"][i]][0]
        c[i] = ui - uim1
    
    return c

def solve_bead_pour(verbose=True):
    nx, nu = 1, 1
    dt, tf = 0.05, 10.0 # Generate trajectory at 20 Hz, use controller at 100 Hz to track
    t_vec = np.arange(0, tf + dt, dt)
    N = len(t_vec)

    Q, R, Qf = np.array([[1]]), np.array([[1]]), np.array([[10]])

    idx = create_idx(nx, nu, N)
    xic, xg = np.array([200]), np.array([60])

    params = {"Q": Q, "R": R, "Qf": Qf, "xic": xic, "xg": xg, "dt": dt, "N": N, "idx": idx}

    x_l, x_u = np.zeros(idx["nz"]), np.full(idx["nz"], np.inf)
    for i in range(N - 1):
        x_l[idx["u"][i]] = -0.785
        x_u[idx["u"][i]] = 0.25

    c_l, c_u = np.full(N - 2, -0.01), np.full(N - 2, 0.01)

    state_init = np.linspace(xic, xg, N)
    control_init = np.concatenate([
        np.linspace(0.25, -0.785, (N - 1) // 2),
        np.linspace(-0.785, 0.25, (N - 1) // 2)
    ])

    z0 = np.ones(idx["nz"])
    for i in range(N - 1):
        z0[idx["x"][i]] = state_init[i]
        z0[idx["u"][i]] = control_init[i]
    z0[idx["x"][-1]] = state_init[-1]

    result = minimize(cost, z0, 
                    args=(params,), 
                    method="SLSQP",
                    constraints=[
                        {"type": "eq", "fun": equality_constraint, "args": (params,)},
                        {"type": "ineq", "fun": lambda Z, params: inequality_constraint(Z, params) - c_l, "args": (params,)},
                        {"type": "ineq", "fun": lambda Z, params: c_u - inequality_constraint(Z, params), "args": (params,)}
                    ],
                    bounds=[(l, u) for l, u in zip(x_l, x_u)],
                    options={"maxiter": 50, "ftol": 1e-5, "disp": verbose}) #1e-6

    print(result)

    Z = result.x
    X = [Z[idx["x"][i]] for i in range(N)]
    U = [Z[idx["u"][i]] for i in range(N - 1)]

    return X, U, t_vec, params

X, U, t_vec, _ = solve_bead_pour(verbose=True)

Xm = np.array(X)
Um = np.array(U)

plt.figure()
plt.plot(t_vec, Xm, label="State")
plt.xlabel("Time (s)")
plt.ylabel("Weight (g)")
plt.title("State Trajectory")
plt.legend()
plt.show()

plt.figure()
plt.plot(t_vec[:-1], Um, label="Control")
plt.xlabel("Time (s)")
plt.ylabel("Pitch (rad)")
plt.title("Controls")
plt.legend()
plt.show()

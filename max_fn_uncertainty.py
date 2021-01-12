import numpy as np
import scipy.io
import matplotlib.pyplot as plt


def max_fn(*args):
    return np.max(args, axis=0)


def if_fn(current, thr):
    curr_ineq = current < thr*np.ones(current.shape)
    return curr_ineq


def compute_uncertainty(f, *args, n_samples=10000, **kwargs):
    """Computes uncertainty of function f under normal distributions characterised by argument tuples (m1, v1)"""

    samples = []

    for arg in args:
        samples.append(arg[0] + np.sqrt(arg[1]) * np.random.randn(n_samples))

    y = f(*samples, **kwargs)
    return np.var(y)


# The current setup runs a Monte Carlo simulation for the uncertainty propagated by the if statement

means = np.linspace(-10, 10, 100)
variances = np.linspace(0.001, 3, 100)
print(variances)
thresholds = [1] # np.logspace(-3, 0.5, 100)

results = []

for m1 in means:
    for var1 in variances:
        for th_var in thresholds:
            uncertainty = compute_uncertainty(if_fn, (0, th_var), (m1, var1), n_samples=10000)
            results.append((m1, var1, th_var, uncertainty))
    print(m1)

np.savez("if_uncertainty.npz", results)
scipy.io.savemat("if_uncertainty.mat", mdict={"results":results})

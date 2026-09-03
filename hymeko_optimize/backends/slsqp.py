"""SciPy SLSQP as a replaceable backend for a HyMeKo optimisation projection.

Read this file looking for the word "resistor". It is not here, and neither is "net", "bulb",
"voltage" or "divider" -- the only nouns are `x`, `f`, `h` and `g`. That is the whole point of the
exercise: the semantics live in `resistor_divider.hymeko`, the typed projection turns them into a
constrained problem, and a backend is handed the problem with the meaning stripped off. This file
could not reproduce the design if it wanted to, because nothing ever tells it what the design is.

It speaks the line protocol in `backend.rs`:

    <- setup <n> <equalities> <inequalities> <tolerance> <start...> <lower...> <upper...>
    -> eval <x...>
    <- <f> <df...> <h...> <dh row-major...> <g...> <dg row-major...>
    -> done <0|1> <iterations> <x...>

Numbers, whitespace separated, one message per line. There are no names in it.
"""

import sys

import numpy as np
from scipy.optimize import minimize


class Oracle:
    """The parent process, asked one point at a time.

    SLSQP evaluates the objective, its gradient and every constraint separately at the same point,
    so a naive driver would pay four round-trips per iteration. One cached point removes all but the
    first -- and the cache is keyed on exact equality rather than a tolerance, because a point that
    is merely close is a different point and answering it from the cache would silently smooth the
    very gradients the parent computed exactly.
    """

    def __init__(self, n, equalities, inequalities):
        self.n = n
        self.equalities = equalities
        self.inequalities = inequalities
        self.at = None
        self.cached = None
        self.calls = 0

    def __call__(self, x):
        key = tuple(x)
        if self.at == key:
            return self.cached
        sys.stdout.write("eval " + " ".join(repr(float(v)) for v in x) + "\n")
        sys.stdout.flush()
        line = sys.stdin.readline()
        if not line:
            raise SystemExit("the parent closed the channel")
        values = [float(w) for w in line.split()]
        take = iter(range(len(values)))

        def chunk(count):
            return np.array([values[next(take)] for _ in range(count)])

        f = chunk(1)[0]
        df = chunk(self.n)
        h = chunk(self.equalities)
        dh = chunk(self.equalities * self.n).reshape(self.equalities, self.n)
        g = chunk(self.inequalities)
        dg = chunk(self.inequalities * self.n).reshape(self.inequalities, self.n)

        self.at = key
        self.cached = (f, df, h, dh, g, dg)
        self.calls += 1
        return self.cached


def main():
    words = sys.stdin.readline().split()
    if not words or words[0] != "setup":
        raise SystemExit(f"expected a setup line, got {words[:1]}")
    n = int(words[1])
    equalities = int(words[2])
    inequalities = int(words[3])
    tolerance = float(words[4])
    rest = [float(w) for w in words[5:]]
    start = np.array(rest[0:n])
    lower = np.array(rest[n : 2 * n])
    upper = np.array(rest[2 * n : 3 * n])

    oracle = Oracle(n, equalities, inequalities)

    constraints = []
    if equalities:
        constraints.append(
            {
                "type": "eq",
                "fun": lambda x: oracle(x)[2],
                "jac": lambda x: oracle(x)[3],
            }
        )
    if inequalities:
        constraints.append(
            {
                "type": "ineq",
                "fun": lambda x: oracle(x)[4],
                "jac": lambda x: oracle(x)[5],
            }
        )

    result = minimize(
        lambda x: (oracle(x)[0], oracle(x)[1]),
        start,
        jac=True,
        method="SLSQP",
        bounds=list(zip(lower, upper)),
        constraints=constraints,
        options={"ftol": tolerance, "maxiter": 500},
    )

    sys.stdout.write(
        "done %d %d %s\n"
        % (
            1 if result.success else 0,
            int(result.nit),
            " ".join(repr(float(v)) for v in result.x),
        )
    )
    sys.stdout.flush()


if __name__ == "__main__":
    main()

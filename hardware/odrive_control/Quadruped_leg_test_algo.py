import numpy as np
from scipy.optimize import fsolve

# Function to compute S
def compute_S(A, B, K, H):
    numerator = A**2 + B**2 - K**2 - H**2
    denominator = 2 * A * B
    return numerator / denominator

# System of equations in terms of M only (using substitution)
def equations(x, A, B, K, S):
    # x is tan(M/2)
    sin_M = 2 * x / (1 + x**2)
    cos_M = (1 - x**2) / (1 + x**2)

    lhs = A * sin_M * np.sqrt(1 - S**2) + cos_M * (A * S - B)
    rhs = K

    return lhs - rhs

# Solve for M using fsolve
def solve_M(A, B, K, H):
    S = compute_S(A, B, K, H)

    if not (-1 <= S <= 1):
        raise ValueError("Invalid inputs: sin(N - M) = S must be in [-1, 1]. Got S =", S)

    # Initial guess for x = tan(M/2)
    x0 = np.tan(np.pi / 6)  # initial guess ~30 degrees

    # Solve
    sol = fsolve(equations, x0, args=(A, B, K, S))[0]

    # Recover M
    M = 2 * np.arctan(sol)

    # Compute N
    N = M + np.arcsin(S)

    # Ensure both are in (0, pi/2)
    if 0 < M < np.pi/2 and 0 < N < np.pi/2:
        return N, M
    else:
        raise ValueError("No valid solution in the interval (0, π/2). Try different inputs.")

# Main execution
if __name__ == "__main__":
    import math

    # Input values
    A = float(input("Enter A: "))
    B = float(input("Enter B: "))
    K = float(input("Enter K: "))
    H = float(input("Enter H: "))

    try:
        N, M = solve_M(A, B, K, H)
        print(f"Solution found:")
        print(f"N = {N:.6f} radians ({math.degrees(N):.2f} degrees)")
        print(f"M = {M:.6f} radians ({math.degrees(M):.2f} degrees)")
    except ValueError as e:
        print("Error:", e)

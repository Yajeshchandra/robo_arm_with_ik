import sympy as sp

# Define symbolic joint variables (q1 and q2 are joint angles)
q1, q2 = sp.symbols('q1 q2')

# Define link lengths (l1 and l2 are link lengths)
l1, l2 = sp.symbols('l1 l2')

# Forward kinematics (position of the end-effector in 2D space)
x = l1 * sp.sin(q1) + l2 * sp.cos(q2 - q1)
y = l1 * sp.cos(q1) + l2 * sp.sin(q2 - q1)

# Define the end-effector position vector
end_effector_pos = sp.Matrix([x, y])

# Compute the Jacobian matrix
jacobian = end_effector_pos.jacobian([q1, q2])

# Compute the inverse of the Jacobian matrix
jacobian_inv = jacobian.inv()

# Display the symbolic Jacobian matrix
sp.pprint(jacobian)
print()
sp.pprint(jacobian_inv)
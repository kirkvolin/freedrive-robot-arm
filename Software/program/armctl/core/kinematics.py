import math, numpy as np
from typing import List, Tuple, Optional
from ..config.arm_config import ArmConfig, DEFAULT_CONFIG

def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])

def forward_kinematics(angles_deg, config=DEFAULT_CONFIG):
    positions = [(0.0,0.0,0.0)]; transforms = []; T = np.eye(4)
    for i, dh in enumerate(config.dh_params):
        if i >= len(angles_deg): break
        theta = math.radians(angles_deg[i]) + dh["offset"]
        T = T @ dh_matrix(theta, dh["d"], dh["a"], dh["alpha"])
        transforms.append(T.copy()); positions.append((T[0,3],T[1,3],T[2,3]))
    return positions, transforms

def get_end_effector(angles_deg, config=DEFAULT_CONFIG):
    positions, _ = forward_kinematics(angles_deg, config)
    return np.array(positions[-1])

def compute_jacobian(angles_deg, delta=0.1, config=DEFAULT_CONFIG):
    n = min(5, len(angles_deg)); ee = get_end_effector(angles_deg, config)
    J = np.zeros((3, n))
    for i in range(n):
        ap = list(angles_deg[:n]); ap[i] += delta
        J[:, i] = (get_end_effector(ap, config) - ee) / delta
    return J

def inverse_kinematics(target, current_angles, config=DEFAULT_CONFIG, max_iter=50, tolerance=1.0, damping=5.0):
    angles = list(current_angles[:5])
    limits = [(j.min_angle, j.max_angle) for j in config.joints[:5]]
    for _ in range(max_iter):
        ee = get_end_effector(angles, config); error = target - ee
        if np.linalg.norm(error) < tolerance:
            return angles + [current_angles[5] if len(current_angles) > 5 else 50]
        J = compute_jacobian(angles, config=config)
        JJT = J @ J.T + (damping**2) * np.eye(3)
        d_theta = J.T @ np.linalg.solve(JJT, error)
        step_norm = np.max(np.abs(d_theta))
        if step_norm > 5.0: d_theta *= 5.0 / step_norm
        for i in range(5):
            angles[i] = max(limits[i][0], min(limits[i][1], angles[i] + d_theta[i]))
    return angles + [current_angles[5] if len(current_angles) > 5 else 50]

"""Independent finite-difference audit of chassis_task.c five-bar coordinates.

Run: python docs/jacobian_coordinate_check.py
Angles in this script are radians; the firmware position input is degrees.
No motor commands are sent. Uses numpy only.
"""
import numpy as np

L1 = L4 = L5 = 0.15
L2 = L3 = 0.27


def forward(q):
    p1, p2 = np.pi + q[0], np.pi - q[1]
    b = np.array([-L5 / 2 + L1 * np.cos(p1), L1 * np.sin(p1)])
    d = np.array([L5 / 2 - L4 * np.cos(p2), L4 * np.sin(p2)])
    a, bb = 2 * L2 * (d - b)
    c = L2**2 + np.sum((d - b)**2) - L3**2
    disc = a*a + bb*bb - c*c
    if disc <= 0:
        raise ValueError('Invalid or singular closure')
    p = 2 * np.arctan2(bb + np.sqrt(disc), a + c)
    e = b + L2 * np.array([np.cos(p), np.sin(p)])
    return np.array([np.linalg.norm(e), np.arctan2(e[0], e[1])]), b, d, e


def inverse_jacobian(q):
    (length, angle), b, d, e = forward(q)
    p1, p2 = np.pi + q[0], np.pi - q[1]
    u, v = e - b, e - d
    radial = np.array([np.sin(angle), np.cos(angle)])
    angular = length * np.array([np.cos(angle), -np.sin(angle)])
    den1 = L1 * np.dot(u, [-np.sin(p1), np.cos(p1)])
    den2 = L4 * np.dot(v, [np.sin(p2), np.cos(p2)])
    # Rows are derivatives of phi1, phi2, NOT encoder angles.
    return np.array([[u @ radial / den1, u @ angular / den1],
                     [v @ radial / den2, v @ angular / den2]])


def corrected_firmware_n(q):
    (length, angle), b, d, _ = forward(q)
    p1, p2 = np.pi + q[0], np.pi - q[1]
    # Exact corrected firmware expression: unfiltered local geometry only.
    e = length * np.array([np.sin(angle), np.cos(angle)])
    u, v = e - b, e - d
    radial = np.array([np.sin(angle), np.cos(angle)])
    angular = length * np.array([np.cos(angle), -np.sin(angle)])
    den1 = L1 * np.dot(u, [-np.sin(p1), np.cos(p1)])
    den2 = L4 * np.dot(v, [np.sin(p2), np.cos(p2)])
    return np.array([[u @ radial / den1, u @ angular / den1],
                     [v @ radial / den2, v @ angular / den2]])


def difference_jacobian(q):
    eps = 1e-6
    return np.column_stack([(forward(q + eps * u)[0] - forward(q - eps * u)[0]) / (2 * eps)
                            for u in np.eye(2)])


if __name__ == '__main__':
    np.set_printoptions(precision=7, suppress=True)
    # d(phi1,phi2) = diag(1,-1) d(q_first,q_second).
    signs = np.diag([1., -1.])
    max_err = max_identity = max_firmware = max_power = max_expansion = 0.
    count = 0
    for a in np.linspace(-55, 20, 16):
        for b in np.linspace(-20, 55, 16):
            q = np.deg2rad([a, b])
            n = inverse_jacobian(q)
            max_firmware = max(max_firmware, np.max(np.abs(corrected_firmware_n(q) - n)))
            j_encoder = np.linalg.solve(n, signs)
            j_fd = difference_jacobian(q)
            max_err = max(max_err, np.max(np.abs(j_encoder - j_fd)))
            max_identity = max(max_identity, np.max(np.abs(n @ j_fd - signs)))
            for side in (1., -1.):
                j_side = np.diag([1., side]) @ j_encoder
                qdot, wrench = np.array([.7, -.4]), np.array([60., 2.])
                tau = j_side.T @ wrench
                det = np.linalg.det(n)
                if side > 0:  # left: output order motor 1,2
                    expanded = np.array([(n[1, 1]*wrench[0]-n[1, 0]*wrench[1])/det,
                                         (n[0, 1]*wrench[0]-n[0, 0]*wrench[1])/det])
                else:         # right: output order motor 4,3
                    expanded = np.array([(n[1, 1]*wrench[0]+n[1, 0]*wrench[1])/det,
                                         (n[0, 1]*wrench[0]+n[0, 0]*wrench[1])/det])
                max_expansion = max(max_expansion, np.max(np.abs(tau-expanded)))
                max_power = max(max_power, abs(tau @ qdot - wrench @ (j_side @ qdot)))
            count += 1
    print(f'poses={count}; max forward derivative error={max_err:.3e}')
    print(f'max N*J_fd-diag(1,-1) error={max_identity:.3e}')
    print(f'max corrected firmware N error={max_firmware:.3e}')
    print(f'max expanded motor-torque error={max_expansion:.3e}')
    print(f'max virtual power residual, both sides={max_power:.3e}')
    assert (max_err < 1e-7 and max_identity < 1e-6 and max_firmware < 1e-10
            and max_expansion < 1e-10 and max_power < 1e-10)
    q = np.deg2rad([-30., 30.])
    print('example encoder positions (deg):', np.rad2deg(q))
    print('example [length, local leg angle]:', forward(q)[0])
    print('correct N=d(phi)/d(length,local angle):\n', inverse_jacobian(q))
    print('corrected firmware N:\n', corrected_firmware_n(q))
    print('correct J_encoder=d(length,local angle)/d(q):\n', difference_jacobian(q))

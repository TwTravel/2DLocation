
import numpy as np
print ("hello world")

def quat2matA(w, x, y, z):
    """ Convert Quaternion to Euler Angles.  See rotation.py for notes """
    quat = np.empty((1, 4), dtype=np.float64)
    quat[0,0] = w
    quat[0,1] = x
    quat[0,2] = y
    quat[0,3] = z
    quat = np.asarray(quat, dtype=np.float64)
    print(quat.shape[-1])
    assert quat.shape[-1] == 4, "Invalid shape quat {}".format(quat)

    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    Nq = np.sum(quat * quat, axis=-1)
    s = 2.0 / Nq
    X, Y, Z = x * s, y * s, z * s
    wX, wY, wZ = w * X, w * Y, w * Z
    xX, xY, xZ = x * X, x * Y, x * Z
    yY, yZ, zZ = y * Y, y * Z, z * Z

    mat = np.empty(quat.shape[:-1] + (3, 3), dtype=np.float64)
    mat[..., 0, 0] = 1.0 - (yY + zZ)
    mat[..., 0, 1] = xY - wZ
    mat[..., 0, 2] = xZ + wY
    mat[..., 1, 0] = xY + wZ
    mat[..., 1, 1] = 1.0 - (xX + zZ)
    mat[..., 1, 2] = yZ - wX
    mat[..., 2, 0] = xZ - wY
    mat[..., 2, 1] = yZ + wX
    mat[..., 2, 2] = 1.0 - (xX + yY)
    return np.where((Nq > 0.000001)[..., np.newaxis, np.newaxis], mat, np.eye(3))


print (quat2matA(1.0, 0, 0, 0))
#!/usr/bin/env python
import numpy as np
import rtde_receive
from scipy.spatial.transform import Rotation as R
from rich import print
import math

'''raw_image/qr_coordinates '''
'''Se reciben las coordenadas de Hololens y se contruye la matriz de transformación de HL2 respecto del cobot. Se habilita al suscriptor de coordenadas de la cámara'''
##################### CONTRUIR MATRIZ DE TRANSFORMACIÓN HOMOGÉNEA
def get_transform_matrix_3points(p1,p2,p3): # Origen, X, Y
    U = (p1 - p2)/ np.linalg.norm(p1 - p2)
    D=p3-p1
    B=np.cross(U, D)
    W=np.divide(B,np.linalg.norm(B))
    C=np.cross(W, U)
    V=np.divide(C,np.linalg.norm(C))

    # Transformation matrix of box respect camera frame
    mth = np.array([[U[0], V[0], W[0], p1[0]],
                         [U[1], V[1], W[1],  p1[1]],
                         [U[2], V[2], W[2],  p1[2]],
                         [   0,  0,    0,     1]])
    return mth

def refA2refB(Tab, point):
    p_A = np.array([point[0],point[1],point[2], 1]).reshape((4,1))
    # Point in tool reference
    p_B = np.dot(Tab, p_A)
    p_B = p_B[:3].flatten()
    return p_B

def euclidean_distance3D(vectA, vectB):
    x_diff = vectB[0] - vectA[0]
    y_diff = vectB[1] - vectA[1]
    z_diff = vectB[2] - vectA[2]
    distance = math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
    return distance

def pose2mth(pose):
    vec=R.from_rotvec(pose[3:])
    mat=vec.as_matrix()

    # Transformation Matrix (4x4)
    T = np.eye(4)
    T[:3, :3] = mat
    T[:3, 3] = np.array(pose[:3])
    mth=T
    return mth
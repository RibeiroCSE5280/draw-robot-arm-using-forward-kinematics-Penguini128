from vedo import *
import numpy as np
from math import cos

def RotationMatrix(theta, axis_name):
    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
  
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij
  

def setAngles(phis, lengths):


  sphereRadius = 0.5

  # Set lengths and angles
  L1, L2, L3 = lengths
  phi1, phi2, phi3 = phis
  
  # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame) 
  R_01 = RotationMatrix(phi1, axis_name = 'z')   # Rotation matrix
  R_011 = RotationMatrix(-90, axis_name='y')
  t_01 = np.array([[3],[2], [0.0]])
  
  T_01 = getLocalFrameMatrix(R_011 @ R_01, t_01)         # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
  
  # Create the coordinate frame mesh and transform
  Frame1 = createCoordinateFrameMesh()
  Frame1 += Cylinder(r=0.4, height=L1, pos = (L1/2,0,0), c="yellow", alpha=.8, axis=(1,0,0))
  Frame1 += Sphere(r=sphereRadius).color("gray").alpha(.8)

  # Transform the part to position it at its correct location and orientation 
  Frame1.apply_transform(T_01)  
  
  # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)   
  R_12 = RotationMatrix(phi2, axis_name = 'z')   # Rotation matrix
  t_12 = np.array([[L1],[0.0], [0.0]])         # Translation vector
  
  # Matrix of Frame 2 w.r.t. Frame 1 
  T_12 = getLocalFrameMatrix(R_12, t_12)
  
  # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
  T_02 = T_01 @ T_12
  
  # Create the coordinate frame mesh and transform
  Frame2 = createCoordinateFrameMesh()
  Frame2 += Cylinder(r=0.4, height=L2, pos = (L2/2,0,0), c="red", alpha=.8, axis=(1,0,0))
  Frame2 += Sphere(r=sphereRadius).color("gray").alpha(.8)
 
  # Transform the part to position it at its correct location and orientation 
  Frame2.apply_transform(T_02)  
  
  # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)   
  R_23 = RotationMatrix(phi3, axis_name = 'z')   # Rotation matrix
  p3   = np.array([[L2],[0.0], [0.0]])           # Frame's origin (w.r.t. previous frame)
  t_23 = p3                                      # Translation vector
  
  # Matrix of Frame 3 w.r.t. Frame 2 
  T_23 = getLocalFrameMatrix(R_23, t_23)
  
  # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
  T_03 = T_02 @ T_23
  
  # Create the coordinate frame mesh and transform. This point is the end-effector. So, I am 
  # just creating the coordinate frame. 
  Frame3 = createCoordinateFrameMesh()
  Frame3 += Cylinder(r=0.4, height=1, pos = (1/2,0,0), c="blue", alpha=.8, axis=(1,0,0))
  Frame3 += Sphere(r=0.5).color("gray").alpha(.8)

  # Transform the part to position it at its correct location and orientation 
  Frame3.apply_transform(T_03)  


  axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))
  # Show everything 
  plt = show([Frame1, Frame2, Frame3], viewup="z", axes=axes, new=False, bg='beige', bg2='lb', offscreen=False, interactive=False)
  plt.remove(Frame1)
  plt.remove(Frame2)
  plt.remove(Frame3)
  plt.remove(axes)

  
def loop_function(time):
  return (cos(time) + 1)/2

def main(): 


  
  start_angles = (70, -50, -50)
  end_angles = (-20, 50, 120)
  lengths = (8, 5, 1)
  
  timer = 0
  speed = 0.1
  while (True):
    loop_time = loop_function(timer)
    current_angles = (start_angles[0] * loop_time + end_angles[0] * (1 - loop_time),
                      start_angles[1] * loop_time + end_angles[1] * (1 - loop_time),
                      start_angles[2] * loop_time + end_angles[2] * (1 - loop_time),)
    setAngles(current_angles, lengths)
    timer += speed

if __name__ == '__main__':
    main()




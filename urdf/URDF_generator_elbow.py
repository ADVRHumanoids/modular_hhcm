import sys
import re
import xml.etree.ElementTree as ET
import xacro

from read_yaml import read_yaml

import tf  

def main():    

  ET.register_namespace('xacro', "http://ros.org/wiki/xacro")

  #obtaining tree from base file
  tree = ET.parse('ModularBot_new.urdf.xacro')

  root = tree.getroot()

  count=0
  origin, xaxis, yaxis, zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)
  
  T = tf.transformations.translation_matrix(origin)
  R = tf.transformations.identity_matrix()
  H0 = tf.transformations.concatenate_matrices(T, R)
  
  J1 = read_yaml("module_elbow.yaml")
  count+=1

  J1.get_rototranslation(H0, J1.Proximal_tf)

  p=str(J1.kinematics.joint.distal.p_dl)
  n=str(J1.kinematics.joint.distal.n_dl)
  
  #adding 3 links to the tree
  a = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_elbow", suffix = str(count), p = p, n= n, x = J1.x, y= J1.y, z= J1.z, roll= J1.roll, pitch= J1.pitch, yaw= J1.yaw)

  ###########################################################

  J2 = read_yaml("module_elbow.yaml")
  count+=1

  J2.get_rototranslation(J1.Distal_tf, J2.Proximal_tf)

  p=str(J2.kinematics.joint.distal.p_dl)
  n=str(J2.kinematics.joint.distal.n_dl)

  b = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_elbow", suffix = str(count), p = p, n= n, x = J2.x, y= J2.y, z= J2.z, roll= J2.roll, pitch=J2.pitch, yaw=J2.yaw)
  
  ###########################################################
  
  J3 = read_yaml("module_elbow.yaml")
  count+=1

  J3.get_rototranslation(J2.Distal_tf, J3.Proximal_tf)

  p=str(J3.kinematics.joint.distal.p_dl)
  n=str(J3.kinematics.joint.distal.n_dl)
  
  c = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_elbow", suffix = str(count), p = p, n= n, x = J3.x, y= J3.y, z= J3.z, roll= J3.roll, pitch=J3.pitch, yaw=J3.yaw)

  ###########################################################

  #writing .xacro file
  tree.write(sys.argv[1], xml_declaration=True, encoding='utf-8')

  #converting from .xacro to .urdf
  xacro.main()

if __name__ == '__main__':
  main()
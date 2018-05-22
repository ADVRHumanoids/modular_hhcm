import sys
import re
import xml.etree.ElementTree as ET
import xacro

from read_yaml import read_yaml

import tf

# class get_rototranslation(link):
  

def main():    

  ET.register_namespace('xacro', "http://ros.org/wiki/xacro")

  #obtaining tree from base file
  tree = ET.parse('ModularBot_new.urdf.xacro')

  root = tree.getroot()


  # for child in root:
  #   print child.tag, child. attrib, child.text

  # childs=root.findall('link')

  # count = 0
  # for link in root.iter('link'):
  #     if link.get('suffix') > count :
  #       count = link.get('suffix')
  #       LastElement = link

  length_value = str(0.8)

  origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

  mydict = read_yaml("module_angular.yaml")

  proximal=mydict.kinematics.joint.proximal
  distal=mydict.kinematics.joint.distal

  H1=tf.transformations.rotation_matrix(proximal.delta_pl, zaxis)
  H2=tf.transformations.translation_matrix((0,0,proximal.p_pl))
  H3=tf.transformations.translation_matrix((proximal.a_pl,0,0))
  H4=tf.transformations.rotation_matrix(proximal.alpha_pl, xaxis)
  H5=tf.transformations.translation_matrix((0,0,proximal.n_pl))

  P=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(P)

  x = str(trans[0])
  y = str(trans[1])
  z = str(trans[2] + 0.4)
  roll=str(angles[0])
  pitch=str(angles[1])
  yaw=str(angles[2])

  H1=tf.transformations.translation_matrix((0,0,distal.p_dl))
  H2=tf.transformations.translation_matrix((distal.a_dl,0,0))
  H3=tf.transformations.rotation_matrix(distal.alpha_dl, xaxis)
  H4=tf.transformations.translation_matrix((0,0,distal.n_dl))
  H5=tf.transformations.rotation_matrix(3.14, zaxis)

  D=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(D)

  p=str(distal.p_dl)
  n=str(distal.n_dl)
  
  #adding 3 links to the tree
  a = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_angular", suffix = '1', p = p, n= n, x = x, y= y, z= z, roll= roll, pitch= pitch, yaw= yaw)

  

  ###########################################################

  mydict = read_yaml("module_angular.yaml")

  proximal=mydict.kinematics.joint.proximal
  distal=mydict.kinematics.joint.distal

  H1=tf.transformations.rotation_matrix(proximal.delta_pl, zaxis)
  H2=tf.transformations.translation_matrix((0,0,proximal.p_pl))
  H3=tf.transformations.translation_matrix((proximal.a_pl,0,0))
  H4=tf.transformations.rotation_matrix(proximal.alpha_pl, xaxis)
  H5=tf.transformations.translation_matrix((0,0,proximal.n_pl))

  P=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(P)

  F=tf.transformations.concatenate_matrices(D, P)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(F)

  x = str(trans[0])
  y = str(trans[1])
  z = str(trans[2])
  roll=str(angles[0])
  pitch=str(angles[1])
  yaw=str(angles[2])

  H1=tf.transformations.translation_matrix((0,0,distal.p_dl))
  H2=tf.transformations.translation_matrix((distal.a_dl,0,0))
  H3=tf.transformations.rotation_matrix(distal.alpha_dl, xaxis)
  H4=tf.transformations.translation_matrix((0,0,distal.n_dl))
  H5=tf.transformations.rotation_matrix(3.14, zaxis)

  D=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(D)

  p=str(distal.p_dl)
  n=str(distal.n_dl)

  b = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_angular", suffix = '2', p = p, n= n, x = x, y= y, z= z, roll= roll, pitch=pitch, yaw=yaw)
  

  ###########################################################
  mydict = read_yaml("module_angular.yaml")

  proximal=mydict.kinematics.joint.proximal
  distal=mydict.kinematics.joint.distal

  H1=tf.transformations.rotation_matrix(proximal.delta_pl, zaxis)
  H2=tf.transformations.translation_matrix((0,0,proximal.p_pl))
  H3=tf.transformations.translation_matrix((proximal.a_pl,0,0))
  H4=tf.transformations.rotation_matrix(proximal.alpha_pl, xaxis)
  H5=tf.transformations.translation_matrix((0,0,proximal.n_pl))

  P=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(P)

  F=tf.transformations.concatenate_matrices(D, P)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(F)

  x2 = str(trans[0])
  y2 = str(trans[1])
  z2 = str(trans[2])
  roll2=str(angles[0])
  pitch2=str(angles[1])
  yaw2=str(angles[2])


  H1=tf.transformations.translation_matrix((0,0,distal.p_dl))
  H2=tf.transformations.translation_matrix((distal.a_dl,0,0))
  H3=tf.transformations.rotation_matrix(distal.alpha_dl, xaxis)
  H4=tf.transformations.translation_matrix((0,0,distal.n_dl))
  H5=tf.transformations.rotation_matrix(distal.delta_dl, zaxis)

  F=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

  scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(F)

  p=str(distal.p_dl)
  n=str(distal.n_dl)
  
  c = ET.SubElement(root, "{http://ros.org/wiki/xacro}add_link_angular", suffix = '3', p = p, n= n, x = x2, y= y2, z= z2, roll= roll2, pitch=pitch2, yaw=yaw2)

  x3 = str(trans[0])
  y3 = str(trans[1])
  z3 = str(trans[2])
  roll3=str(angles[0])
  pitch3=str(angles[1])
  yaw3=str(angles[2])

  #writing .xacro file
  tree.write(sys.argv[1], xml_declaration=True, encoding='utf-8')

  #converting from .xacro to .urdf
  xacro.main()

if __name__ == '__main__':
  main()
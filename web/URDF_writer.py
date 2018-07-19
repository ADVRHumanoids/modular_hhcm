import os, sys
currDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.abspath(os.path.join(currDir, '../..'))
if rootDir not in sys.path: # add parent dir to paths
  sys.path.append(rootDir)

import re
import xml.etree.ElementTree as ET
import xacro
import xml.dom.minidom

from read_yaml import read_yaml, Module

import modular

import tf  

ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')

#obtaining tree from base file
path_name = os.path.dirname(modular.__file__)
basefile_name=path_name + '/urdf/ModularBot_new.urdf.xacro'
urdf_tree = ET.parse(basefile_name)

root = urdf_tree.getroot()

i=0
joints=0
suffix=str(joints)
suffix_bis=suffix

Modules=[]
origin, xaxis, yaxis, zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

T = tf.transformations.translation_matrix(origin)
R = tf.transformations.identity_matrix()
H0 = tf.transformations.concatenate_matrices(T, R)
data = {'Homogeneous_tf': H0, 'type': "link"}
Modules.append(Module(data))
#print(Modules[0].type)
i=i+1

size = 3

def main(filename):
  global i, joints, suffix, suffix_bis, size
  module_name = path_name + '/web/static/yaml/' + filename

  Modules.append(read_yaml(module_name))

  #Modules[i].size = str(size)
  
  setattr(Modules[i], 'size', str(size))
  print(Modules[i].size)

  #print(i)
  #print(Modules[i].type)
  
  if(Modules[i-1].type == 'joint'):
    if(Modules[i].type == 'joint'):
      #joint + joint
      data = {'result' == 'Cannot attach two consecutive joints'}
      return data
    else:
      #joint + link
      Modules[i].get_rototranslation(Modules[i-1].Distal_tf, tf.transformations.identity_matrix())
      #ET.SubElement(root, "xacro:add_link", suffix = suffix, size_y = Modules[i].link_size_y, size_z = Modules[i].link_size_z)
      ET.SubElement(root, "xacro:add_fixed_joint", suffix = suffix_bis, x = Modules[i].x, y= Modules[i].y, z= Modules[i].z, roll= Modules[i].roll, pitch= Modules[i].pitch, yaw= Modules[i].yaw)
      suffix_bis = suffix_bis + '_bis'
      if(Modules[i].type == 'link'):
        ET.SubElement(root, "xacro:add_link", suffix = suffix_bis, size_z = Modules[i].link_size_z, size = Modules[i].size)
      elif(Modules[i].type == 'elbow'):
        ET.SubElement(root, "xacro:add_elbow", suffix = suffix_bis, size_y = Modules[i].link_size_y, size_z = Modules[i].link_size_z, size = Modules[i].size)
      else:
        if(size>1):
          ET.SubElement(root, "xacro:add_size_adapter", suffix = suffix_bis, size_z = Modules[i].link_size_z, size_in = Modules[i].size_in, size_out = Modules[i].size_out)
          size = Modules[i].size_out
        else:
          #ERROR
          print("Error")
  else:
    if(Modules[i].type == 'joint'):
      #link + joint
      Modules[i].get_rototranslation(Modules[i-1].Homogeneous_tf, tf.transformations.identity_matrix())
      joints=joints+1
      suffix=str(joints)
      ET.SubElement(root, "xacro:add_fixed_joint_stator", suffix = suffix, suffix_bis = suffix_bis, x = Modules[i].x, y= Modules[i].y, z= Modules[i].z, roll= Modules[i].roll, pitch= Modules[i].pitch, yaw= Modules[i].yaw)
      ET.SubElement(root, "xacro:add_joint_stator", suffix = suffix, size_y = Modules[i].joint_size_y, size_z = Modules[i].joint_size_z, size = Modules[i].size)
      Modules[i].get_rototranslation(tf.transformations.identity_matrix(), Modules[i].Proximal_tf)
      jointData = Modules[i].kinematics.joint.joint
      upper_lim=str(jointData.upper_limit)
      lower_lim=str(jointData.lower_limit)
      effort=str(jointData.effort)
      velocity=str(jointData.velocity)
      ET.SubElement(root, "xacro:add_joint", suffix = suffix, x = Modules[i].x, y= Modules[i].y, z= Modules[i].z, roll= Modules[i].roll, pitch= Modules[i].pitch, yaw= Modules[i].yaw, upper_lim=upper_lim, lower_lim=lower_lim, effort=effort, velocity=velocity)
      suffix_bis=suffix

    else:
      #link + link
      Modules[i].get_rototranslation(Modules[i-1].Homogeneous_tf, tf.transformations.identity_matrix())
      
      ET.SubElement(root, "xacro:add_fixed_joint", suffix = suffix_bis, x = Modules[i].x, y= Modules[i].y, z= Modules[i].z, roll= Modules[i].roll, pitch= Modules[i].pitch, yaw= Modules[i].yaw)
      suffix_bis = suffix_bis + '_bis'
      
      if(Modules[i].type == 'link'):
        ET.SubElement(root, "xacro:add_link", suffix = suffix_bis, size_z = Modules[i].link_size_z, size = Modules[i].size)
      elif(Modules[i].type == 'elbow'):
        ET.SubElement(root, "xacro:add_elbow", suffix = suffix_bis, size_y = Modules[i].link_size_y, size_z = Modules[i].link_size_z, size = Modules[i].size)
      else:
        if(size>1):
          ET.SubElement(root, "xacro:add_size_adapter", suffix = suffix_bis, size_z = Modules[i].link_size_z, size_in = Modules[i].size_in, size_out = Modules[i].size_out)
          size=Modules[i].size_out
        else:
          #ERROR
          print("Error")

  #update the urdf file, adding the new module 
  string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

  data = {'result': string, 'lastModule': Modules[i].type, 'size': size}
  # data = jsonify(data)

  i=i+1

  return data


#Function writin the urdf file after converting from .xacro (See xacro/__init__.py for reference)
def write_urdf(urdf_filename, tree):
  
  out=xacro.open_output(urdf_filename)

  urdf_xacro_filename = urdf_filename + '.xacro'

  #writing .xacro file
  # tree.write(urdf_xacro_filename, xml_declaration=True, encoding='utf-8')
  xmlstr = xml.dom.minidom.parseString(ET.tostring(tree.getroot())).toprettyxml(indent="   ")
  with open(urdf_xacro_filename, "w") as f:
    f.write(xmlstr)
  

  #parse the document into a xml.dom tree
  doc = xacro.parse(None, urdf_xacro_filename)
  #perform macro replacement
  xacro.process_doc(doc)
  
  #print(doc.lastChild.toprettyxml(indent='  '))

  # add xacro auto-generated banner
  banner = [xml.dom.minidom.Comment(c) for c in
            [" %s " % ('=' * 83),
              " |    This document was autogenerated by xacro from %-30s | " % urdf_xacro_filename,
              " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
              " %s " % ('=' * 83)]]
  first = doc.firstChild
  for comment in banner:
    doc.insertBefore(comment, first)

  out.write(doc.toprettyxml(indent='  '))

  return doc.toprettyxml(indent='  ')
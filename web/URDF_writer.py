import os, sys
currDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.abspath(os.path.join(currDir, '../..'))
if rootDir not in sys.path: # add parent dir to paths
  sys.path.append(rootDir)

import re
import xml.etree.ElementTree as ET
import xacro
import xml.dom.minidom

from read_yaml import read_yaml, ModuleNode

import modular

import tf  

#from anytree import NodeMixin, RenderTree, Node, AsciiStyle
import anytree

ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')
ns = {'xacro': 'http://ros.org/wiki/xacro'}

#obtaining tree from base file
path_name = os.path.dirname(modular.__file__)
basefile_name=path_name + '/urdf/ModularBot_new.urdf.xacro'
urdf_tree = ET.parse(basefile_name)

root = urdf_tree.getroot()

i=0
joints=0
fixed_joints=0

origin, xaxis, yaxis, zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

T = tf.transformations.translation_matrix(origin)
R = tf.transformations.identity_matrix()
H0 = tf.transformations.concatenate_matrices(T, R)
data = {'Homogeneous_tf': H0, 'type': "link", 'name': "L_0", 'link_index': 0}
L_0 = ModuleNode(data, 'L_0')
#print(Modules[0].type)
i=i+1

size = 3

def add_module(filename, father):
  global i, joints, fixed_joints, size
  module_name = path_name + '/web/static/yaml/' + filename

  print(father)
  parent_module = anytree.search.findall_by_attr(L_0, father)[0]
  print(parent_module)
  new_module = read_yaml(module_name, parent_module)

  #new_module.size = str(size)
  
  setattr(new_module, 'size', str(size))
  #print(new_module.size)

  print(i)
  
  if(parent_module.type == 'joint'):
    if(new_module.type == 'joint'):
      #joint + joint
      data = {'result' == 'Cannot attach two consecutive joints'}
      return data
    else:
      #joint + link
      new_module.get_rototranslation(parent_module.Distal_tf, tf.transformations.identity_matrix())

      fixed_joints=1
      if(new_module.type == 'link'):
        setattr(new_module,'name', 'L_'+str(joints)+'_link_'+str(fixed_joints))
        ET.SubElement(root, "xacro:add_link", type = "link", name = new_module.name, size_z = new_module.link_size_z, size = new_module.size)
      elif(new_module.type == 'elbow'):
        setattr(new_module,'name', 'L_'+str(joints)+'_elbow_'+str(fixed_joints))
        ET.SubElement(root, "xacro:add_elbow", type = "elbow", name = new_module.name, size_y = new_module.link_size_y, size_z = new_module.link_size_z, size = new_module.size)
      else:
        if(size>1):
          setattr(new_module,'name', 'L_'+str(joints)+'_size_adapter_'+str(fixed_joints))
          ET.SubElement(root, "xacro:add_size_adapter", type = "size_adapter", name = new_module.name, size_z = new_module.link_size_z, size_in = new_module.size_in, size_out = new_module.size_out)
          size = new_module.size_out
        else:
          #ERROR
          print("Error")

      fixed_joint_name = 'L_'+str(joints)+'_fixed_joint_'+str(fixed_joints)
      ET.SubElement(root, "xacro:add_fixed_joint", type = "fixed_joint", name = fixed_joint_name, father = 'L_'+str(joints), child = new_module.name, x = new_module.x, y= new_module.y, z= new_module.z, roll= new_module.roll, pitch= new_module.pitch, yaw= new_module.yaw)
      
  else:
    if(new_module.type == 'joint'):
      #link + joint
      new_module.get_rototranslation(parent_module.Homogeneous_tf, tf.transformations.identity_matrix())
      joints=joints+1
      setattr(new_module, 'name', 'J' + str(joints))
      stator_name = 'L_' + str(joints-1) + '_' + new_module.name + '_stator'
      joint_stator_name = "fixed__"+new_module.name
      ET.SubElement(root, "xacro:add_fixed_joint_stator", type = "fixed_joint_stator", name = joint_stator_name, father = parent_module.name, child = stator_name, x = new_module.x, y= new_module.y, z= new_module.z, roll= new_module.roll, pitch= new_module.pitch, yaw= new_module.yaw)
      ET.SubElement(root, "xacro:add_joint_stator", type = "joint_stator", name = stator_name, size_y = new_module.joint_size_y, size_z = new_module.joint_size_z, size = new_module.size)
      new_module.get_rototranslation(tf.transformations.identity_matrix(), new_module.Proximal_tf)
      jointData = new_module.kinematics.joint.joint
      upper_lim=str(jointData.upper_limit)
      lower_lim=str(jointData.lower_limit)
      effort=str(jointData.effort)
      velocity=str(jointData.velocity)
      ET.SubElement(root, "xacro:add_joint", type = "joint", name = new_module.name, father = stator_name, child = 'L_' + str(joints), x = new_module.x, y= new_module.y, z= new_module.z, roll= new_module.roll, pitch= new_module.pitch, yaw= new_module.yaw, upper_lim=upper_lim, lower_lim=lower_lim, effort=effort, velocity=velocity)

    else:
      #link + link
      new_module.get_rototranslation(parent_module.Homogeneous_tf, tf.transformations.identity_matrix())
      print(new_module.z)
  
      fixed_joints+=1
      
      if(new_module.type == 'link'):
        setattr(new_module,'name', 'L_'+str(joints)+'_link_'+str(fixed_joints))
        ET.SubElement(root, "xacro:add_link", type = "link", name = new_module.name, size_z = new_module.link_size_z, size = new_module.size)
      elif(new_module.type == 'elbow'):
        setattr(new_module,'name', 'L_'+str(joints)+'_elbow_'+str(fixed_joints))
        ET.SubElement(root, "xacro:add_elbow", type = "elbow", name = new_module.name, size_y = new_module.link_size_y, size_z = new_module.link_size_z, size = new_module.size)
      else:
        if(size>1):
          setattr(new_module,'name', 'L_'+str(joints)+'_size_adapter_'+str(fixed_joints))
          ET.SubElement(root, "xacro:add_size_adapter", type = "size_adapter", name = new_module.name, size_z = new_module.link_size_z, size_in = new_module.size_in, size_out = new_module.size_out)
          size=new_module.size_out
        else:
          #ERROR
          print("Error")
      
      fixed_joint_name = 'L_'+str(joints)+'_fixed_joint_'+str(fixed_joints)
      ET.SubElement(root, "xacro:add_fixed_joint", name = fixed_joint_name, type = "fixed_joint", father = parent_module.name, child = new_module.name, x = new_module.x, y= new_module.y, z= new_module.z, roll= new_module.roll, pitch= new_module.pitch, yaw= new_module.yaw)
      
  setattr(new_module,'link_index', fixed_joints)

  #update the urdf file, adding the new module 
  string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

  #Render tree
  for pre, _, node in anytree.render.RenderTree(L_0):
    print("%s%s" % (pre, node.name))

  i=i+1

  data = {'result': string, 'lastModule_type': new_module.type, 'lastModule_name': new_module.name, 'size': size, 'count': i}
  # data = jsonify(data)

  return data

def remove_module(father):
  global i, size, joints, fixed_joints
  last_module = anytree.search.findall_by_attr(L_0, father)[0]
  if (last_module.type == 'joint'):
    root.remove(root.findall("*[@type='joint']", ns)[-1])
    root.remove(root.findall("*[@type='joint_stator']", ns)[-1])
    root.remove(root.findall("*[@type='fixed_joint_stator']", ns)[-1])
    joints=joints-1
  else:
    #print(root.findall("*[@type='link']", ns)[-1])
    root.remove(root.findall("*[@type='fixed_joint']", ns)[-1])
    if (last_module.type == 'link'):
      root.remove(root.findall("*[@type='link']", ns)[-1])
    elif (last_module.type == 'elbow'):
      root.remove(root.findall("*[@type='elbow']", ns)[-1])
    elif (last_module.type == 'size_adapter'):
      root.remove(root.findall("*[@type='size_adapter']", ns)[-1])
      size=last_module.size_in

  i=i-1
  fixed_joints=last_module.link_index

  #update the urdf file, removing the module 
  string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

  data = {'result': string, 'lastModule_type': last_module.parent.type, 'lastModule_name': last_module.parent.name, 'size': size, 'count': i}
  # data = jsonify(data)

  #last_module.parent.children = None
  last_module.parent = None
  del last_module

  #Render tree
  for pre, _, node in anytree.render.RenderTree(L_0):
    print("%s%s" % (pre, node.name))

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
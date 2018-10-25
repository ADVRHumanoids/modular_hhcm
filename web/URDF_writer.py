from __future__ import print_function
import os, sys
currDir = os.path.dirname(os.path.realpath(__file__))
rootDir = os.path.abspath(os.path.join(currDir, '../..'))
if rootDir not in sys.path: # add parent dir to paths
	sys.path.append(rootDir)

import re
import xml.etree.ElementTree as ET
import xacro
import xml.dom.minidom

from read_yaml import module_from_yaml, ModuleNode, mastercube_from_yaml, slavecube_from_yaml

import modular

import tf  

#from anytree import NodeMixin, RenderTree, Node, AsciiStyle
import anytree

ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')
ns = {'xacro': 'http://ros.org/wiki/xacro'}

# #obtaining tree from base file
path_name = os.path.dirname(modular.__file__)
# basefile_name=path_name + '/urdf/ModularBot_new.urdf.xacro'
# urdf_tree = ET.parse(basefile_name)

# root = urdf_tree.getroot()
root = 0
urdf_tree = 0

tag = 1
branch_switcher = {
	1: '_A',
	2 : '_B',
	3 : '_C',
	4 : '_D',
	5 : '_E',
	6 : '_F',
	7 : '_G',
	8 : '_H',
	9: '_I',
	10 : '_L',
	11 : '_M',
	12 : '_N',
	13 : '_O',
	14 : '_P',
	15 : '_Q',
	16 : '_R'
}

n_cubes = 0
cube_switcher = {
	0: 'a',
	1 : 'b',
	2 : 'c',
	3 : 'd',
	4 : 'e',
	5 : 'f',
	6 : 'g',
	7 : 'h'
}

origin, xaxis, yaxis, zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

# T = tf.transformations.translation_matrix(origin)
# R = tf.transformations.identity_matrix()
# H0 = tf.transformations.concatenate_matrices(T, R)
# data = {'Homogeneous_tf': H0, 'type': "link", 'name': "L_0", 'i': 0, 'p': 0, 'size': 3}
# L_0 = ModuleNode(data, 'L_0')

# #create ModuleNode for branch A connector
# origin_A = (0, 0, 0.1) #origin_A = (0, 0.3, 0.3)
# T_A = tf.transformations.translation_matrix(origin_A)
# R_A = tf.transformations.identity_matrix() #rotation_matrix(-1.57, xaxis)
# H0_A = tf.transformations.concatenate_matrices(T_A, R_A)
# data = {'Homogeneous_tf': H0_A, 'type': "link", 'name': "L_0_A", 'i': 0, 'p': 0, 'size': 3, 'tag': "_A"}
# L_0_A = ModuleNode(data, 'L_0_A', parent=L_0)

# #create ModuleNode for branch B connector
# origin_B = (0, 0, 0.1) #origin_B = (0, -0.3, 0.3)
# T_B = tf.transformations.translation_matrix(origin_B)
# R_B = tf.transformations.identity_matrix() #rotation_matrix(1.57, xaxis)
# H0_B = tf.transformations.concatenate_matrices(T_B, R_B)
# data = {'Homogeneous_tf': H0_B, 'type': "link", 'name': "L_0_B", 'i': 0, 'p': 0, 'size': 3, 'tag': "_B"}
# L_0_B = ModuleNode(data, 'L_0_B', parent=L_0)

# data = {'type': "link", 'name': "L_0a", 'i': 0, 'p': 0, 'size': 3}
# base = ModuleNode(data, 'L_0a')
L_0a = mastercube_from_yaml(path_name + '/web/static/yaml/master_cube.yaml')
T_con = tf.transformations.translation_matrix((0, 0, L_0a.geometry.connector_length))
setattr(L_0a, 'name', "L_0a")

#create ModuleNode for branch 1 connector
#H0_1 = L_0a.kinematics.connector_1.Homogeneous_tf
# origin_1 = (0, 0.2, 0.2)
# T_1 = tf.transformations.translation_matrix(origin_1)
# R_1 = tf.transformations.rotation_matrix(-1.57, xaxis)
#H0_1 = tf.transformations.concatenate_matrices(H0_1, T_con)
name_con1 = 'L_0a' + '_con1' 
data1 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con1, 'i': 0, 'p': 0, 'size': 3}
L_0a_con1 = ModuleNode(data1, name_con1, parent=L_0a)

#create ModuleNode for branch 2 connector
#H0_2 = L_0a.kinematics.connector_2.Homogeneous_tf
#origin_2 = (0, -0.2, 0.2)
# T_2 = tf.transformations.translation_matrix(origin_2)
# R_2 = tf.transformations.rotation_matrix(1.57, xaxis)
#H0_2 = tf.transformations.concatenate_matrices(H0_2, T_con)
name_con2 = 'L_0a' + '_con2'
data2 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con2, 'i': 0, 'p': 0, 'size': 3}
L_0a_con2 = ModuleNode(data2, name_con2, parent=L_0a)

#create ModuleNode for branch 3 connector
#H0_3 = L_0a.kinematics.connector_3.Homogeneous_tf
# origin_3 = (0, 0, 0.4)
# T_3 = tf.transformations.translation_matrix(origin_3)
# R_3 = tf.transformations.identity_matrix()
#H0_3 = tf.transformations.concatenate_matrices(H0_3, T_con)
name_con3 = 'L_0a' + '_con3'
data3 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con3, 'i': 0, 'p': 0, 'size': 3}
L_0a_con3 = ModuleNode(data3, name_con3, parent=L_0a)

#create ModuleNode for branch 3 connector
#H0_4 = L_0a.kinematics.connector_4.Homogeneous_tf
# origin_4 = (0, 0, 0)
# T_4 = tf.transformations.translation_matrix(origin_4)
# R_4 = tf.transformations.rotation_matrix(3.14, xaxis)
#H0_4 = tf.transformations.concatenate_matrices(H0_4, T_con)
name_con4 = 'L_0a' + '_con4'
data4 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con4, 'i': 0, 'p': 0, 'size': 3}
L_0a_con4 = ModuleNode(data4, name_con4, parent=L_0a)

#Render tree
for pre, _, node in anytree.render.RenderTree(L_0a):
	print("%s%s" % (pre, node.name))

def add_slave_cube(father):
def add_slave_cube(father, angle_offset):
	global T_con, L_0a, n_cubes
	
	n_cubes+=1
	name = 'L_0' + cube_switcher.get(n_cubes)

	parent_module = anytree.search.findall_by_attr(L_0a, father)[0]

	T_con_inv = tf.transformations.inverse_matrix(T_con)
	name_con1 = name + '_con1' 
	data1 = {'Homogeneous_tf': T_con_inv, 'type': "link", 'name': name_con1, 'i': 0, 'p': 0, 'size': 3}
	slavecube_con1 = ModuleNode(data1, name_con1, parent=parent_module)

	parent_module.get_rototranslation(parent_module.Homogeneous_tf, tf.transformations.rotation_matrix(angle_offset, zaxis))
	fixed_joint_name = 'FJ_' + parent_module.parent.name + '_' + name
	ET.SubElement(root, "xacro:add_fixed_joint", type="fixed_joint", name=fixed_joint_name, father=father, child=name_con1, x=parent_module.x, y=parent_module.y, z=parent_module.z, roll=parent_module.roll, pitch=parent_module.pitch, yaw=parent_module.yaw)

	filename = path_name + '/web/static/yaml/master_cube.yaml'
	slavecube = slavecube_from_yaml(filename, name, slavecube_con1)
	setattr(slavecube, 'name', name)
	setattr(slavecube, 'i', 0)
	setattr(slavecube, 'p', 0)

	ET.SubElement(root, "xacro:add_slave_cube", name=name)

	#create ModuleNode for branch 2 connector
	name_con2 = name + '_con2'
	data2 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con2, 'i': 0, 'p': 0, 'size': 3}
	slavecube_con2 = ModuleNode(data2, name_con2, parent=slavecube)

	#create ModuleNode for branch 3 connector
	name_con3 = name + '_con3'
	data3 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con3, 'i': 0, 'p': 0, 'size': 3}
	slavecube_con3 = ModuleNode(data3, name_con3, parent=slavecube)

	#create ModuleNode for branch 3 connector
	name_con4 = name + '_con4'
	data4 = {'Homogeneous_tf': T_con, 'type': "link", 'name': name_con4, 'i': 0, 'p': 0, 'size': 3}
	slavecube_con4 = ModuleNode(data4, name_con4, parent=slavecube)

	#Render tree
	for pre, _, node in anytree.render.RenderTree(L_0a):
		print("%s%s" % (pre, node.name))

	# new_Link = slavecube_con1 
	# past_Link = parent_module
	# new_Link.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.identity_matrix())
	# print(new_Link.z)

	# ET.SubElement(root, "xacro:add_master_cube", name=name)

	# fixed_joint_name = 'cube_joint'
	# ET.SubElement(root, "xacro:add_fixed_joint", name=fixed_joint_name, type="fixed_joint", father=past_Link.name, child=new_Link.name, x=new_Link.x, y=new_Link.y, z=new_Link.z, roll=new_Link.roll, pitch=new_Link.pitch, yaw=new_Link.yaw)

	#update the urdf file, adding the new module
	string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

	data = {'result': string, 'lastModule_type': 'mastercube', 'lastModule_name': name, 'size': 3, 'count': n_cubes}

	return data


def read_file(file_str):
	"""Open the URDF chosen from the front-end and import it as a tree"""
	global root, urdf_tree
	print(file_str)
	root = ET.fromstring(file_str.encode('utf-8'))
	urdf_tree = ET.ElementTree(root)
	print(ET.tostring(urdf_tree.getroot()))

	#include files necessary for Gazebo&XBot simulation
	# ET.SubElement(root, "xacro:include", filename="$(find modular)/urdf/config.xacro")
	# ET.SubElement(root, "xacro:include", filename="$(find modular)/urdf/modular.gazebo")

	doc = xacro.parse(file_str.encode('utf-8'))
	xacro.process_doc(doc, in_order=True)
	string = doc.toprettyxml(indent='  ')

	data = {'string': string}
	return data

def add_module(filename, selected_module, angle_offset):
	"""Add a module specified by filename to the selected module. Return info on the new module"""
	global tag
	
	if selected_module.endswith('_stator'):
		selected_module = selected_module[:-7]

	module_name = path_name + '/web/static/yaml/' + filename

	print(selected_module)
	parent_module = anytree.search.findall_by_attr(L_0a, selected_module)[0]
	print(parent_module)
	new_module = module_from_yaml(module_name, parent_module)

	print(angle_offset)

	# print(anytree.search.find_by_attr(L_0a, name="tag"))

	# print(parent_module.children)
	# if 'L_0' in parent_module.name:
	# 	if len(parent_module.children) == 1:
	# 		setattr(new_module, 'tag', '_A')
	# 	elif len(parent_module.children) == 2:
	# 		setattr(new_module, 'tag', '_B')
	# 	elif len(parent_module.children) == 3:
	# 		setattr(new_module, 'tag', '_C')
	# 	elif len(parent_module.children) == 4:
	# 		setattr(new_module, 'tag', '_D')
	# else:
	# 	setattr(new_module, 'tag', parent_module.tag)

	if '_con' in parent_module.name:
		tag_letter = branch_switcher.get(tag)
		setattr(new_module, 'tag', tag_letter)
		tag+=1
	else:
	 	setattr(new_module, 'tag', parent_module.tag)


	setattr(new_module, 'size', parent_module.size)
	setattr(new_module, 'i', parent_module.i)
	setattr(new_module, 'p', parent_module.p)

	if parent_module.type == 'joint':
		if new_module.type == 'joint':
			#joint + joint
			data = {'result' == 'Cannot attach two consecutive joints'}
			return data
		else:
			#joint + link
			link_after_joint(new_module, parent_module)
	else:
		if new_module.type == 'joint':
			#link + joint
			joint_after_link(new_module, parent_module)
		else:
			#link + link
			link_after_link(new_module, parent_module, angle_offset)

	#write the urdf tree to a string
	xmlstr = xml.dom.minidom.parseString(ET.tostring(urdf_tree.getroot())).toprettyxml(indent="   ")

	#parse the string to convert from xacro
	doc = xacro.parse(xmlstr)
	#perform macro replacement
	xacro.process_doc(doc)

	string = doc.toprettyxml(indent='  ')

	#update the urdf file, adding the new module
	#string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

	#Render tree
	for pre, _, node in anytree.render.RenderTree(L_0a):
		print("%s%s" % (pre, node.name))

	data = {'result': string, 'lastModule_type': new_module.type, 'lastModule_name': new_module.name, 'size': new_module.size, 'count': new_module.i}

	return data

def remove_module(selected_module):
	"""Remove the selected module and return info on its parent"""
	global tag, n_cubes

	if isinstance(selected_module, basestring):
		last_module = access_module(selected_module)
	else:
		last_module = selected_module
	print(last_module.name)
	if '_con' in last_module.name:
		last_module = last_module.parent

	if last_module.type != 'cube':
		print('eliminate child')
		for child in last_module.children:
			remove_module(child)

	print(last_module.children)
	print(last_module.parent.name)

	if last_module.type == 'joint':
		father = last_module.parent

		stator_name = last_module.name + '_stator'
		joint_stator_name = "fixed_" + last_module.name

		for node in root.findall("*"):
			if node.attrib['name'] == last_module.name:
				root.remove(node)
		for node in root.findall("*"):
			if node.attrib['name'] == stator_name:
				root.remove(node)
		for node in root.findall("*"):
			if node.attrib['name'] == joint_stator_name:
				root.remove(node)
		# root.remove(root.findall("*[@name=last_module.name]", ns)[-1])
		# root.remove(root.findall("*[@name=stator_name]", ns)[-1])
		# root.remove(root.findall("*[@name=joint_stator_name]", ns)[-1])
	elif last_module.type == 'cube':
		father = last_module.parent.parent
		print(last_module.parent.name)
		n_cubes-=1
		for child in last_module.children:
			for node in root.findall("*"):
				if node.attrib['name'] == child.name:
					root.remove(node)
		for node in root.findall("*"):
			if node.attrib['name'] == last_module.name:
				root.remove(node)
		# last_module.parent = None
		parent_module = access_module(last_module.parent.name)
		joint_name = 'FJ_' + parent_module.parent.parent.name + '_' + last_module.name
		print(joint_name)
		for node in root.findall("*"):
			if node.attrib['name'] == joint_name:
				print(joint_name)
				root.remove(node)
		parent_module.parent = None
		del parent_module
	else:
		father = last_module.parent

		#print(root.findall("*[@type='link']", ns)[-1])
		fixed_joint_name = 'L_'+str(last_module.i)+'_fixed_joint_'+str(last_module.p)+last_module.tag
		#root.remove(root.findall("*[@name=fixed_joint_name]", ns)[-1])
		for node in root.findall("*"):
			if node.attrib['name'] == fixed_joint_name:
				root.remove(node)

		if last_module.type == 'link':
			#root.remove(root.findall("*[@name=last_module.name]", ns)[-1])
			for node in root.findall("*"):
				if node.attrib['name'] == last_module.name:
					root.remove(node)
		elif last_module.type == 'elbow':
			#root.remove(root.findall("*[@name=last_module.name]", ns)[-1])
			for node in root.findall("*"):
				if node.attrib['name'] == last_module.name:
					root.remove(node)
		elif last_module.type == 'size_adapter':
			#root.remove(root.findall("*[@name=last_module.name]", ns)[-1])
			for node in root.findall("*"):
				if node.attrib['name'] == last_module.name:
					root.remove(node)

	#update the urdf file, removing the module
	string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)
	if father.type=='cube':
		data = {'result': string, 'lastModule_type': father.type, 'lastModule_name': father.name, 'size': father.size, 'count': n_cubes}
	else:
		data = {'result': string, 'lastModule_type': father.type, 'lastModule_name': father.name, 'size': father.size, 'count': father.i}
	# data = jsonify(data)

	if '_con' in father.name:
		tag-=1
	
	#last_module.parent.children = None
	last_module.parent = None

	del last_module

	#Render tree
	for pre, _, node in anytree.render.RenderTree(L_0a):
		print("%s%s" % (pre, node.name))

	return data

def access_module(selected_module):
	"""Find the selected module in the tree"""
	if selected_module.endswith('_stator'):
		selected_module = selected_module[:-7]
	last_module = anytree.search.findall_by_attr(L_0a, selected_module)[0]

	return last_module

def select_module(module):
	"""Returns info on the selected module"""
	last_module = access_module(module)

	if last_module.type == 'cube':
		data = {'lastModule_type': last_module.type, 'lastModule_name': last_module.name, 'size': last_module.size, 'count': n_cubes}
	else:
		data = {'lastModule_type': last_module.type, 'lastModule_name': last_module.name, 'size': last_module.size, 'count': last_module.i}

	return data

def link_after_joint(new_Link, past_Joint):
	"""Adds to the URDF tree a link module as a child of a joint module"""
	new_Link.get_rototranslation(past_Joint.Distal_tf, tf.transformations.identity_matrix())
	setattr(new_Link, 'p', past_Joint.p + 1)

	if new_Link.type == 'link':
		setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_link_'+str(new_Link.p)+new_Link.tag)
		ET.SubElement(root, "xacro:add_link", type="link", name=new_Link.name, size_z=new_Link.link_size_z, size=str(new_Link.size))
	elif new_Link.type == 'elbow':
		setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_elbow_'+str(new_Link.p)+new_Link.tag)
		ET.SubElement(root, "xacro:add_elbow", type="elbow", name=new_Link.name, size_y=new_Link.link_size_y, size_z=new_Link.link_size_z, size=str(new_Link.size))
	else:
		if new_Link.size > 1:
			setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_size_adapter_'+str(new_Link.p)+new_Link.tag)
			ET.SubElement(root, "xacro:add_size_adapter", type="size_adapter", name=new_Link.name, size_z=new_Link.link_size_z, size_in=new_Link.size_in, size_out=new_Link.size_out)
			setattr(new_Link, 'size', new_Link.size_out)
		else:
			#ERROR
			print("Error")

	fixed_joint_name = 'L_'+str(new_Link.i)+'_fixed_joint_'+str(new_Link.p)+new_Link.tag
	ET.SubElement(root, "xacro:add_fixed_joint", type="fixed_joint", name=fixed_joint_name, father='L_'+str(new_Link.i)+new_Link.tag, child=new_Link.name, x=new_Link.x, y=new_Link.y, z=new_Link.z, roll=new_Link.roll, pitch=new_Link.pitch, yaw=new_Link.yaw)

def joint_after_link(new_Joint, past_Link):
	"""Adds to the URDF tree a joint module as a child of a link module"""
	new_Joint.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.identity_matrix())

	setattr(new_Joint, 'i', past_Link.i + 1)
	setattr(new_Joint, 'p', 0)

	setattr(new_Joint, 'name', 'J' + str(new_Joint.i)+new_Joint.tag)
	stator_name = new_Joint.name + '_stator'
	joint_stator_name = "fixed_" + new_Joint.name
	ET.SubElement(root, "xacro:add_fixed_joint_stator", type="fixed_joint_stator", name=joint_stator_name, father=past_Link.name, child=stator_name, x=new_Joint.x, y=new_Joint.y, z=new_Joint.z, roll=new_Joint.roll, pitch=new_Joint.pitch, yaw=new_Joint.yaw)
	ET.SubElement(root, "xacro:add_joint_stator", type="joint_stator", name=stator_name, size_y=new_Joint.joint_size_y, size_z=new_Joint.joint_size_z, size=str(new_Joint.size))
	new_Joint.get_rototranslation(tf.transformations.identity_matrix(), new_Joint.Proximal_tf)
	jointData = new_Joint.kinematics.joint.joint
	upper_lim = str(jointData.upper_limit)
	lower_lim = str(jointData.lower_limit)
	effort = str(jointData.effort)
	velocity = str(jointData.velocity)
	ET.SubElement(root, "xacro:add_joint", type="joint", name=new_Joint.name, father=stator_name, child='L_'+str(new_Joint.i)+new_Joint.tag, x=new_Joint.x, y=new_Joint.y, z=new_Joint.z, roll=new_Joint.roll, pitch=new_Joint.pitch, yaw=new_Joint.yaw, upper_lim=upper_lim, lower_lim=lower_lim, effort=effort, velocity=velocity)

def link_after_link(new_Link, past_Link, offset):
	"""Adds to the URDF tree a link module as a child of a link module"""
	new_Link.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.rotation_matrix(offset, zaxis))
	print(new_Link.z)

	setattr(new_Link, 'p', past_Link.p + 1)

	if new_Link.type == 'link':
		setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_link_'+str(new_Link.p)+new_Link.tag)
		ET.SubElement(root, "xacro:add_link", type="link", name=new_Link.name, size_z=new_Link.link_size_z, size=str(new_Link.size))
	elif new_Link.type == 'elbow':
		setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_elbow_'+str(new_Link.p)+new_Link.tag)
		ET.SubElement(root, "xacro:add_elbow", type="elbow", name=new_Link.name, size_y=new_Link.link_size_y, size_z=new_Link.link_size_z, size=str(new_Link.size))
	else:
		if new_Link.size > 1:
			setattr(new_Link, 'name', 'L_'+str(new_Link.i)+'_size_adapter_'+str(new_Link.p)+new_Link.tag)
			ET.SubElement(root, "xacro:add_size_adapter", type="size_adapter", name=new_Link.name, size_z=new_Link.link_size_z, size_in=new_Link.size_in, size_out=new_Link.size_out)
			setattr(new_Link, 'size', new_Link.size_out)
		else:
			#ERROR
			print("Error")

	fixed_joint_name = 'L_'+str(new_Link.i)+'_fixed_joint_'+str(new_Link.p)+new_Link.tag
	ET.SubElement(root, "xacro:add_fixed_joint", name=fixed_joint_name, type="fixed_joint", father=past_Link.name, child=new_Link.name, x=new_Link.x, y=new_Link.y, z=new_Link.z, roll=new_Link.roll, pitch=new_Link.pitch, yaw=new_Link.yaw)

#Function writin the urdf file after converting from .xacro (See xacro/__init__.py for reference)
def write_urdf(doc):
	"""Returns the string with the URDF, after writing it to file"""
	urdf_filename = path_name + '/urdf/ModularBot_test.urdf'
	
	out = xacro.open_output(urdf_filename)

	urdf_xacro_filename = urdf_filename + '.xacro'

	# #writing .xacro file
	# # tree.write(urdf_xacro_filename, xml_declaration=True, encoding='utf-8')
	# xmlstr = xml.dom.minidom.parseString(ET.tostring(tree.getroot())).toprettyxml(indent="   ")
	# with open(urdf_xacro_filename, "w") as f:
	# 	f.write(xmlstr)

	# #parse the document into a xml.dom tree
	# doc = xacro.parse(None, urdf_xacro_filename)
	doc = xacro.parse(doc)
	# #perform macro replacement
	# xacro.process_doc(doc)

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
	
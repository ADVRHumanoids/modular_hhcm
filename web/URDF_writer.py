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

from read_yaml import read_yaml, ModuleNode

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

origin, xaxis, yaxis, zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

T = tf.transformations.translation_matrix(origin)
R = tf.transformations.identity_matrix()
H0 = tf.transformations.concatenate_matrices(T, R)
data = {'Homogeneous_tf': H0, 'type': "link", 'name': "L_0", 'i': 0, 'p': 0, 'size': 3}
L_0 = ModuleNode(data, 'L_0')
#print(Modules[0].type)
#i=i+1

#size = 3

def read_file(file_str):
	"""Open the URDF chosen from the front-end and import it as a tree"""
	global root, urdf_tree
	print(file_str)
	root = ET.fromstring(file_str.encode('utf-8'))
	urdf_tree = ET.ElementTree(root)
	print(ET.tostring(urdf_tree.getroot()))

	doc = xacro.parse(file_str.encode('utf-8'))
	xacro.process_doc(doc)
	string = doc.toprettyxml(indent='  ')

	data = {'string': string}
	return data

def add_module(filename, selected_module):
	"""Add a module specified by filename to the selected module. Return info on the new module"""
	if selected_module.endswith('_stator'):
		selected_module = selected_module[:-7]

	module_name = path_name + '/web/static/yaml/' + filename

	print(selected_module)
	parent_module = anytree.search.findall_by_attr(L_0, selected_module)[0]
	print(parent_module)
	new_module = read_yaml(module_name, parent_module)

	print(parent_module.children)
	if parent_module.name == 'L_0':
		if len(parent_module.children) == 1:
			setattr(new_module, 'tag', '_A')
		elif len(parent_module.children) == 2:
			setattr(new_module, 'tag', '_B')
		elif len(parent_module.children) == 3:
			setattr(new_module, 'tag', '_C')
		elif len(parent_module.children) == 4:
			setattr(new_module, 'tag', '_D')
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
			link_after_link(new_module, parent_module)

	#update the urdf file, adding the new module
	string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

	#Render tree
	for pre, _, node in anytree.render.RenderTree(L_0):
		print("%s%s" % (pre, node.name))

	data = {'result': string, 'lastModule_type': new_module.type, 'lastModule_name': new_module.name, 'size': new_module.size, 'count': new_module.i}

	return data

def remove_module(selected_module):
	"""Remove the selected module and return info on its parent"""
	if isinstance(selected_module, basestring):
		last_module = access_module(selected_module)
	else:
		last_module = selected_module

	for child in last_module.children:
		remove_module(child)

	if last_module.type == 'joint':
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

	else:
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

	data = {'result': string, 'lastModule_type': last_module.parent.type, 'lastModule_name': last_module.parent.name, 'size': last_module.parent.size, 'count': last_module.parent.i}
	# data = jsonify(data)

	#last_module.parent.children = None
	last_module.parent = None

	del last_module

	#Render tree
	for pre, _, node in anytree.render.RenderTree(L_0):
		print("%s%s" % (pre, node.name))

	return data

def access_module(selected_module):
	"""Find the selected module in the tree"""
	if selected_module.endswith('_stator'):
		selected_module = selected_module[:-7]
	last_module = anytree.search.findall_by_attr(L_0, selected_module)[0]

	return last_module

def select_module(module):
	"""Returns info on the selected module"""
	last_module = access_module(module)

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

def link_after_link(new_Link, past_Link):
	"""Adds to the URDF tree a link module as a child of a link module"""
	new_Link.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.identity_matrix())
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
def write_urdf(urdf_filename, tree):
	"""Returns the string with the URDF, after writing it to file"""
	out = xacro.open_output(urdf_filename)

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
	
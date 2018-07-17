import yaml
import sys
import tf

# Class describing all the properties of a joint module 
# - Kinematics and dynamics paramters are loaded from a YAML file
# - Methods are provided to compute frame transformations and store them as class attributes 
class Module(dict):
    
    # Method used to import as class attributes the fields of the dictionary obtained by reading the YAML file 
    def __getattr__(self, name):
        value = self[name]
        if isinstance(value, dict):
            value = Module(value)
        return value

    def set_type(self, x):
        print(x)
        switcher = {
            '/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_joint.yaml': "joint",
            '/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_link.yaml' : "link",
            '/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_link_500mm.yaml' : "link",
            '/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_link_700mm.yaml' : "link",
            '/home/edoardo/catkin_ws/src/modular/web/static/yaml/module_elbow.yaml' : "elbow"
        }
        setattr(self, 'type', switcher.get(x,"Invalid file name"))

    def set_size(self, x):
        print(x)
        switcher = {
            'small': 1,
            'medium' : 2,
            'big' : 3,
        }
        setattr(self, 'size', switcher.get(x,"Invalid file name"))

    # Computes the homogeneous transformation matrices for the distal and proximal part of the joint
    def get_proximal_distal_matrices(self):
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        proximal=self.kinematics.joint.proximal
        distal=self.kinematics.joint.distal

        H1=tf.transformations.rotation_matrix(proximal.delta_pl, zaxis)
        H2=tf.transformations.translation_matrix((0,0,proximal.p_pl))
        H3=tf.transformations.translation_matrix((proximal.a_pl,0,0))
        H4=tf.transformations.rotation_matrix(proximal.alpha_pl, xaxis)
        H5=tf.transformations.translation_matrix((0,0,proximal.n_pl))

        P=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

        # Add the transformation matrix for the Proximal part as attribute of the class
        setattr(self, 'Proximal_tf', P)

        H1=tf.transformations.translation_matrix((0,0,distal.p_dl))
        H2=tf.transformations.translation_matrix((distal.a_dl,0,0))
        H3=tf.transformations.rotation_matrix(distal.alpha_dl, xaxis)
        H4=tf.transformations.translation_matrix((0,0,distal.n_dl))
        H5=tf.transformations.rotation_matrix(distal.delta_dl, zaxis) #3.14

        D=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

        # Add the transformation matrix for the Distal part as attribute of the class
        setattr(self, 'Distal_tf', D)

        #TO BE CHECKED!!!
        size_x = 0
        size_y = proximal.p_pl + distal.p_dl
        print(size_y)
        size_z = proximal.n_pl + distal.n_dl 
        print(size_z)

        setattr(self, 'joint_size_x', str(size_x))
        setattr(self, 'joint_size_y', str(size_y))
        setattr(self, 'joint_size_z', str(size_z))
    
     # Computes the homogeneous transformation matrix for the link
    def get_homogeneous_matrix(self):
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        link=self.kinematics.link

        #TO BE CHECKED!!!
        H1=tf.transformations.rotation_matrix(link.delta_l_in, zaxis)
        H2=tf.transformations.translation_matrix((0,0,link.p_l))
        H3=tf.transformations.translation_matrix((link.a_l,0,0))
        H4=tf.transformations.rotation_matrix(link.alpha_l, xaxis)
        H5=tf.transformations.translation_matrix((0,0,link.n_l))
        H6=tf.transformations.rotation_matrix(link.delta_l_out, zaxis)

        H=tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5, H6)

        # Add the transformation matrix for the Proximal part as attribute of the class
        setattr(self, 'Homogeneous_tf', H)

        size_x = 0
        size_y = link.p_l
        size_z = link.n_l

        setattr(self, 'link_size_x', str(size_x))
        setattr(self, 'link_size_y', str(size_y))
        setattr(self, 'link_size_z', str(size_z))
    
    # Computes the rototranslation from the frame centered at the previous joint to the frame at the center of this joint module
    def get_rototranslation(self, Distal_previous, Proximal):
        F=tf.transformations.concatenate_matrices(Distal_previous, Proximal)

        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(F)

        setattr(self, 'x', str(trans[0]))
        setattr(self, 'y', str(trans[1]))
        setattr(self, 'z', str(trans[2]))
        setattr(self, 'roll', str(angles[0]))
        setattr(self, 'pitch', str(angles[1]))
        setattr(self, 'yaw', str(angles[2]))

    def get_transform(self):
        x=self.type
        print(self.type)
        return {
        'joint': self.get_proximal_distal_matrices(),
        'link' : self.get_homogeneous_matrix(),
        'elbow' : self.get_homogeneous_matrix()
        }.get(x, 'Invalid type')


# Function parsing YAML file and returning an instance of a Module class
def read_yaml(filename):
    with open(filename, 'r') as stream:
        try:
            data=yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    result = Module(data)
    result.set_type(filename)
    result.get_transform()
    result.set_size(result.size)
    return result 

def main():
    my_dict = read_yaml(sys.argv[1])
    print(my_dict)

if __name__ == '__main__':
  main()
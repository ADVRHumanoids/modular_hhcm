# MODULAR

We store all files we cannot share here (PDFs, DATA and in general binaries and larger files) in the OwnCloud:

https://advrcloud.iit.it/owncloud/index.php/s/J7PoGhHNs6WVy4Y

To genereate URDF (from 'urdf' folder):

python URDF_generator.py ModularBot_Generated.urdf.xacro > ModularBot_Generated.urdf

To launch rviz:

roslaunch modular display.launch model:='$(find modular)/urdf/ModularBot_Generated.urdf'


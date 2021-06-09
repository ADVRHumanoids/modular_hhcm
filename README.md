# MODULAR

There are 3 ways to install:

1. **pip install**

After cloning the repo, from the main directory run:
`pip install .`
The package can be installed in system, local, or virtualenv site-packages directory.
If you plan to make modifications it and don't want to re-install it every time, install it in 'editable mode' :
`pip install -e .`

2. **Catkin install**

After cloning the repo in the caktin sourcespace run:
`catkin_make install`
Similarly to the superbuild, to run the application some dependencies can be installed with:
```rosdep install modular```
while some python packages need to be installed manually:
```pip install Flask anytree protobuf_to_dict```

When the catkin workspace is sourced the app can be run from everywhere with:
```env FLASK_APP=modular.web flask run```

3. **Superbuild install**

When installing modular from superbuild set through ccmake:
```
PLAYGROUND = ON
SUPERBUILD_modular = ON
```
and then:
```
cd $ROBOTOLOGY_ROOT/build
make modular
```
To run the application some dependencies can be installed with:
```rosdep install modular```
while some python packages need to be installed manually:
```pip install Flask anytree protobuf_to_dict```

When the superbuild is sourced the app can be run from everywhere with:
```env FLASK_APP=modular.web flask run```

  - **Run the simulation**

  After generating the robot with the configurator, and installing the package via the superbuild: 

  ```
  cd $ROBOTOLOGY_ROOT/build
  make modular 
  ```

  Gazebo simulation can be run with:

  ` roslaunch modular modular_world.launch `

  and then the plugins in the **modular_plugins** repo can be used to control the robot. For instance:

  ` rosservice call /Modularbot_switch 1 `
  
########################################

**When reading configuration from hardware**

Python 3 package for Protocol Buffers need to be installed 

` pip3 install protobuf `

and then the .bashrc has to be modified adding the line:

` export PYTHONPATH=$PYTHONPATH:/usr/lib/python3.5/dist-packages `

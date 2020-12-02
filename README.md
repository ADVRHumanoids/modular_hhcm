# MODULAR

- We store all files we cannot share here (PDFs, DATA and in general binaries and larger files) in the OwnCloud:

https://advrcloud.iit.it/owncloud/index.php/s/J7PoGhHNs6WVy4Y

- The XBotCore plugins to control the generated modular robots is at:

https://github.com/ADVRHumanoids/modular_plugins

**Instal Flask**

After cloning the repo, create a virtual environment:

``` 
cd path/to/modular/web
python3 -m venv venv 
```

Then activate the environment and proceed to install Flask

```
. venv/bin/activate
pip install Flask
```

Deactivate the environment

`deactivate`

Install anytree

`pip install anytree`

**Run the configurator tool**

The online tool to configurate the robot can be run with: 

```
cd path/to/modular/web 
python application.py
```

A generic URDF file can be selected to be opened by the online tool or the **ModularBot_new.urdf.xacro** file to start building a modular robot. 
While creating the robot the file **ModularBot.urdf.xacro** will be modified, and can then be opened by rviz or Gazebo.
Also urdf, srdf, joint map, etc. files will be created in the configs folder of the superbuild.

To launch rviz:

` roslaunch modular display.launch model:='$(find modular)/urdf/ModularBot_test.urdf' `

To launch Gazebo:

` roslaunch modular gazebo.launch model:='$(find modular)/urdf/ModularBot_test.urdf' `

**Run the simulation**

After generating the robot with the configurator, and installing the package via the superbuild: 

```
cd $ROBOTOLOGY_ROOT/build
make modular 
```

Gazebo simulation can be run with:

` roslaunch modular modular_world.launch `

and then the plugins in the **modular_plugins** repo can be used to control the robot. For instance:

` rosservice call /Modularbot_switch 1 `


**When reading configuration from hardware**

Python 3 package for Protocol Buffers need to be installed 

` pip3 install protobuf `

and then the .bashrc has to be modified adding the line:

` export PYTHONPATH=$PYTHONPATH:/usr/lib/python3.5/dist-packages `

**Superbuild install**

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

**Catkin install**

After cloning the repo in the caktin sourcespace run:
`catkin_make install`
Similarly to the superbuild, to run the application some dependencies can be installed with:
```rosdep install modular```
while some python packages need to be installed manually:
```pip install Flask anytree protobuf_to_dict```

When the catkin workspace is sourced the app can be run from everywhere with:
```env FLASK_APP=modular.web flask run```

**pip install**

After cloning the repo, from the main directory run:
`pip install .`
The package can be installed in system, local, or virtualenv site-packages directory.
If you plan to make modifications it and don't want to re-install it every time, install it in 'editable mode' :
`pip install -e .`

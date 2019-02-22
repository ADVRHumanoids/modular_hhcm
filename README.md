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
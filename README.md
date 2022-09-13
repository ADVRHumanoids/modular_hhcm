<!-- PROJECT SHIELDS -->
<!-- These badges can be used once we make the project public -->
<!-- [![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url] -->

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/ADVRHumanoids/modular">
    <img src="https://alberobotics.it/images/apple-touch-icon.png" alt="Logo" width="80" height="80">
  </a>

  <h2 align="center">modular</h2>

  <p align="center">
    Redefining flexible automation.
    <br />
    <a href="https://www.youtube.com/channel/UCNyqcpavE5nsVidipXZQ8OQ">View Demo</a>
    ·
    <a href="https://github.com/ADVRHumanoids/modular/issues">Request Feature</a>
    ·
    <a href="https://github.com/ADVRHumanoids/modular/issues">Report Bug</a>
    <br />
    <a href="#documentation"><strong>Explore the docs »</strong></a>
  </p>
</p>

[![Build Status](https://app.travis-ci.com/ADVRHumanoids/modular.svg?token=zJseufwSAzkrEc1mqg8v&branch=python3)](https://app.travis-ci.com/ADVRHumanoids/modular)

<!--
[![codecov](https://codecov.io/gh/ADVRHumanoids/modular/branch/development/graph/badge.svg?token=aW77dBlb1w)](https://codecov.io/gh/ADVRHumanoids/modular)
-->

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <!-- <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul> -->
    </li>
    <li><a href="#installation">Installation</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#documentation">Docs</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#contact">Contact</a></li>
    <!-- <li><a href="#acknowledgements">Acknowledgements</a></li> -->
  </ol>
</details>

<!-- ABOUT THE PROJECT -->

## About The Project

This project focuses on the rapid generation of Tailor-Made solutions with few to many modules.

![reconfigurable_pino](https://alberobotics.it/templates/yootheme/cache/reconfigurable_pino-2e1209e8.png)

This is handled in 2 different scenarios: online or offline.

In the offline approach, a user sets up a robot with our reconfigurable modules in the virtual environment: the user selects modules from a library, assembles them toform a mechanism that is immediateley visualised in 3D.

The more pragmatic user might prefer an online approach to assemble the robot from real physical modules. This gives more immediate impression of the size and shape of the mechanism. Potentially this user also directly assembles the modules in the eventual operating environment and sees how it fits without modelling the entire workplace in CAD.

No matter which approach a user might select, the workflow is the same. Once the user has assembled a set of modules in a desired configuration, our Operating System automatically reconfigures the entire software architecture on-the-fly, generating mathematical models of both the robot kinematics and dynamics and updating the controller architecture.

<!--
### Built With:

- [Python](https://nodejs.org/en/)
- [Travis-CI](https://travis-ci.com/)
- [ROS melodic]()
- [modular](https://github.com/ADVRHumanoids/modular/)
-->
<!-- - [CodeCov](https://about.codecov.io/) -->

<!-- GETTING STARTED -->

## Installation

Currently we only support Python3 (for the last version supporting Python2 see [v.0.0.4](https://github.com/ADVRHumanoids/modular/releases/tag/v0.0.4)) and it's recommended to have pip version 21.3 or newer if you plan use editable installs (see below).

There are 3 ways to install modular:

### Option 1. **pip install**

After cloning the repo, from the main directory run:
`pip install .`
The package can be installed in system, local, or virtualenv site-packages directory.
If you plan to make modifications it and don't want to re-install it every time, install it in 'editable mode' :
`pip install -e .`

REQUIREMENTS: ros-melodic-tf package: `sudo apt update && sudo apt install ros-melodic-tf`.

### Option 2. **Catkin install**

After cloning the repo in the caktin sourcespace run:
`catkin_make install`
Similarly to the superbuild, to run the application some dependencies can be installed with:
`rosdep install modular`
while some python packages need to be installed manually:
`pip install Flask anytree protobuf_to_dict`

When the catkin workspace is sourced the app can be run from everywhere with:
`env FLASK_APP=modular.web flask run`

### Option 3. **Superbuild install**

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
`rosdep install modular`
while some python packages need to be installed manually:
`pip install Flask anytree protobuf_to_dict`

When the superbuild is sourced the app can be run from everywhere with:
`env FLASK_APP=modular.web flask run`

- **Run the simulation**

After generating the robot with the configurator, and installing the package via the superbuild:
After generating the robot with the configurator, and installing the package via the superbuild:
After generating the robot with the configurator, and installing the package via the superbuild:

```
cd $ROBOTOLOGY_ROOT/build
make modular
  make modular
make modular
```

Gazebo simulation can be run with:

`roslaunch modular modular_world.launch`

and then the plugins in the **modular_plugins** repo can be used to control the robot. For instance:

`rosservice call /Modularbot_switch 1`

########################################

**When reading configuration from hardware**

Python 3 package for Protocol Buffers need to be installed

`pip3 install protobuf`

and then the .bashrc has to be modified adding the line:

## Usage

To use modular start the python server:

```bash
python modular/src/modular/web/RobotDesignStudio.py
```

Then open <http://0.0.0.0:5000/> from a browser to acces the graphical interface.

## Documentation

Static documentation for API calls has been added and is stored in the `modular/src/modular/web/docs` directory.
It can be accessed locally as follow:

1. clone the repo and navigate to the docs folder:

   ```bash
   git clone git@github.com:ADVRHumanoids/modular.git
   cd modular/src/modular/web/docs
   ```

2. start a local python HTTP server:

   ```bash
   python -m http.server
   ```

3. open <http://0.0.0.0:8000/> from a browser

<!-- ROADMAP -->

## Roadmap

See the [open issues](https://github.com/ADVRHumanoids/modular/issues) for a list of proposed features (and known issues).

<!--See the [Roadmap kanban](https://github.com/ADVRHumanoids/modular/projects/1) for the state of the development. -->

<!-- CONTRIBUTING -->

## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<!-- TODO:LICENSE - ->
## License

Distributed under the MIT License. See `LICENSE` for more information. -->

<!-- CONTACT -->

## Contact

Alberobotics team - alberobotics@iit.it

Project Link: [https://github.com/ADVRHumanoids/modular](https://github.com/ADVRHumanoids/modular)

<!-- ACKNOWLEDGEMENTS - ->
## Acknowledgements -->

<!-- MARKDOWN LINKS & IMAGES -->
<!-- These will be used once we make the project public -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links - ->

[contributors-shield]: https://img.shields.io/github/contributors/ADVRHumanoids/modular.svg?style=for-the-badge
[contributors-url]: https://github.com/ADVRHumanoids/modular/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/ADVRHumanoids/modular.svg?style=for-the-badge
[forks-url]: https://github.com/ADVRHumanoids/modular/network/members
[stars-shield]: https://img.shields.io/github/stars/ADVRHumanoids/modular.svg?style=for-the-badge
[stars-url]: https://github.com/ADVRHumanoids/modular/stargazers
[issues-shield]: https://img.shields.io/github/issues/ADVRHumanoids/modular.svg?style=for-the-badge
[issues-url]: https://github.com/ADVRHumanoids/modular/issues
[license-shield]: https://img.shields.io/github/license/ADVRHumanoids/modular.svg?style=for-the-badge
[license-url]: https://github.com/ADVRHumanoids/modular/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png -->

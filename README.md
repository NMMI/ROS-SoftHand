![qbhand logo](http://www.qbrobotics.com/wp-content/themes/qbrobotics/img/qbhand.svg)

[qbhand](http://www.qbrobotics.com/products/qbhand/) is an **adaptable yet robust robotic hand** gripper for collaborative robotic arms and for humanoid robots. It is in scale 1:1 with the average human hand and it is similarily composed by 19 completely desensorized phalanges and a flat palm.

The specific system of tendons connected to just a single motor provides the mechanical adaptability of the grasp to the shape and softness of any object without damaging it, i.e. exploiting the intrinsic capability of soft robots to continuously deform in a huge variety of possible configurations through interaction with external items.

In addition, the single actuator fixed directly on the back of the hand barely affects the total encumbrance and weight (only 480 grams) of the device.

## Table of Contents
1. [Installation](#markdown-header-installation)
   1. [ROS Packages Installation](#markdown-header-ros-packages-installation)
   1. [Setup](#markdown-header-setup)
1. [Usage](#markdown-header-usage)
   1. [Simulated qbhand](#markdown-header-simulated-qbhand)
   1. [Physical qbhand](#markdown-header-physical-qbhand)
      1. [GUI Control](#markdown-header-1.-gui-control)
      1. [Waypoint Control](#markdown-header-2.-waypoint-control)
      1. [API Control](#markdown-header-3.-api-control)
1. [Demo Applications](#markdown-header-demo-applications)
1. [ROS Packages Overview](#markdown-header-ros-packages-overview)
1. [Support, Bugs and Contribution](#markdown-header-support)
1. [Purchase](#markdown-header-purchase)
1. [Roadmap](#markdown-header-roadmap)

>This `README` is basically a mirror of the [_qbhand_ ROS wiki](http://wiki.ros.org/Robots/qbhand) and it is supplied only for offline documentation. Please, refer to the online wiki whenever you can; it is our main and most updated reference for _qbhand_ ROS related application.

## Installation
### ROS Packages Installation
>Since you are interested in the ROS interfaces for our devices, it is assumed that you are familiar at least with the very basics of the ROS environment. If not, it might be useful to spend some of your time with [ROS](http://wiki.ros.org/ROS/Tutorials) and [catkin](http://wiki.ros.org/catkin/Tutorials) tutorials. After that, don't forget to come back here and start having fun with our nodes.

Install the _qbhand_ packages for a ROS user is straightforward. Nonetheless the following are the detailed steps which should be easy to understand even for ROS beginners:

1. Clone both the `qb_device` and `qb_hand` packages to your Catkin Workspace, e.g. `~/catkin_ws`:
   ```
   cd `~/catkin_ws/src`
   git clone --recursive git@bitbucket.org:qbrobotics/qbdevice-ros.git
   git clone git@bitbucket.org:qbrobotics/qbhand-ros.git
   ```

1. Compile the packages using `catkin`:
   ```
   cd `~/catkin_ws`
   catkin_make
   ```

1. If you were not familiar with ROS you should be happy now: everything is done! Nonetheless, if you encounter some troubles during the compilation, feel free to ask for support on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).

1. **[recommanded]** You probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. To do so, just open a terminal and execute the following command:
   ```
   sudo gpasswd -a <user_name> dialout
   ```
   where you need to replace the `<user_name>` with your current linux username. Then - **don't forget to** - logout or reboot.

>At the moment the _qbhand_ related ROS packages have been tested only on Ubuntu Xenial 16.04. We are currently working to improve the compatibility with the major distributions of linux, this requires time though. We apologize for the inconvenience and we will be glad if you report any problem encountered with not yet supported distros.

### Setup
Connect a _qbhand_ to your system is basically a matter of plugging in a USB cable. Nonetheless, read carefully the following notes:

- Connect the _qbhand_ to your system either directly through an USB cable or to an already connected _qbrobotics_ device chain.
  - **Single mode:** the hand has to be connected either directly to your ROS master host through a USB cable or with an ERNI cable to a _qbrobotics dummy board_ which in turn has to be connected to the system through a USB cable. In both cases the _qbhand_ has to be powered with another ERNI cable respectively directly or through the ``dummy board``.
  - **Chained mode:** the hand has to be connected through an ERNI cable to an already connected and powered _qbrobotics_ device chain.
    
    >Even if your _qbhand_ is connected to a _qbrobotics_ device chain, with this node you can control only the _qbhand_ specified by the given ID. In case you are wondering how to properly control many _qbrobotics_ devices simultaneously, have a look at the specific [qb_chain](http://wiki.ros.org/qb_chain) ROS package which simply groups together several [qb_hand_control](http://wiki.ros.org/qb_hand_control) and [qb_move_control](http://wiki.ros.org/qb_move_control) nodes to manage their resources efficiently.

- In the following you need to know the current device ID of your _qbhand_. If it is the first time you are using our product or simply you are not sure about it, check the device info with the [command line](https://bitbucket.org/qbrobotics/qbdevice-admin) utility. Actually the _qbhand_ ID is often `1` or `2`, but it is not always the case.

  Once installed the utility following its `README`, you should be able to retrieve the device info by executing the following command in a terminal:
   ```
   cd <qb_admin_unix_binaries_path>
   ./qbadmin -p
   ```

>Each _qbrobotics®_ device you connect to your system *must have a unique ID*.

For more details about interfacing the _qbhand_ with your system or with other _qbrobotics_ devices we recommend you to read carefully the [_qbhand_ manual](http://www.qbrobotics.com/products/qbhand/).

## Usage
### Simulated qbhand
>The simulation package is still a work in progress. The _qbhand_ will be ready to work with Gazebo as soon as possible!

### Physical qbhand
The _qbhand_ can be used in the following control modes:

#### 1. GUI Control
This control mode is the simpler and the one suggested to test that everything is working as expected. You are able to open and close the hand interactively, but nothing more than this.

##### Prerequisites
1. A _qbhand_ properly connected to your system (cf. [Setup](#markdown-header-setup)).
1. Set the current device ID in the launch file:
   1. Open the `qb_hand_control/launch/gui_control.launch`.
   1. Check if the `device_id` argument is set with ID you have annotated before.
   1. If not, simply change it with the current ID.

##### Control
To start the ROS node open a terminal and execute the following command:
```
roslaunch qb_hand_control gui_control.launch
```

If you have made any modifications at the C++ code (this does not apply for `.xml` launch files and `.yaml` configuration files), remember to recompile the the whole Catkin Workspace (cf. [ROS Packages Installation](#markdown-header-ros-packages-installation)).

After a while a GUI should appear to screen with two empty dropdown menus, a red enable button below them, and a _speed scaling_ slider at the bottom.
1. Select the _Controller Manager_ namespace from the left menu, e.g. `/qbhand/controller_manager` (where `/qbhand/` is the current device namespace, modifiable from within the launch file). This enables the right menu which provides all the controllers available for the connected device.
1. Select the _qbhand_ controller: the only option available is the single synergy controller which acts on the motor position.
   
   >Be aware that the chosen controller **must match** the `control_action` parameter set in the launch file.

1. Click the enable button (available only after the selection of the controller). A single slider will appear in the GUI to control the closure of the hand, which ranges from `0` (hand completely open) to `1` (hand completely closed).
1. Move the slider to perform a complete grasp or to partially open/close the _qbhand_. You can also vary the speed through the bottom _speed scaling_ slider if you like a faster/slower motion. No other timing constraints can be set in this mode.

If `rviz` is enabled (look at the launch file and see if `use_rviz` is set to `true`) you should see a virtual hand on screen performing a similar behavior. Be aware that the _qbhand_ is desensorized and therefore it is not possible to know exactly the position of each finger: the screen visualization is just the result of an estimation of the closure value and may differ form the physical configuration of your _qbhand_ (e.g. when grasping an object).

#### 2. Waypoint Control
This control mode is a bit more structured and useful than the previous: it allow to set a fixed trajectory of any number of position waypoints (with timing constraints) and set the robot to cycle infinitely on it (because of the loop it is recommended to set the first and last waypoint in a similar configuration to avoid unwanted sudden changes).

##### Prerequisites
1. A _qbhand_ properly connected to your system (cf. [Setup](#markdown-header-setup)).
1. Set the current device ID in the launch file:
   1. Open the `qb_hand_control/launch/waypoint_control.launch`.
   1. Check if the `device_id` argument is set with ID you have annotated before.
   1. If not, simply change it with the current ID.

##### Customization
With this mode, you can modify the waypoint trajectory to replicate the behavior you want: you can either modify the `qb_hand_control/config/qbhand_waypoints.yaml` or add another custom application-specific `<namespace>_waypoints.yaml` file in the `config` directory. In the second case you need also to modify the launch file accordingly, i.e. set `waypoint_namespace` with the chosen `<namespace>`.

The waypoint configuration is as follows: you can specify the values you want, but the names are fixed a part from `<namespace>` which is the one you need:
```
# Waypoints describe the desired motion trajectory:
#  - time [s]: can be either a single value or an interval for which joint_positions hold
#  - joint_positions:
#     - hand closure [0,1].

waypoints:
 -
   time: [1.0]
   joint_positions:
     <namespace>: [0.0]
 -
   time: [2.25, 2.75]
   joint_positions:
     <namespace>: [0.7]
 -
   ...
```

##### Control
To start the ROS node open a terminal and execute the following command:
```
roslaunch qb_hand_control waypoint_control.launch
```

If you have made any modifications at the C++ code (this does not apply for `.xml` launch files and `.yaml` configuration files), remember to recompile the the whole Catkin Workspace (cf. [ROS Packages Installation](#markdown-header-ros-packages-installation)).

You won't see any control interface in this case but the _qbhand_ should start moving according to the given trajectory.

If `rviz` is enabled (look at the launch file and see if `use_rviz` is set to `true`) you should see a virtual hand on screen performing similar behavior. Be aware that the _qbhand_ is desensorized and therefore it is not possible to know exactly the position of each finger: the screen visualization is just the result of an estimation of the closure value and may differ form the physical configuration of your _qbhand_ (e.g. when grasping an object).

#### 3. API Control
If you need a complex (i.e. real) control application, e.g. the _qbhand_ is mounted on a robot which uses computer vision aid to grasp objects, the previous two control modes don't really help much. What we provide for real applications is the full ROS libraries to manage and control the _qbhand_.

##### Prerequisites
- A _qbhand_ properly connected to your system (cf. [Setup](#markdown-header-setup)).

##### Control
You have to dig into the [qb_hand](http://wiki.ros.org/qb_hand) package documentation and find what better suite for your needs, e.g. use form within the code or extend the `qbHandControl` class provided, or even redesign it from scratch by following an approach similar to ours.

The creation of one or several launch files to start your application is up to you.

>Our recommendation is to use as much as possible our resources, classes and macros to help you while developing your application. Don't reinvent the wheel!

At last, if you come up with a something useful for the whole community, it will be amazing if you propose your improvement with a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).

## Demo Applications
TODO: add video examples (GUI, waypoints)

## ROS Packages Overview
| |Packages|
|---:|---|
|[qb_device](http://wiki.ros.org/qb_device): |[qb_device_bringup](http://wiki.ros.org/qb_device_bringup), [qb_device_control](http://wiki.ros.org/qb_device_control), [qb_device_description](http://wiki.ros.org/qb_device_description), [qb_device_driver](http://wiki.ros.org/qb_device_driver), [qb_device_hardware_interface](http://wiki.ros.org/qb_device_hardware_interface), [qb_device_msgs](http://wiki.ros.org/qb_device_msgs), [qb_device_srvs](http://wiki.ros.org/qb_device_srvs)|
|[qb_hand](http://wiki.ros.org/qb_hand): |[qb_hand_control](http://wiki.ros.org/qb_hand_control), [qb_hand_description](http://wiki.ros.org/qb_hand_description), [qb_hand_hardware_interface](http://wiki.ros.org/qb_hand_hardware_interface)|

## Support, Bugs and Contribution
Since we are not only focused on this project it might happen that you encounter some trouble once in a while. Maybe we have just forget to think about your specific use case or we have not seen a terrible bug inside our code. In such a case, we are really sorry for the inconvenience and we will provide any support you need.

To help you in the best way we can, we are asking you to do the most suitable of the following steps:

1. It is the first time you are holding a _qbhand_, or the first time you are using ROS, or even both: it is always a pleasure for us to solve your problems, but please consider first to read again the instructions above and the ROS tutorials. If you have ROS related questions the right place to ask is [ROS Answers](http://answers.ros.org/questions/).
1. You are a beginner user stuck on something you completely don't know how to solve or you are experiencing unexpected behaviour: feel free to contact us at [support+ros at qbrobotics.com](support+ros@qbrobotics.com), you will receive the specific support you need as fast as we can handle it.
1. You are quite an expert user, everything has always worked fine, but now you have founded something strange and you don't know how to fix it: we will be glad if you open an Issue in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).
1. You are definitely an expert user, you have found a bug in our code and you have also correct it: it will be amazing if you open a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS); we will merge it as soon as possible.
1. You are comfortable with _qbrobotics®_ products but you are wondering whether is possible to add some additional software features: feel free to open respectively an Issue or a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS), according to whether it is just an idea or you have already provided your solution.

In any case, thank you for using [_qbrobotics®_](http://www.qbrobotics.com) solutions.

## Purchase
If you have just found out our company and you are interested in our products, come to [visit us](http://www.qbrobotics.com) and feel free to ask for a quote.

## Roadmap
Our journey through the ROS ecosystem has just started and we have already planned new midterm features to be implemented in the following months:

- Support for the most used ROS/Ubuntu distros
- CI an tests
- Gazebo ROS simulation
- Improved controller to better replicate the real grasp

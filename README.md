# The multi_robot_sim Modular Robotic Simulation Launcher package



#### Dependencies
**multi_ridgeback_nav by @neel.nagda**

The multi_robot_sim package is dependend upon the multi_ridgeback_nav package, which can be found here: https://github.com/ROOSTER-fleet-management/multi_ridgeback_nav/tree/master

Install this package before continuing, carefully following the README.md instructions. 

**mutli_husky_nav by @neel.nagda**

This mutli_robot_sim package is dependend upon the multi_husky_nav package, which can be found here:
https://github.com/ROOSTER-fleet-management/multi_husky_nav/tree/master

Install this package before continuing, carefully following the README.md instructions.

**PyQT4**

This package requires the python module *PyQt4* to be installed:

```console
sudo apt-get install python-qt4
```

---

#### Installation
Once all dependencies have been succesfully installed you can continue with the multi_robot_sim package itself.

1. Clone the repository into your workspace src folder. 
2. Run catkin_make in the toplevel directory of your workspace.
3. Use the following rosluanch command to launch the multi_robot_sim package: 

```console
roslaunch multi_robot_sim launcher_gui.launch 
```


---


#### Usage
***The multi_robot_sim Launcher application.***<br/>
In the **Launch tab** the Launch list is managed. New robots can defined and added to a pool (Launch list). Right clicking a robot in the *Launch list* allows the user to delete it. The entire list can also be cleared with the "Clear Launch list" button.<br/>
From this tab the simulation can be launched or shutdown. A Launch status is provided in the top right.<br/>
*Note: Each Robot ID in the Launch list must be unique.* <br/>

In the **Docking stations tab** a docking station (ds) ID can be selected and it's properties adjusted. <br/>
The origin (x, y, theta) of the docking station refers to the position and orientation in the simulated world based on the /map frame.
The docking cells of a specific docking station are place in the positive x and y directions from its origin based on the provided orientation. <br/>
Cell spacing (x and y) determines the space between cell borders. Cell angle (theta) determines the orientation of the docking cells relative to the docking station's orientation.<br/>
The maximum number of robots per row and column determine the docking station's capacity and docking cell arrangement.
<br/>

Hovering over certain items in the Launcher will provide further information in the *status bar* at the bottom of the application.

More information on the Launcher and multi_robot_sim package, including license information, can be found in the *About screen* (File>About or *Ctrl+A*).

**Shortcuts**

Shortcut | Explanation
-------- | ------------
`Ctrl+N` | **N**ew setup file.
`Ctrl+S` | **S**ave the current setup.
`Ctrl+O` | **O**pen and existing setup file.
`Ctrl+A` | Show the **A**bout screen.
`Ctrl+Q` | **Q**uit the application.
`Alt+C` | **C**lear the Launch list.
`Alt+A` | **A**dd a new robot to the Launch list based on the provided input fields.
`Alt+L `| **L**aunch Gazebo and robots based on the current Launch list and docking stations.
`Alt+S `| **S**hut down the currently launched robots and Gazebo.


***dynaRviz application.***<br/>
The dynaRviz application is basically the normal RViz application except that it is launched dynamically based on the fleet as specified in the Multi Robot Sim Launcher GUI.
<br/>


---

#### Planned
The following items are planned to be implemented in the multi_robot_sim package:

- [X] Optional launching of the [Fleet Manager package](https://github.com/ROOSTER-fleet-management/rooster_fleet_manager/tree/master).
- [X] Optional launching of dynamic Rviz based on provided launch information (robots).
- [ ] Expansion of the supported robot types.

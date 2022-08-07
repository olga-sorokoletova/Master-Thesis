# Autonomous Robots as a Resource for Mass Epidemic Sustainability assisting in critical care wards

This is the repository devoted to the *Master Thesis*. 

The goal is to implement robots as acting remote interface for physicians at home in smart-working mode, to perform diagnostic tasks that do not require high precision manual interactions, and therefore decreasing the workload of physically available operators in critical care wards. 

## Compile and run the project

### Using the Manual Launch

**1. Open terminal, navigate to ```MARRTINO_APPS_HOME/docker```, and start the docker:**

```
cd $MARRTINO_APPS_HOME/docker
./start_docker.bash

```

**2. In a browser open ```http://localhost``` and follow ```Bringup``` link. Press ```CONNECT```.**

**3. Return to the terminal and run the docker image for the ```stage``` simulator as ```docker exec -it stage tmux```. Then in the opened ```tmux``` window execute the desired stage:**.

```
cd src/stage_environments/scripts
python start_simulation.py ER_planfloor

``` 

**4. Open one more terminal window and run docker image for navigation as ```docker exec -it navigation tmux a```.**

In a ```tmux``` window's bottom pannel go to:

**- 1. ```0:loc``` to start the ```localization```:**

```
cd ../navigation
python startloc.py ER_planfloor [<x> <y> <a_deg>]
```

```<x> <y> <a_deg>``` are the 0, 1 and 3 coordinates of the robot pose, respectively. See these coordinates in a ```stage``` window.

**- 2. ```1:nav``` to start the ```navigation``` itself. For example:**

```
cd ../navigation
roslaunch move_base.launch
```

But also other options are possible.

**- 3. ```4:rviz``` to launch ```rviz```:**

```
cd ../navigation
rosrun rviz rviz -d nav.rviz
```

Through ```rviz``` fix ```2D Pose Estimate```, check that the correct map is chosen and then set the navigation goal using ```2D Nav Goal```. Observe the motion in both ```rviz``` and ```stage```.

### Using the Autostart Scripts

**1. The same as before: open terminal, navigate to ```MARRTINO_APPS_HOME/docker```, and start the docker:**

```
cd $MARRTINO_APPS_HOME/docker
./start_docker.bash

```

**2. Now navigate to the ```start``` folder and launch authomatic start:**


```
cd ../start
python3 autostart.py ER_planfloor_new.yaml

```
Take care of map ```.yaml```, ```.png``` and ```include``` folder be present in the ```$HOME/playground/maps```.

## Author
- Olga Sorokoletova - 1937430

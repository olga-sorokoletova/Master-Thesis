# Autonomous Robots as a Resource for Mass Epidemic Sustainability assisting in critical care wards

This is the repository devoted to the *Master Thesis*. 

The goal is to implement robots as acting remote interface for physicians at home in smart-working mode, to perform diagnostic tasks that do not require high precision manual interactions, and therefore decreasing the workload of physically available operators in critical care wards. 

## Prerequisites

Make sure to have ```marrtino_apps``` and ```stage_environments``` in ```$HOME/src```.

## Compile and run the project

### Using the Manual Launch

**1. Open terminal, navigate to ```MARRTINO_APPS_HOME/docker```, and start the docker:**

```
cd $MARRTINO_APPS_HOME/docker
./start_docker.bash

```
As the result, docker containers must start. If it didn't happen, run ```tmux a``` and check ```marrtino up``` tab for the presence of errors.

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

## Files

### 1. The semantic map:  [ER_planfloor_new.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/map/ER_planfloor_new.yaml).
It has:
- **doors**, **indoors** and **outdoors** for the 4 top-right rooms in the map = assumed offices for 4 doctors,
- **home** = assumed location of the charging station, 
- **beds** in the corridors (since it's a property of the ED), implemented as boxes,
- **telecom** = assumed room where telecommunication with a physician at home can be performed,
- 6 categories of **people** inspired by the map from [here](https://kierantimberlake.com/updates/report-from-the-studio-mapping-jefferson-hospitals-emergency-department/).

### 2. Prototypic database: [providers.csv](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/providers.csv).
It has three fields:
- **name** of the doctor (for now, they are just "provider1", ..., "provider4"),
- **office** of the doctor ("1",..."4" in a correspondence to 4 top-right rooms in the map),
- **status** of the doctor ("home" = doctor is available in a smart-working mode today, "hospital" = doctor is working in the hospital, "NA" - doctor is not available)

### 3. Script:  [create_nav_cmd.py](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/create_nav_cmd.py).
The command to execute this script is: 

```
python create_nav_cmd.py "provider1"

```
where instead of ```"provider1"``` can be any name of the doctor who is present in the database.

It checks what is the status of the requested doctor, and:
- if he is not available => simply informs,
- otherwise => creates a file [```er_cmd.nav```](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/navigation/er_cmd.nav) inside the ```marrtino_apps/navigation``` folder to be executed as:

```
python nav.py er_cmd.nav. 

```
This file contains just one command: ```gotoLabel(target_location)```, where ```target_location``` can be: 1) "telecom", if status is "home", or 2) outdoors of the doctor's office if status is "hospital". 

## Author
- Olga Sorokoletova - 1937430

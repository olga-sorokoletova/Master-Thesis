# Enhancing Robot Reliability for Health-Care Facilities by means of Human-Aware Navigation Planning

<h3 align="center">
  This is the repository devoted to the Master Thesis 
  
  on Social (Human-Aware) Navigation of Autonomous Robots assisting in Critical Care Wards as a resource for Health-Care Sustainability. 

  The thesis was defended upon completion of the Master Course on Artificial Intelligence and Robotics in Faculty of Information Engineering, Informatics and Statistics of Sapienza University of Rome, Rome, Italy.
</h3>

## Abstract

With the aim of enabling robots to cooperate with humans, carry-out human-like tasks or navigate among humans we need to ensure that they are equipped with the ability to comprehend human behaviours and use the extracted knowledge for the intelligent decision-making. This ability is particularly important in the **safety-critical and human-centered environment** of health-care institutions. In the field of robotic navigation the most cutting-edge approaches to enhance robot reliability in the application domain of health-care facilities and in general pertain to **augmenting navigation system with human-aware properties**. To implement this in our work the [**Co-operative Human Aware Navigation (CoHAN) planner**](https://github.com/sphanit/CoHAN_Planner#co-operative-human-aware-navigation-cohan-planner) has been integrated into [**ROS-based differential-drive robot MARRtina**](https://www.marrtino.org/home) and exhaustively challenged within various simulated contexts and scenarios (mainly, modeling the situations relevant in medical domain) to draw attention to the integrated system's benefits and identify its drawbacks or instances of poor performance while exploring the scope of system capabilities and creating a full characterization of its applicability.

## Install, Compile and Run the project

### Software Description

The MARRtino robot is a low-cost nevertheless fully compatible with other expensive and professional platforms ROS-based differential-drive mobile robot that is an open hardware/software project, designed for experimentation and commonly used in education and research. The software for MARRtino exists in several forms, starting from low-level source codes and ending with a ready-to-use pre-installed on a VirtualBox Virtual Machine version. The [**marrtino\_apps**](https://bitbucket.org/iocchi/marrtino\_apps/) repository (maintained by [Prof. Luca Iocchi](https://sites.google.com/a/dis.uniroma1.it/iocchi/home?authuser=0)) is a **Docker-based** MARRtino software that was used for the purposes of this thesis. It contains ROS(kinetic/melodic)-packages, representing different robot functionalities and control/visualisation modules, distributed among **Docker images** (or, as runtime instances, **Docker containers**) and interfaced with Python and other languages.

The available images are:

- System support:
  - ```orazio``` (for the real robot)
  - ```stage\_environments``` (for the ```stage``` container)
  - ```nginx```
  - ```devrt/xserver```
- Operational:
  - ```base``` (robot base)
  - ```system```
  - ```teleop``` (teleoperation)
  - ```navigation``` and ```navigation-cohan```
  - ```mapping```
  - ```vision```
  - ```speech``` 
  - ```objrec``` (object recognition)

In particular, the ```navigation/navigation-cohan``` and ```mapping``` components are responsible for the Navigation Stack functioning. Using nomenclature of ROS, ```mapping``` contains the Mapping module, and ```navigation``` stores the executables to perform Localisation and to move robot base along the paths, provided by the Path Planning. The Path Planning itself is represented by the variety of planning approaches. They are normally implemented as external systems and pulled from the outer repositories into inherited from ```navigation``` image. Similarly, the Co-operative Human-Aware Navigation (CoHAN) planner repository is pulled to the ```navigation-cohan``` image.

The Docker containers, involved in the integration and testing of CoHAN, are the following five:
- ```navigation```
- ```base```
- ```stage```
- ```nginx```
- ```xserver```

The last two containers in the list are related to the computer setup (web server), ```base``` and ```navigation``` are the runtime instances of the ```base``` and ```navigation-cohan``` images, respectively, and ```stage``` is a simulator.

---

### System Prerequisites

**1. Linux OS.**

**2. Python and tmux**:

```
sudo apt install python tmux python3-yaml
```

**3. Docker engine (not docker Desktop) (tested on v. 19.03, 20.10).**

Usually, this should work: 

```
sudo apt install docker.io
```

or [install from binaries](https://docs.docker.com/engine/install/binaries/).

See also [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/). In particular, add your user to the ```docker``` group:

```
sudo usermod -aG docker $USER
```

Log out and log in before proceeding.

**4. Docker-compose (tested on v. 1.28).**

First, remove any other ```docker-compose``` file, if present (check with ```which docker-compose```).
Download binary file for v. 1.28.5:

```
cd /usr/local/bin
sudo wget https://github.com/docker/compose/releases/download/1.28.5/docker-compose-Linux-x86_64
sudo mv docker-compose-Linux-x86_64 docker-compose
sudo chmod a+x docker-compose
docker-compose -v
```
---

### Software Installation

**1. Download [```marrtino_apps```](https://bitbucket.org/iocchi/marrtino\_apps/) (```cohan``` branch) and [```stage_environments```](https://bitbucket.org/iocchi/stage_environments.git) repositories to the ```~/src``` folder and create ```~/playground``` folder:**

```
cd ~/src 
git clone -b cohan https://bitbucket.org/iocchi/marrtino_apps.git
git clone https://bitbucket.org/iocchi/stage_environments.git
mkdir ~/playground
```

Every time you would like to get up-to-date version, enter cloned repository and execute:

```
git pull
```

**2. Add in your ```~/.bashrc```**:

```
export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps
export MARRTINO_PLAYGROUND=$HOME/playground
export ROS_IP=127.0.0.1
export ROBOT_TYPE=stage
```

Open a new terminal (to make ```.bashrc``` changes effective).

**3. Build docker images:**

```
cd $MARRTINO_APPS_HOME/docker
./docker_build_cohan.bash
```

It is fine, if there are some errors during execution of the ```./docker_build_cohan.bash```, but if you encounter problems with further steps, then try to build ```./docker_build.bash``` first and ```./docker_build_cohan.bash``` $-$ after.

*Note*: ```./docker_build_cohan.bash``` builds only the docker images needed to replicate navigaion tests on stage, ```./docker_build.bash``` $-$  also other images.

**4. Edit system configuration file as in [system_config.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/system_config.yaml):**

```
cd $MARRTINO_APPS_HOME
cp docker/system_config_template.yaml system_config.yaml
nano system_config.yaml 
```

If you prefer, set the ```stage``` field ```on``` instead of ```vnc```. This will use the X11 server on your host, instead of VNC through a browser.
For the ```vnc``` option, GUI is accessible at [```http://localhost:3000```](http://localhost:3000).

---

### System Start and Stop

**To start the system, open the terminal, navigate to ```MARRTINO_APPS_HOME/docker```, and start the docker:**:

```
cd $MARRTINO_APPS_HOME/docker
./start_docker.bash
```

Check with ```docker ps``` that docker containers as in the image below are running. If it didn't happen, run ```tmux a``` and look up a ```marrtino up``` tab to see errors.

<p align="center">
  <img src="./readme imgs/running containers.png" width="907" height="186"/>
</p>

When containers started, it becomes possible to access them by executing: 

```
docker exec -it <container_name> tmux [a].
```

**To stop the system after tests are done, execute:**

```
cd $MARRTINO_APPS_HOME/docker
./stop_docker.bash
```

---

### Navigation Stack Launch

**1. Start the docker.**

**2. If in ```system_config.yaml``` the ```vnc``` is chosen, open ```http://localhost:3000``` in browser, otherwise, open ```http://localhost```, follow ```Bringup``` link and press ```CONNECT```.**
 
**3. Run a stage environment.** 

Two alternative options are available:

- From host OS:

  ```
  docker exec -it stage bash -ci "roscd stage_environments/scripts && python start_simulation.py <map_name>"
  ```

- By accessing the ```stage``` container: 

  ```
  docker exec -it stage tmux
  ```

  Then in the opened ```tmux``` window navigate to the folder with scripts and execute the stage with the desired map:
  
  ```
  cd src/stage_environments/scripts
  python start_simulation.py <map_name>
  ```

In both cases ```<map_name>``` is a name of the yaml file forming the semantic map of the corresponding test. 

E.g. to recreate a *Visibility Test: A human in open space*, execute:

```
python start_simulation.py ICU_visibility_open_space
```

Stage sources are ```map_name.yaml```, ```map_name.png``` (or another image extension) and ```include``` folder. Relevant locations to store these files are:

- ```~/playground/maps```
- ```~/src/marrtino_apps/mapping/maps```
- ```~/src/stage_environments/maps```

The full set of maps used for testing is stored in this reposiory in [```playground/maps```](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/maps).

Expected result with ```ICU``` map:

<p align="center">
  <img src="./readme imgs/icu_mini.png" width="797" height="524"/>
</p>

To quit the simulation, use:

```
rosrun stage_environments quit.sh
```

**4. Start modules.**

Each command must be given in a new terminal or in a new ```tmux``` tab.
To create new ```tmux``` tabs, use the keys ```CTRL-b c```.

**4.1. Start the localization:**

 - From host OS:
    
 ```
 docker exec -it navigation bash -ci "cd \$MARRTINO_APPS_HOME/navigation && python startloc.py <map_name> [<x> <y> <a_deg>]"
 ```

 - By accessing the ```navigation``` container: 
    
 ```
 docker exec -it navigation tmux a
 ```

 In the opened ```tmux``` window's bottom panel go to ```0:loc``` and execute: 
    
 ```
 cd ../navigation
 python startloc.py <map_name> [<x> <y> <a_deg>]
 ```

In both cases  ```<map_name>``` is the same as before, and  ```<x> <y> <a_deg>``` are the 0th, 1st and 3rd map coordinate of the robot's initial pose, respectively. They are printed out in the log of terminal with running ```stage```. When using a ```stage``` simulator, initial map coordinates are retrieved by the ```stage``` ground truth: in this case, it is not necessary to provide them. Otherwise, you need to provide the actual initial pose of the robot for localization.

E.g. for ```ICU_visibility_open_space``` map in ```stage``` simulator, you can use:

```
python startloc.py ICU_visibility_open_space
```

**4.2. Start the human tracking:**

[```stage/humans_bridge.py```](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/stage/humans_bridge.py) is responsible execitable.

 - From host OS:
    
 ```
 docker exec -it navigation bash -ci "cd \$MARRTINO_APPS_HOME/stage && python humans_bridge.py <nh>"
 ```

 - In the ```navigation``` container ```2:obst``` tab: 
    
 ```
 cd ../stage
 python humans_bridge.py <nh>
 ```
 
In both cases ```<nh>``` is a number of humans in the semantic map.

E.g. for *Visibility Test: A human in open space* (currently uploaded semantic map has two humans, so for this test comment a line defining one of them in it):

```
python humans_bridge.py 1
```

To check that ```/tracked_humans``` topic is being published, run from another tab:

```
rostopic echo /tracked_humans
```

**4.3. Start the ```move_base``` node:**

 - From host OS:
    
 ```
 docker exec -it navigation bash -ci "cd \$MARRTINO_APPS_HOME/navigation && roslaunch cohan_nav.launch"
 ```

 - In the ```navigation``` container ```1:nav``` tab: 
    
 ```
 cd ../navigation
 roslaunch [move_base.launch | cohan_nav.launch]
 ```
 
```cohan_nav.launch``` is an adapted ```move_base``` node for [**Co-operative Human Aware Navigation (CoHAN) Planner**](https://github.com/sphanit/CoHAN_Planner#co-operative-human-aware-navigation-cohan-planner), ```move_base.launch``` is referred in quantitative analysis as Simple Move Base (SMB). You can also use other ```launch``` files here.


**4.4. Start visualisation in RViz:**

 - From host OS:
    
 ```
 docker exec -it navigation bash -ci "cd \$MARRTINO_APPS_HOME/navigation && rosrun rviz rviz -d nav_cohan.rviz"
 ```

 - In the ```navigation``` container ```4:rviz``` tab: 
    
 ```
 cd ../navigation
 rosrun rviz rviz -d nav_cohan.rviz
 ```
 
In RViz fix ```2D Pose Estimate``` and send the navigation goal using ```2D Nav Goal```. Observe the motion in both RViz and ```stage```.

## Compile and run the project

### Using the Manual Launch

### Using the Autostart Scripts

**1. The same as before: open terminal, navigate to ```MARRTINO_APPS_HOME/docker```, and start the docker:**

```
cd $MARRTINO_APPS_HOME/docker
./start_docker.bash
```

**2. Now navigate to the ```start``` folder and launch authomatic start:**


```
cd ../start
python3 autostart.py ER_start.yaml
```
Take care of map ```.yaml```, ```.png``` and ```include``` folder be present in the ```$HOME/playground/maps```.

To stop the modules inside the containers (but not the containers):

```
python3 autostart.py ER_start.yaml --kill
```

## PROGRAMMING MARRTINO

## 1. Semantic navigation 

### 1. The semantic maps:  for the Emergency Department floor - [ER_planfloor_new.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/maps/ER_planfloor_new.yaml), for the ICU (Intensive Care Unit) - [ICU.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/maps/ICU.yaml), [ICU2.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/maps/ICU2.yaml) and [sample_new.yaml](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/playground/maps/sample_new.yaml).

Semantic information:
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
This script is executed outside the containers, and the command to execute this script is: 

```
cd ~/playground/
python3 create_nav_cmd.py "provider1"

```
where instead of ```"provider1"``` can be any name of the doctor who is present in the database.

It checks what is the status of the requested doctor, and:
- if he is not available => simply informs,
- otherwise => creates a file [```er_cmd.nav```](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/navigation/er_cmd.nav) inside the ```marrtino_apps/navigation``` folder to be executed as:

```
python nav.py er_cmd.nav. 

```
This file contains just one command: ```gotoLabel(target_location)```, where ```target_location``` can be: 1) "telecom", if status is "home", or 2) outdoors of the doctor's office if status is "hospital". 

## 2. Getting and setting poses for humans in the stage 

### 1. Using ROS

Display the list of topics in one of the free tabs of navigation container while running navigation:
```
rostopic list
```
5 topics are being published for each of the humans. For example, for the human named in a semantic map as ```human1``` their names are:
```
/human1/base_pose_ground_truth
/human1/cmd_vel
/human1/odom
/human1/setpose
/human1/stage_say
```
To listen to a particular topic, use:
```
rostopic echo /topic_name

#or to read just one message
rostopic echo -n 1 /topic_name
```
To know the type of the message, use:
```
rostopic type /topic_name
```
To publish message to a topic, use publisher:
```
rostopic pub /topic_name /topic_type  /message
```
To move a human, publish to its ```/cmd_vel``` topic message with a new velocity. For example (use double tab to autocomplete the message structure):
```
rostopic pub /human1/cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
As the result, observe the ```human1``` moving forward.

### 2. Using Python

Use one of the free tabs of navigation container while running navigation to execute the scripts, mentioned below.

2.1. To get the pose of the ```human1```:
```
python getpose.py human1
```
2.2. To set the pose of the ```human1```:
```
python setstagepose.py <x_new> <y_new> <a_deg_new> human1
```
As the result, ```human1```'s pose is changed accordingly.

2.3. To move human continuously along ```x``` axis using custom script [move_obstacle.py](https://github.com/olga-sorokoletova/Master-Thesis/blob/main/navigation/move_obstacle.py):

```
python move_obstacle.py human1 <x_human1> <y_human1> <a_deg_human1> <x_new_human1>
```
```<x_human1> <y_human1> <a_deg_human1>``` are the initial coordinates of the ``human1`` pose, and ```<x_new_human1>``` is a desired coordinate of the ```human1``` on ```x``` axis.

## Author
- Olga Sorokoletova - 1937430

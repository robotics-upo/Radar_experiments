# Experiments Radar-Aruco to avoid Smoke

This repo contains some configuration files to use in the research of smoke avoidance navigation and localization.
To download bag files to use with these launch files go to the dropbox folder:

[DropBox Bag Folder](https://www.dropbox.com/sh/kt2q8ulnx4171dk/AACkiA1Xx5g9ah2zWdHbek7Fa?dl=0)

[DropBox MultiRadar Bag Folder](https://www.dropbox.com/sh/k868avekgrstwzs/AADj7J1sI7gRccPbI_15fMdOa?dl=0)

By **default** the launch ```make_aruco_map.launch``` is configured to avoid using the /tf and /tf_static topics in the case they are present on the bag file. So if you need to use these topics because some reasons, just run the launch file with the arg *use_bag_tf* set to true.

Of course, to use this launch you will need to download some ROS Packages such as [MCL3D](https://github.com/robotics-upo/mcl3d/) [Fiducials](https://github.com/robotics-upo/fiducials) and [Odom To Tf](https://github.com/robotics-upo/odom_to_tf). Just follow the instructions on every package, clone them into your catkin workspace an compile.

## Localize your robot

The launch file come with a default position in the attached map, but it happens that some bags are recorded from a different start position, so you will need to initially localize them, using "P" key in the RViz window.

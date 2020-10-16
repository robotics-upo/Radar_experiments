# Experiments Radar-Aruco to avoid Smoke

This repo contains some configuration files to use in the research of smoke avoidance navigation and localization.
To download bag files to use with these launch files go to the dropbox folder:

[DropBox Bag Folder](https://www.dropbox.com/sh/kt2q8ulnx4171dk/AACkiA1Xx5g9ah2zWdHbek7Fa?dl=0)

By **default** the launch ```make_aruco_map.launch``` is configured to avoid using the /tf and /tf_static topics in the case they are present on the bag file. So if you need to use these topics because some reasons, just run the launch file with the arg *use_bag_tf* set to true.
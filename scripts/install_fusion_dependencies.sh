#!/bin/bash
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}-- Installing apt depends: AMCL, PointCloud to LaserScan ...${NC}"
sudo apt update 
sudo apt install ros-"$ROS_DISTRO"-pointcloud-to-laserscan ros-"$ROS_DISTRO"-amcl ros-"$ROS_DISTRO"-map-server -y

echo -e "${YELLOW}-- Clonning Dependencies Repos ...${NC}"
read -p "Enter your catkin workspace location, ej: /home/user/catkin_ws: " catkin_path

cd "$catkin_path"/src || (echo "ERROR: Couldn't go to the catkin path: $catkin_path " && exit )
# Cloud concatenator, odom to tf, nix driver

echo -e "${YELLOW}    ---- Clonning NIx Driver ...${NC}"
git clone https://github.com/robotics-upo/NIx_driver -b radar_experiments || ( echo "Stashing your changings and checking out to radar experiments branch... "  && cd NIx_driver && git stash && git checkout radar_experiemnts && git pull && cd ..)
echo -e "${YELLOW}    ---- Clonning Odom to tf  ...${NC}"
git clone https://github.com/robotics-upo/odom_to_tf -b develop           || ( echo "Stashing your changings and checking out to develop branch... " && cd odom_to_tf && git stash && git checkout develop && git pull && cd ..)
echo -e "${YELLOW}    ---- Clonning Cloud Concatenator  ...${NC}"
git clone https://github.com/robotics-upo/cloud_concatenator              || ( echo "Stashing your changes and pulling latest commit... " && cd cloud_concatenator && git stash && git pull && cd .. )
echo -e "${YELLOW}    ---- Clonning UPO Markers  ...${NC}"
git clone https://github.com/robotics-upo/upo_markers                     || ( echo "Stashing your changes and pulling latest commit... "  && cd upo_markers && git stash && git pull && cd .. )
echo -e "${YELLOW}    ---- Clonning Fiducials  ...${NC}"
git clone https://github.com/robotics-upo/fiducials                     || ( echo "Stashing your changes and pulling latest commit... "  && cd upo_markers && git stash && git pull && cd .. )

read -p "-- Installing dependencies of fiducials: Installing G2O. Proceed (y/N) (enter not if you have already installed G2O in your system)" -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    "$catkin_path"/src/fiducials/fiducial_slam/scripts/g2o_installation.sh
fi

echo -e "${YELLOW}-- Repositories cloned, compiling...${NC}"

cd ..
catkin_make

echo -e "${YELLOW}-- Next step is to download bags from dropbox. You need at least 20 GB of disk space ...${NC}"
path="radar_fusion_bags"

read -p "Do you want to continue? (y/N)" -n 1 -r
echo 
echo $REPLY
if [[ $REPLY =~ [Yy] ]]; then
    echo -e "${YELLOW}-- Starting bags download${NC}"
    mkdir ~/"$path"
    cd ~/"$path" || (echo "ERROR: Couldn't go to downlaod location: ~/$path " && exit )
    wget -O radar_fusion_bags.zip https://www.dropbox.com/sh/alo422a13irluy1/AADyN_DxhLUYeS6uhpSHvDYWa?dl=1
    unzip radar_fusion_bags.zip
    rm radar_fusion_bags.zip
else
    echo -e "${YELLOW}-- Skipping bags download${NC}"
fi

echo -e "${YELLOW}-- Dependencies installation script finished${NC}"

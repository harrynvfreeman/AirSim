# Welcome to SkeletalMeshAirSim

This is a fork off of https://github.com/rajat2004/AirSim/commits/pr/2472_rebased, commit c0bb4f90.

This assumes you have followed the setup described in https://bitbucket.org/castacks/core_central/src/official_airsim/ and have Unreal, forest, and core_central installed and working.

## How to run

### Setup AirSim
If you have existing an existing airsim directory, move or delete
```
git clone https://github.com/harrynvfreeman/AirSim.git
cd AirSim
git checkout SkeletalMeshAirSim
./setup.sh
./build.sh
cd ros
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
source devel/setup.bash
```

### Update forest plugins
```
cd ~/forest
rm -rf Plugins
ln -s ~/AirSim/Unreal/Environments/Blocks/Plugins ./Plugins
```

### Build blocks project
Open Unreal, browse and select AirSim/Unreal/Environments/Blocks/Blocks.uproject.  If you get prompted for incompatible version, select in-place conversion under more options.  Build.  When the build is complete, close unreal.


### Start forest
Start forest by openning Unreal and selecting forest/forest.uproject (build in place again if prompted) or setting an alias and calling that.
```
alias ue2='$HOME/UnrealEngine/Engine/Binaries/Linux/UE4Editor $HOME/forest/forest.uproject'
```
Push play.

### Start the mesh_info node
Open up a new terminal and run
```
cd ~/AirSim/ros
mon launch core_central airsim_example_multi.launch
```
Verify the node starts okay and that you can see /mesh_vertices when running rostopic list.

### Run RVIZ
Open up a new terminal and run rviz. Under displays in Global Options confirm that Fixed Frame is "map".  Push add, select By Topic, and choose /mesh_vertices PointCloud2.  The skeletal mesh should now display (it may require some moving and zooming to locate). 




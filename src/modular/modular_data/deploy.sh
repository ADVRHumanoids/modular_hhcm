#!/bin/bash

RED='\033[0;31m'
PURPLE='\033[0;35m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
YELLOW='\033[1;33m'
NC='\033[0m'

end_exec(){
    # end execution cleanly regardless of the scripe being sourced or not
    [[ "${BASH_SOURCE[0]}" != "${0}" ]] && return || exit 1
}

print_help () {
    printf "${ORANGE}"
    echo "Try something like:                         ./deply.sh modular_ros_pkg"
    echo "You can choose the destination folder with: -d /path/to/destination/dir"
    echo "You can choose the framework to use with: -f {ros_control} - {xbotcore} - {xbot2}"
    echo "You can add vebosity with:                  -v"
    printf "${NC}"
    end_exec
}

if [ -n "$ROBOTOLOGY_ROOT" ]; then
    export DESTINATION_FOLDER=$ROBOTOLOGY_ROOT/robots
else
    export DESTINATION_FOLDER="$HOME/albero_xbot2_ws/robots"
fi

# read arguments
while test $# -gt 0
do
    case "$1" in
        -[vV] | --verbose) # add verbosity
            VERBOSITY='-v'
            printf "Verbose mode ${GREEN}ON${NC}\n"
            ;;
        -[dD] | --destination-folder) # add destination folder
            shift
            case "$1" in
                -*)
		            printf "${RED}Path ('$1')  is not valid!${NC}\n"
       		        print_help
                    ;;
                *)  export DESTINATION_FOLDER=$1;
                ;;
            esac
            ;;
        -[fF] | --framework) # add destination folder
            shift
            case "$1" in
                -*)
		            printf "${RED}Path ('$1')  is not valid!${NC}\n"
       		        print_help
                    ;;
                *)  export FRAMEWORK=$1;
                ;;
            esac
            ;;
        -[hH] | --help) # Print suggestions
            echo "Help incoming:";
            print_help
            ;;
        -*) echo "bad option $1"
	        print_help
            ;;
        *)  if [ -n "$package_name" ]; then
		        printf "${RED}Package name inserted twice!${NC}\n"
       		    print_help
	        else
		        package_name="$1"
	        fi
            ;;
    esac
    shift
done

# check correctness of input
if ! [ -n "$package_name" ]; then
	printf "${RED}No package name argument passed!${NC}\n"
    print_help
else
    if ! [ -n "$DESTINATION_FOLDER" ]; then
        printf "${RED}No destination fodler specified!${NC}\n"
        print_help
    else
        printf "${GREEN}Deploying package ${YELLOW}${package_name}${GREEN} into ${YELLOW}$DESTINATION_FOLDER${NC}\n"
    fi
fi

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))

mkdir -p $DESTINATION_FOLDER/${package_name}
pushd $DESTINATION_FOLDER/${package_name} > /dev/null #hide print

# Deploy ROS package info
# - package.xml
cp $SCRIPT_ROOT/package.xml ./package.xml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./package.xml
# - CMakeLists.txt
cp $SCRIPT_ROOT/CMakeLists.txt ./CMakeLists.txt $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./CMakeLists.txt
printf "${GREEN}[1/9] Deployed ROS package config files${NC}\n"

## Deploy XbotCore configs
## - Low level config
#mkdir -p ./configs
#mkdir -p ./configs/hal
#cp /tmp/ModularBot/configs/ModularBot.yaml ./configs/ModularBot.yaml $VERBOSITY || end_exec
#sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./configs/ModularBot.yaml
# - High level config
#cp $SCRIPT_ROOT/configs/ModularBot_xbot1.yaml ./ModularBot_basic.yaml $VERBOSITY || end_exec
#sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./ModularBot_basic.yaml
#printf "${GREEN}[2/9] Deployed XBotCore configs${NC}\n"

# Deploy Xbot2 configs
mkdir -p ./config
mkdir -p ./config/hal
mkdir -p ./config/joint_config

# - Low level hal configs
#   - ec_all (modified)
cp /tmp/ModularBot/config/hal/ModularBot_ec_all.yaml ./config/hal/ModularBot_ec_all.yaml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./config/hal/ModularBot_ec_all.yaml
#   - gz (not modified)
cp $SCRIPT_ROOT/configs/low_level/hal/ModularBot_gz.yaml ./config/hal/ModularBot_gz.yaml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./config/hal/ModularBot_gz.yaml
#   - dummy (not modified)
cp $SCRIPT_ROOT/configs/low_level/hal/ModularBot_dummy.yaml ./config/hal/ModularBot_dummy.yaml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./config/hal/ModularBot_dummy.yaml

# - Low level joint configs
cp -TRf /tmp/ModularBot/config/joint_config ./config/joint_config $VERBOSITY || end_exec

# - High level config
cp $SCRIPT_ROOT/configs/ModularBot_xbot2.yaml ./config/ModularBot.yaml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./config/ModularBot.yaml
printf "${GREEN}[2/9] Deployed XBot2 configs${NC}\n"

# Deploy cartesio config
mkdir -p ./cartesio
# - /ModularBot_cartesio_config.yaml
cp /tmp/ModularBot/cartesio/ModularBot_cartesio_config.yaml ./cartesio/ModularBot_cartesio_config.yaml $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./cartesio/ModularBot_cartesio_config.yaml
printf "${GREEN}[3/9] Deployed cartesio configs${NC}\n"

# Deploy launch files
mkdir -p ./launch
# - cartesio.launch
cp $SCRIPT_ROOT/launch/cartesio.launch ./launch/ModularBot_cartesio.launch $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./launch/ModularBot_cartesio.launch
# - ModularBot_ik.launch
cp $SCRIPT_ROOT/launch/ModularBot_ik.launch ./launch/ModularBot_ik.launch $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./launch/ModularBot_ik.launch
# - ModularBot_sliders.launch
cp $SCRIPT_ROOT/launch/ModularBot_sliders.launch ./launch/ModularBot_sliders.launch $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./launch/ModularBot_sliders.launch
# - gazebo.launch
cp $SCRIPT_ROOT/launch/ModularBot_gazebo.launch ./launch/ModularBot_gazebo.launch $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./launch/ModularBot_gazebo.launch
printf "${GREEN}[4/9] Deployed ModularBot launch files${NC}\n"

# TODO: fix moveit deploy
## Deploy moveit_config
#mkdir -p ./moveit_config
#cp -TRf $SCRIPT_ROOT/moveit_config ./moveit_config $VERBOSITY || end_exec
#cp -TRf /tmp/ModularBot/moveit_config ./moveit_config $VERBOSITY || end_exec
## MoveIt launch files
#cp /tmp/ModularBot/launch/ros_controllers.launch ./launch/ros_controllers.launch $VERBOSITY || end_exec
#cp -TRf $SCRIPT_ROOT/moveit_launch ./launch $VERBOSITY || end_exec
#printf "${GREEN}[4.5/9] Deployed moveit configs and launch files${NC}\n"

# Deply meshes
mkdir -p -p ./database/${package_name}_fixed_base
cp -TRf $SCRIPT_ROOT/../web/static/models ./database $VERBOSITY || end_exec
printf "${GREEN}[5/9] Deployed meshes${NC}\n"

# Deploy joint_map
mkdir -p ./joint_map
cp /tmp/ModularBot/joint_map/ModularBot_joint_map.yaml ./joint_map/ModularBot_joint_map.yaml $VERBOSITY || end_exec
printf "${GREEN}[6/9] Deployed joint map${NC}\n"

# Deploy SRDF
mkdir -p ./srdf
cp /tmp/ModularBot/srdf/ModularBot.srdf ./srdf/ModularBot.srdf $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./srdf/ModularBot.srdf
printf "${GREEN}[7/9] Deployed SRDF${NC}\n"

# Deploy URDF
mkdir -p ./urdf
cp /tmp/ModularBot/urdf/ModularBot.urdf ./urdf/ModularBot.urdf $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./urdf/ModularBot.urdf
sed -i -e "s+/tmp/ModularBot+package://${package_name}+g" ./urdf/ModularBot.urdf
sed -i -e "s+package://modular/src/modular/web/static/models/modular/meshes+package://${package_name}/database/modular/meshes+g" ./urdf/ModularBot.urdf
printf "${GREEN}[8/9] Deployed URDF${NC}\n"

# Deploy gazebo model
# - modular_world.sdf
cp $SCRIPT_ROOT/database/ModularBot_fixed_base/ModularBot_world.sdf ./database/${package_name}_fixed_base/${package_name}_world.sdf $VERBOSITY || end_exec
# - manifest.xml
cp $SCRIPT_ROOT/database/ModularBot_fixed_base/manifest.xml ./database/${package_name}_fixed_base/manifest.xml $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./database/${package_name}_fixed_base/manifest.xml
# - model.config
cp $SCRIPT_ROOT/database/ModularBot_fixed_base/model.config ./database/${package_name}_fixed_base/model.config $VERBOSITY || end_exec
sed -i -e "s+PACKAGE_NAME+${package_name}+g" ./database/${package_name}_fixed_base/model.config
# - ModularBot.sdf
gz sdf --print \
    $DESTINATION_FOLDER/${package_name}/urdf/ModularBot.urdf > \
    $DESTINATION_FOLDER/${package_name}/database/${package_name}_fixed_base/${package_name}.sdf \
    || end_exec
printf "${GREEN}[9/9] Deployed gazebo model${NC}\n"

# All done
popd > /dev/null #hide print
printf "\n${GREEN}[ \xE2\x9C\x94 ] Package ${YELLOW}${package_name}${GREEN} succesfully deployed into ${YELLOW}$DESTINATION_FOLDER${NC}\n\n"
unset DESTINATION_FOLDER VERBOSITY SCRIPT_ROOT RED PURPLE GREEN ORANGE YELLOW NC package_name

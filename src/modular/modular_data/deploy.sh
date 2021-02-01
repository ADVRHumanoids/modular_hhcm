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
    echo "You can add vebosity with:                  -v"
    printf "${NC}"
    end_exec
}

if [ -n "$ROBOTOLOGY_ROOT" ]; then
    export DESTINATION_FOLDER=$ROBOTOLOGY_ROOT/robots
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

mkdir -p $DESTINATION_FOLDER/${package_name}
pushd $DESTINATION_FOLDER/${package_name} > /dev/null #hide print

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))

# Deploy ROS package info
# - package.xml
cp $SCRIPT_ROOT/ModularBot/package.xml ./package.xml $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./package.xml
# - CMakeLists.txt
cp $SCRIPT_ROOT/ModularBot/CMakeLists.txt ./CMakeLists.txt $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./CMakeLists.txt
printf "${GREEN}[1/9] Deployed ROS package config files${NC}\n"

# Deploy Xbot2 configs
# - Low level config
mkdir -p ./configs
cp /tmp/modular/configs/ModularBot.yaml ./configs/${package_name}.yaml $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./configs/${package_name}.yaml
# - High level config
cp $SCRIPT_ROOT/ModularBot/ModularBot_basic.yaml ./${package_name}_basic.yaml $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./${package_name}_basic.yaml
printf "${GREEN}[2/9] Deployed XBot2 configs${NC}\n"

# Deploy cartesio config
mkdir -p ./cartesio
# - cartesio.launch
cp $SCRIPT_ROOT/ModularBot/cartesio/cartesio.launch ./cartesio/cartesio.launch $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./cartesio/cartesio.launch
# - /ModularBot_cartesio_config.yaml
cp /tmp/modular/cartesio/ModularBot_cartesio_config.yaml ./cartesio/${package_name}_cartesio_config.yaml $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./cartesio/${package_name}_cartesio_config.yaml
printf "${GREEN}[3/9] Deployed cartesio configs${NC}\n"

# Deploy launch files
mkdir -p ./launch
# - ModularBot_ik.launch
cp $SCRIPT_ROOT/ModularBot/launch/ModularBot_ik.launch ./launch/${package_name}_ik.launch $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./launch/${package_name}_ik.launch
# - ModularBot_sliders.launch
cp $SCRIPT_ROOT/ModularBot/launch/ModularBot_sliders.launch ./launch/${package_name}_sliders.launch $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./launch/${package_name}_sliders.launch
printf "${GREEN}[4/9] Deployed ${package_name}_ik.launch and ${package_name}_sliders.launch${NC}\n"

# Deply meshes
mkdir -p -p ./database/${package_name}_fixed_base
cp -TRf $SCRIPT_ROOT/../web/static/models ./database $VERBOSITY || end_exec
printf "${GREEN}[5/9] Deployed meshes${NC}\n"

# Deploy joint_map
mkdir -p ./joint_map
cp /tmp/modular/joint_map/ModularBot_joint_map.yaml ./joint_map/${package_name}_joint_map.yaml $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./joint_map/${package_name}_joint_map.yaml
printf "${GREEN}[6/9] Deployed joint map${NC}\n"

# Deploy SRDF
mkdir -p ./srdf
cp /tmp/modular/srdf/ModularBot.srdf ./srdf/${package_name}.srdf $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./srdf/${package_name}.srdf
printf "${GREEN}[7/9] Deployed SRDF${NC}\n"

# Deploy URDF
mkdir -p ./urdf
cp /tmp/modular/urdf/ModularBot.urdf ./urdf/${package_name}.urdf $VERBOSITY || end_exec
sed -i -e "s+ModularBot+${package_name}+g" ./urdf/${package_name}.urdf
printf "${GREEN}[8/9] Deployed URDF${NC}\n"

# Deploy gazebo model
# - modular_world.sdf
cp $SCRIPT_ROOT/ModularBot/database/ModularBot_fixed_base/ModularBot_world.sdf ./database/${package_name}_fixed_base/${package_name}_world.sdf $VERBOSITY || end_exec
# - manifest.xml
cp $SCRIPT_ROOT/ModularBot/database/ModularBot_fixed_base/manifest.xml ./database/${package_name}_fixed_base/manifest.xml $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./database/${package_name}_fixed_base/manifest.xml
# - model.config
cp $SCRIPT_ROOT/ModularBot/database/ModularBot_fixed_base/model.config ./database/${package_name}_fixed_base/model.config $VERBOSITY || end_exec
sed -i -e "s+[package_name]+${package_name}+g" ./database/${package_name}_fixed_base/model.config
# - ModularBot.sdf
gz sdf --print \
    $DESTINATION_FOLDER/${package_name}/urdf/${package_name}.urdf > \
    $DESTINATION_FOLDER/${package_name}/database/${package_name}_fixed_base/${package_name}.sdf \
    || end_exec
printf "${GREEN}[9/9] Deployed gazebo model${NC}\n"

# All done
popd > /dev/null #hide print
printf "\n${GREEN}[ \xE2\x9C\x94 ] Package ${YELLOW}${package_name}${GREEN} succesfully deployed into ${YELLOW}$DESTINATION_FOLDER${NC}\n\n"
unset DESTINATION_FOLDER VERBOSITY SCRIPT_ROOT RED PURPLE GREEN ORANGE YELLOW NC package_name

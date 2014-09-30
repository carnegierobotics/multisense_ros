#! /bin/bash



usage() 
{
cat <<EOF
usage: $0 <build_system>

Where <build_system> is catkin or rosbuild

EOF
}


CATKIN="catkin"
ROSBUILD="rosbuild"

#
# Verify that all required utility commands exist

hash rm       2>&- || { echo >&2 "rm not installed.  Aborting.";     exit 1; }
hash find     2>&- || { echo >&2 "ls not installed.  Aborting.";     exit 1; }
hash cp       2>&- || { echo >&2 "ls not installed.  Aborting.";     exit 1; }

#
# Verify command line arguments
if [ $# -ne 1 ] ; then
    usage
    exit 3
fi

#
# Get the root directory where the script is located
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#
# Check to make sure a build_config directory exists in our root directory
if [ ! -d "$ROOT/build_config" ] ; then
    echo " Unable to locate $ROOT/build_config. Please check the Multisense Driver Installation"
    exit 1
fi

#
# Function which deletes all the unwanted files from the ROS source tree
cleanup()
{
    FILES="$( cd $ROOT/build_config/$1/ && find . -type f -name "*_")"
    for f in $FILES
    do
        FILE=$ROOT/$f
        # 
        # Check to make sure the file exists in our ROS directory
        if [ -f "${FILE%?}" ] ; then
            rm -r "${FILE%?}"
        fi
    done
}

#
# Function which copies all the files from the build_config directory
# To the ROS source tree
copy()
{
    FILES="$( cd $ROOT/build_config/$1/ && find . -type f -name "*_")"
    for f in $FILES
    do
        FILE=$ROOT/$f
        cp $ROOT/build_config/$1/$f "${FILE%?}"
    done
}

#
# Function which checks for specific files associated with a ros distribution
# by looking for a .<rosdistro> extension. If a file matches the users
# current ROS distro make it the default file
preprocess()
{
    #
    # First copy the .default files to the main file
    FILES="$( cd $ROOT/build_config/$1/ && find . -type f -name "*.default")"
    for f in $FILES
    do
        STRIPPED_NAME="$( echo $f | rev | cut -c 9- |rev)"
        cp $ROOT/build_config/$1/$f $ROOT/build_config/$1/$STRIPPED_NAME
    done

    #
    # Next copy the .<rosdistro> files to the main file
    FILES="$( cd $ROOT/build_config/$1/ && find . -type f -name "*.$ROS_DISTRO")"
    for f in $FILES
    do
        LENGTH="$( echo .$ROS_DISTRO | wc -c)"
        STRIPPED_NAME="$( echo $f | rev | cut -c $LENGTH- |rev)"
        cp $ROOT/build_config/$1/$f $ROOT/build_config/$1/$STRIPPED_NAME
    done
}


BUILDSYSTEM=$1

case $BUILDSYSTEM in

$CATKIN)   cleanup $ROSBUILD
           preprocess $CATKIN
           copy $CATKIN
           exit 0
           ;;
$ROSBUILD) cleanup $CATKIN
           preprocess $ROSBUILD
           copy $ROSBUILD
           exit 0
           ;;

*)         echo "Unknown option $1."
           usage
           exit 1
           ;;
esac


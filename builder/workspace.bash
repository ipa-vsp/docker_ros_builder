#!/bin/bash

set -e
set -o pipefail
function builder_setup {
    apt_get_install python3-colcon-common-extensions python3-catkin-pkg python3-pip
    python3 -m pip install catkin_pkg
}
function grep_opt {
    grep "$@" || [[ $? = 1 ]]
}
function update_list {
    local ws=$1; shift
    setup_rosdep
    if [ -f $ws/src/extra.sh ]; then
        $ws/src/extra.sh
    fi
}
function read_depends {
    local src=$1; shift
    for dt in "$@"; do
        grep_opt -rhoP "(?<=<$dt>)[\w-]+(?=</$dt>)" "$src"
    done
}
function list_packages {
    if [ "$ROS_VERSION" -eq 1 ]; then
        local src=$1; shift
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_topological_order --only-names "/opt/ros/$ROS_DISTRO/share"
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_topological_order --only-names "$src"
    fi
    if [ "$ROS_VERSION" -eq 2 ]; then
        local src=$1; shift
        colcon list --base-paths "/opt/ros/$ROS_DISTRO/share" --names-only
        colcon list --base-paths "$src" --names-only
    fi
}
function setup_rosdep {
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    if [ "$ROS_VERSION" -eq 1 ]; then
        if [ "$ROS_DISTRO" == "noetic" ]; then
            if ! command -v rosdep > /dev/null; then
                apt_get_install python3-rosdep > /dev/null
            fi
        else
            if ! command -v rosdep > /dev/null; then
                apt_get_install python-rosdep > /dev/null
            fi
        fi
    fi
    if [ "$ROS_VERSION" -eq 2 ]; then
        if ! command -v rosdep > /dev/null; then
            apt_get_install python3-rosdep > /dev/null
        fi
    fi

    if command -v sudo > /dev/null; then
        sudo rosdep init || true
    else
        rosdep init | true
    fi
    rosdep update
}
function resolve_depends {
    if [[ "$ROS_VERSION" -eq 1 ]]; then
        local src=$1; shift
        comm -23 <(read_depends "$src" "$@"| sort -u) <(list_packages "$src" | sort -u) | xargs -r "/opt/ros/$ROS_DISTRO"/env.sh rosdep resolve | grep_opt -v '^#' | sort -u
    fi
    if [[ "$ROS_VERSION" -eq 2 ]]; then
        local src=$1; shift
        comm -23 <(read_depends "$src" "$@"| sort -u) <(list_packages "$src" | sort -u) | xargs -r rosdep resolve | grep_opt -v '^#' | sort -u
    fi
    if [ "$ROS_VERSION" -ne 2 ] && [ "$ROS_VERSION" -ne 1 ]; then
        echo "Cannot get ROS_VERSION"
        exit 1
    fi
}

function apt_get_install {
    local cmd=()
    if command -v sudo > /dev/null; then
        cmd+=(sudo)
    fi
    cmd+=(apt-get install --no-install-recommends -qq -y)
    if [ -n "$*" ]; then
        "${cmd[@]}" "$@"
    else
        xargs -r "${cmd[@]}"
    fi
}

function build_workspace {
    local ws=$1; shift
    apt_get_install build-essential
    setup_rosdep
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    ls $ws/src
    for file in "$ws/src/*.rosinstall"; do
        if [ -f ${file} ]; then
            if ! command -v vcstool > /dev/null; then
                if [[ "$ROS_VERSION" -eq 1 ]]; then
                    echo "It is: $ROS_VERSION"
                    if [ "$ROS_DISTRO" = "noetic" ]; then
                        echo "It is: $ROS_DISTRO"
                        apt_get_install python3-vcstool > /dev/null
                    else
                        apt_get_install python-vcstool > /dev/null
                    fi
                fi
                if [[ "$ROS_VERSION" -eq 2 ]]; then
                    echo "It is: $ROS_DISTRO, $ROS_VERSION"
                    apt_get_install python3-vcstool > /dev/null
                fi
                if [ "$ROS_VERSION" -ne 2 ] && [ "$ROS_VERSION" -ne 1 ]; then
                    echo "Cannot get ROS_VERSION"
                    exit 1
                fi
            fi
            if ! command -v git > /dev/null; then
                apt_get_install git > /dev/null
            fi
            echo "vcs import"
            vcs import $ws/src/ < $file
            rm $file
        fi
    done;
    for folder in "$ws/src"/*; do
        echo "find: ${folder}"
        if [[ -d ${folder} ]]; then
            echo "find folder: ${folder}"
            for file in "${folder}/*.rosinstall" "${folder}/rosinstall"; do
                if [ -f ${file} ]; then
                    echo "vcs import ${file}"
                    vcs import $ws/src/ < $file
                fi
            done;
        fi
    done;
    resolve_depends "$ws/src" depend build_depend build_export_depend | apt_get_install
    resolve_depends "$ws/src" depend build_export_depend exec_depend run_depend > "$ws/DEPENDS"
    if [ "$ROS_VERSION" -eq 1 ]; then
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_make_isolated -C "$ws" -DCATKIN_ENABLE_TESTING=0
    fi
    if [[ "$ROS_VERSION" -eq 2 ]]; then
        cd $ws && colcon build
    fi
}

function test_workspace {
    local ws=$1; shift
    resolve_depends "$ws/src" depend exec_depend run_depend test_depend | apt_get_install
    if [ "$ROS_VERSION" -eq 1 ]; then
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_make_isolated -C "$ws" -DCATKIN_ENABLE_TESTING=1
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_make_isolated -C "$ws" --make-args run_tests -j1
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_test_results --verbose "$ws"
    else
        cd $ws && colcon test
        colcon test-result --verbose
    fi
}

function install_depends {
    local ws=$1; shift
    apt_get_install < "$ws/DEPENDS"
}

function install_workspace {
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    echo "ROS_VERSION: $ROS_VERSION"
    echo "ROS_DISTRO: $ROS_DISTRO"
    if [[ "$ROS_VERSION" -eq 1 ]]; then
        echo "It is: $ROS_DISTRO"
        local ws=$1; shift
        "/opt/ros/$ROS_DISTRO"/env.sh catkin_make_isolated -C "$ws" --install --install-space "/opt/ros/$ROS_DISTRO"
    fi
    if [[ "$ROS_VERSION" -eq 2 ]]; then
        echo "It is: $ROS_DISTRO"
        local ws=$1; shift
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        # didn't work on galactic
        cd $ws && rm -r install build && colcon build --merge-install --install-base "/opt/ros/$ROS_DISTRO"
    fi
    if [ "$ROS_VERSION" -ne 2 ] && [ "$ROS_VERSION" -ne 1 ]; then
        exit 1
    fi
}

if [ -n "$*" ]; then
    "$@"
fi

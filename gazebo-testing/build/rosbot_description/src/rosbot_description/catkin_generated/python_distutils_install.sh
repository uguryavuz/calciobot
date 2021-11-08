#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/root/catkin_ws/calciobot/gazebo-testing/src/rosbot_description/src/rosbot_description"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/root/catkin_ws/calciobot/gazebo-testing/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/root/catkin_ws/calciobot/gazebo-testing/install/lib/python2.7/dist-packages:/root/catkin_ws/calciobot/gazebo-testing/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/root/catkin_ws/calciobot/gazebo-testing/build" \
    "/usr/bin/python2" \
    "/root/catkin_ws/calciobot/gazebo-testing/src/rosbot_description/src/rosbot_description/setup.py" \
     \
    build --build-base "/root/catkin_ws/calciobot/gazebo-testing/build/rosbot_description/src/rosbot_description" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/root/catkin_ws/calciobot/gazebo-testing/install" --install-scripts="/root/catkin_ws/calciobot/gazebo-testing/install/bin"

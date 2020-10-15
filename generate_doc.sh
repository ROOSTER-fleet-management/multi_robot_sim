#!/bin/bash
# A script to generate documentation and copy the necessary files to the appropriate directories
# so that the html files can be hosted on github pages.

roscd multi_robot_sim
rosdoc_lite .

DIRECTORY=docs
if [ ! -d "$DIRECTORY" ]; then
    mkdir docs
fi

#copying built html files to docs folder to be viewed in browser
/bin/cp -r doc/html/python/* docs/

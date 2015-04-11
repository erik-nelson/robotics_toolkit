#!/bin/bash

# Get a list of all tests
FP=$(rospack find robotics_toolkit)/test
EXECUTABLES=$(find ${FP} -name '*.cc')

GREEN='\033[01;33m'
RESTORE='\033[0m'

# Start a roscore instance
#roscore & &> /dev/null
#ROSCORE_PID=$!
#sleep 1

# Loop over tests
for FILE in ${EXECUTABLES}; do

  # Trim off directory and file extension
  # e.g. "/home/erik/....../test1.cc" --> "test1"
  TRIMMED=${FILE##*/} # directory
  TRIMMED=${TRIMMED%.*} # extension

  # Run the test rosnode
  echo -e '\n' ${GREEN} 'Running test script: ' ${TRIMMED} '\n' ${RESTORE}
  rosrun robotics_toolkit ${TRIMMED}
done

# Stop the background roscore instance
#kill ${ROSCORE_PID}

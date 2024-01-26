#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
colcon test
colcon test-result --verbose

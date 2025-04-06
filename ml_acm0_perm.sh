#!/bin/bash
PORT="${1:-ttyACM0}"

sudo touch /run/lock/LCK..$PORT
sudo chgrp $USER /run/lock/LCK..$PORT
sudo chown $USER /run/lock/LCK..$PORT

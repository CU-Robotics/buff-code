#! /bin/bash

#!/bin/bash
set -e

buffbash="/home/cu-robotics/buff-code/buffpy/buff.bash"
echo "sourcing   	$buffbash"
source "$buffbash"

echo "Host 		$HOSTNAME"
echo "Project Root 	$PROJECT_ROOT"

exec "$@"
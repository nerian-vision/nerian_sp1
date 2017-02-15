#!/bin/bash
if [ $# -lt 2 ]; then
	echo "Usage: $0 SP1-ADDRESS OUTPUT-FILE"
	exit 1
fi

curl http://$1/calibration/calib.yaml > $2
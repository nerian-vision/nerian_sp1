#!/bin/bash
if [ $# -lt 1 ]; then
	echo "Usage: $0 SP1-ADDRESS"
	exit 1
fi

curl --data "op_mode=pass" http://$1/settings/ > /dev/null

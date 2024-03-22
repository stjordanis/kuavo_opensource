#!/bin/bash
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=./lib/jodell_claw_driver/lib:$LD_LIBRARY_PATH

echo "Try to run command: $@"

$@

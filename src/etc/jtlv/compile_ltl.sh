#!/bin/bash
if [ -v $1 -a -v $2 ]
    then
    echo "Usage: compile_ltl smv ltl"
    exit 64
fi
java -ea -Xmx1g -cp GROne:jtlv-prompt1.4.0.jar GROneDebug $1 $2

#!/bin/bash

if ! command -v matlab > /dev/null
then
    echo "Matlab not installed, exiting"
    exit 0;
fi

pwd | grep scripts > /dev/null
if [[ $? == 0 ]]
then
    cd ..
fi
matlab -nodesktop -r 'try; run_from_bash=1; main; catch; end; quit' || exit
#!/usr/bin/env bash

cd ../matlab;
matlab -nosplash -nodesktop -r "addpath('soccer-vision/test'); run('test_soccer_vision.m');exit;"
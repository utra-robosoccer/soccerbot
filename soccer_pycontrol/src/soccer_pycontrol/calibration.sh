#!/usr/bin/env bash


until python calibration.py --collect
do
  sleep 1
done
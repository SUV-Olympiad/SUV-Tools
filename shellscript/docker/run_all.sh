#!/bin/bash

docker stop olympiad qgc1 qgc2 qgc3 leap
docker rm olympiad qgc1 qgc2 qgc3 leap
sh run_olympiad.sh &
sh run_qgc1.sh &
sh run_qgc2.sh &
sh run_qgc3.sh &

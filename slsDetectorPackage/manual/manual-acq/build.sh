#!/bin/bash

mvn clean install -f pom_64.xml -o
cp ./target/nar/bin/amd64-Linux-g++/ds_manual-acq ../../bin/Linux64/ds_manual-acq

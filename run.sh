#!/bin/bash
make clean
make all DEBUG=0 CLOAD=1
echo 'Make done'
make cload
echo 'Cload done'

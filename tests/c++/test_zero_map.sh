#!/bin/bash
cd ../../src/
make -j 4
cd -
make -j 4
./test_zero_map

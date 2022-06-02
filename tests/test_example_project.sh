#!/bin/bash
# args are CMAKE_PREFIX_PATH CMAKE_INSTALL_PREFIX  NUM_CORES
echo "WARNING: this test assumes that there was a make install before!"
set -x

# out of the build dir
cd ../..
rm -rf example_project/build 
rm -rf /tmp/example_project
cp -r example_project /tmp
cd /tmp/example_project
sh ./create_project.sh testc
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ${@:1:${#}-1} ..
make -j$7




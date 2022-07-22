# Installation

If you need more information you can also check the dockerfile [here](https://gitlab.inria.fr/locolearn/public/docker_inria_wbc)
## Create an installation directory

```
cd ~
mkdir install
```

## yaml-cpp

```
sudo apt-get install libyaml-cpp-dev
```

## assimp 5.0

```
git clone --depth 1 https://github.com/assimp/assimp.git
cd assimp
git checkout v5.0.0
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
make clean
```

## pinocchio

More information on pinnochio [here](https://github.com/stack-of-tasks/pinocchio)

```
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='/home/$USERNAME/install' -DCMAKE_PREFIX_PATH='/home/$USERNAME/install' -DBUILD_PYTHON_INTERFACE=OFF ..
make -j$(nproc)
sudo make install
```
## tsid

Install [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) first

```
git clone --recursive https://github.com/stack-of-tasks/eiquadprog.git
sed -i '/#define TRACE_SOLVER/c\//#define TRACE_SOLVER' eiquadprog/src/eiquadprog-fast.cpp
cd eiquadprog
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='/home/$USERNAME/install' -DCMAKE_PREFIX_PATH='/home/$USERNAME/install' ..
make -j$(nproc)
sudo make install
```

Install [tsid](https://github.com/stack-of-tasks/tsid)

```
cd /home/$USERNAME
git clone  --recursive https://github.com/stack-of-tasks/tsid.git
cd tsid
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='/home/$USERNAME/install' -DCMAKE_PREFIX_PATH='/home/$USERNAME/install' -DBUILD_PYTHON_INTERFACE=OFF ..; \
make -j$(nproc)
sudo make install
```

## robot_dart

Use this [installation-guide](https://github.com/resibots/robot_dart/blob/master/docs/installation.md) without the -DDART_ENABLE_SIMD=ON for the dart installation

(If you install with -DDART_ENABLE_SIMD=ON  make sure that everything is installed with SIMD instructions / -march=native flag)

Install dart in ~/install following :
```
./waf configure --dart=~/install --corrade_install_dir=~/install --magnum_install_dir=~/install --magnum_integration_install_dir=~/install --magnum_plugins_install_dir=~/install --prefix=~/install
./waf
./waf install
```

## inria_wbc
```
git clone https://github.com/resibots/inria_wbc.git
cd inria_wbc
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX='~/install' -DCMAKE_PREFIX_PATH='~/install' ..
make
sudo make install
```

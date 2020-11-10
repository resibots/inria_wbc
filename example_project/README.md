# Example project for inria_wbc
# How to create a project?
- copy the content of example_project to your new repository or directory
- run `./create_project.sh your_project_name` (e.g. , `./create_project.sh test_learning`)
- this should rename the cmake files and create correct cmake files
- to compile:
  - `mkdir build`
  - `cd build`
  - `cmake .. -DCMAKE_INSTALL_PREFIX=/user/install -DCMAKE_PREFIX=/user/install -DCMAKE_BUILD_TYPE=Debug` (where `/user/install` is where you have installed inria_wbc via `make install`)
  - `make`
  - `make install`
- you can edit the files in `src` and `include`
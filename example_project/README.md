# Example project for inria_wbc

# Preliminaries
- the `inria_wbc` repository should contain the core of our whole-body controller: it should be stable, and well tested
- scientific and experimental code should be in a different repository (*not* in a branch of inria_wbc, because this would quickly pollute the repository)
- scientific and experimental code is yours: it can be messy, do not worry

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
- you can now edit (and rename!) the files in `src` and `include`
    - `ex_behavior.{cpp,hpp}`: example of a behavior (specific trajectories/interactions)
    - `ex_controller.{cpp,hpp}`: example of a controller (specific stack)
    - `ex_task.{cpp,hpp}`: example of a TSID task
- usually the name of the file reflects the name of the class
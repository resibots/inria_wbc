#!/bin/sh
# set -x
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 name_of_project" >&2
  exit 1
fi

PROJECT_NAME=$1
PROJECT_NAME_UC=$(echo $1 | tr '[:lower:]' '[:upper:]')

FILES=$(find . -type f|grep -v $0|grep -v README.md)
for i in $FILES; do
    echo "Replacing in $i..."
    sed -i -e "s/@project_name@/$PROJECT_NAME/g" $i
    sed -i -e "s/@PROJECT_NAME@/$PROJECT_NAME_UC/g" $i
done
mv cmake/example_project.cmake.in cmake/$PROJECT_NAME.cmake.in 
mv cmake/example_projectConfigVersion.cmake.in cmake/${PROJECT_NAME}ConfigVersion.cmake.in
mv etc/example_project.yaml etc/$PROJECT_NAME.yaml
find .|grep -- -e|xargs rm
    
    

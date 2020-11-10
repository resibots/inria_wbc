#!/bin/sh

PROJECT_NAME=$1
PROJECT_NAME_UC=echo $1 | tr '[:lower:]' '[:upper:]'

FILES=$(find .)
for i in $FILES; do
    echo "Replacing in $i..."
    sed -i -e s/@project_name@/$PROJECT_NAME/g
    sed -i -e s/@PROJECT_NAME@/$PROJECT_NAME_UC/g
done
mv cmake/example_project.cmake.in cmake/$PROJECT_NAME.cmake.in 
mv cmake/example_projectConfigVersion.cmake.in cmake/{$PROJECT_NAME}ConfigVersion.cmake.in 

    
    
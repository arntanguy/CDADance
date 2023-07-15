#!/bin/bash

# Step 1: Remove existing build directory
rm -rf build

# Step 2: Create build directory and navigate into it
mkdir -p build
cd build

# Step 3: Run CMake
cmake ..

# Step 4: Build the project using make
make

# Step 5: inside Build and install
sudo make install


cmake_minimum_required(VERSION 2.8)
project(nao_kinect1.0_control)

find_package(qibuild)

# Create a executable named nao_kinect1.0_control
# with the source file: main.cpp
qi_create_bin(nao_kinect1.0_control "main.cpp")
qi_use_lib(nao_kinect1.0_control ALCOMMON ALPROXIES)

# Add a simple test:
qi_create_test(test_nao_kinect1.0_control "test.cpp")

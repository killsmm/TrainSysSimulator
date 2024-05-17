# Copyright 2024 zhouxinliang
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     https://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

project(train_simulator)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

add_executable(train_simulator
    src/main.cpp
)

target_include_directories(train_simulator
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    "/usr/include/opencv4"
)

target_link_libraries(train_simulator
    PRIVATE
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_imgcodecs)


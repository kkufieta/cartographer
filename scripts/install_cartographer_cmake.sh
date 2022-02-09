#!/bin/sh

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

# Build and install Cartographer.
pushd cartographer

rm -rf build
mkdir build
pushd build

cmake .. -G Ninja -DCMAKE_CXX_STANDARD=17 -DCMAKE_PREFIX_PATH=`brew --prefix` -DQt5_DIR=$(brew --prefix qt5)/lib/cmake/Qt5
ninja
sudo ninja install
popd
popd
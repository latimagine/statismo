#
# This file is part of the statismo library.
#
# Copyright (c) 2019 Laboratory of Medical Information Processing
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# Neither the name of the project's author nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# This script is used to check code basis for a new release
#

#!/bin/bash

if [[ $# != 1 ]]; then
  echo "FAILED: You must provide the version number x.y.z as argument"
  exit 1
fi

tag=$1
check_version_cmake=0
check_version_readme=0
check_version_changelog=0
check_version_conan=0
check_buildstatus_branch=0
check_doc_branch=0

echo "--- Checking version numbers"

version=$(grep -o -E "project.*VERSION\s([0-9]+)\\.([0-9]+)\\.([0-9]+)" CMakeLists.txt | awk '{print $NF}')
if [[ $version !=  $tag ]]; then
  check_version_cmake=1
fi

grep -E "\[Latest Release\](.+)$tag" README.md > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  check_version_readme=1
fi

grep "# \[$tag\]" CHANGELOG.md > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  check_version_changelog=1
fi

grep "version = \"$tag\"" conanfile.py > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  check_version_conan=1
fi

echo "--- Checking ci branches"

grep -E "\[Build Status\](.+)branch=master" README.md > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  check_buildstatus_branch=1
fi

echo "--- Checking doc branches"

grep "branch: master" .travis.yml > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  check_doc_branch=1
fi

echo " "
echo "============================ SUMMARY ============================="
if [[ $check_version_cmake == 0 ]]; then
  echo "Version in root cmake script: OK"
else
  echo "Version in root cmake script: KO (found $version instead of $tag)"
fi
if [[ $check_version_readme == 0 ]]; then
  echo "Version in README: OK"
else
  echo "Version in README: KO (Update Latest Release badge)"
fi
if [[ $check_version_changelog == 0 ]]; then
  echo "Version item in CHANGELOG: OK"
else
  echo "Version item in CHANGELOG: KO (Create a version item in CHANGELOG)"
fi
if [[ $check_version_conan == 0 ]]; then
  echo "Version item in conanfile.py: OK"
else
  echo "Version item in conanfile.py: KO"
fi
if [[ $check_buildstatus_branch == 0 ]]; then
  echo "Build Status reference branch: OK"
else
  echo "Build Status reference branch: KO (Update the Build Status to master)"
fi
if [[ $check_doc_branch == 0 ]]; then
  echo "Doc branch generation: OK"
else
  echo "Doc branch generation: KO (Update the CI script to generate doc from master)"
fi

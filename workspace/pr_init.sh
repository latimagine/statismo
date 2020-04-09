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
# This script should make your local repo ready for working on
# a Pull Request
#

#!/bin/bash

echo "--- Workspace setup started!"

#
# Switch to the right branch and update
#

echo "--- Switch to develop and update"

git fetch >/dev/null 2>&1
git checkout develop > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  echo " "
  echo "FAILED: Switching to branch develop failed!"
  exit 1
fi

git merge --ff-only > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  echo " "
  echo "FAILED: Could not update develop branch properly!"
  exit 1
fi

#
# Set the hooks for development
# Just copy in case 'git config core.hooksPath' is not handled
#

echo "--- Set hooks for development"

cp .githooks/* .git/hooks/

if [[ "$?" -ne "0" ]]; then
  echo "WARNING: Failed to copy hooks from .githooks to .git/hooks, either it is already done or you should do it manually"
fi

#
# Check your personal information is ok
#

echo "--- Checking identification details"

regex="^[a-z0-9!#\$%&'*+/=?^_\`{|}~-]+(\.[a-z0-9!#$%&'*+/=?^_\`{|}~-]+)*@([a-z0-9]([a-z0-9-]*[a-z0-9])?\.)+[a-z0-9]([a-z0-9-]*[a-z0-9])?\$"
if [[ $(git config --global user.email) =~ $regex ]] || [[ $(git config --local user.email) =~ $regex ]] ; then
  echo "[+] Email check success!"
else
  echo " "
  echo "FAILED: Set your git email to a proper value with 'git config --local user.email $email'!"
  exit 1
fi

if [[ $(git config --global user.name) != "" ]] || [[ $(git config --local user.name) != "" ]] ; then
  echo "[+] Name check success!"
else
  echo " "
  echo "FAILED: Set your git username to a proper value with 'git config --local user.name $name'!"
  exit 1
fi

#
# Chosing the PR branch
#

echo "--- Creating a branch for the PR"

echo "Enter the branch name for your PR: "
read branchname

regex="^(feat|fix|sanity|prerelease|release|hotfix)\/[a-z0-9._-]+$"
if [[ ! $branchname =~ $regex ]]; then
  echo " "
  echo "FAILED: Your branch name must follow the $regex pattern"
  exit 1
fi

git checkout -b $branchname > /dev/null 2>&1

if [[ "$?" -ne "0" ]]; then
  echo " "
  echo "FAILED: Failed to switch to your PR branch"
fi

echo "--- Workspace setup success!"
echo " "
echo "--> You are now ready to develop, check the coding rules in CONTRIBUTING.md!"
echo " "

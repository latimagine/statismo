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
# Script used to install statismo and its dependencies from recipes
#
# NOTE: The install is currently experimental with local vtk and itk
#	recipes that does not taken into account most of these frameworks
#	build options.
#

#!/bin/bash

STATISMO_ROOT=$1

echo "Start of conan install..."
echo "Statismo location: ${STATISMO_ROOT}"

echo "Installing VTK..."
cd ${STATISMO_ROOT}/deploy/pack/conan/conan-recipes/vtk && conan create -s compiler.libcxx=libstdc++11 . user/stable

if [[ "$?" -ne "0" ]]; then
 echo "Failed: Installing VTK failed (see /tmp/conancreatevtk.log for details)!"
 exit 1
fi

echo "Installing ITK..."
cd ${STATISMO_ROOT}/deploy/pack/conan/conan-recipes/itk && conan create -s compiler.libcxx=libstdc++11 . user/stable

if [[ "$?" -ne "0" ]]; then
 echo "Failed: Installing ITK failed (see /tmp/conancreateitk.log for details)!"
 exit 1
fi

echo "Installing Statismo..."
cd ${STATISMO_ROOT} && conan create -s compiler.libcxx=libstdc++11 -tf ${STATISMO_ROOT}/deploy/pack/conan/test_package  . user/stable

if [[ "$?" -ne "0" ]]; then
 echo "Failed: Installing Statismo failed (see /tmp/conancreatestatismo.log for details)!"
 exit 1
fi

echo "End of installation"

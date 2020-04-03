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
# This is an extended validation scripts used offline in addition to travis CI
#

#!/bin/bash

DOCKER_DIR=$1
GIT_BRANCH=$2

echo "Start validation process..."
echo "Processing all docker files in directory ${DOCKER_DIR}"

for DOCK in $(find ${DOCKER_DIR} -name "Dockerfile-*")
do
    DOCKNAME=$(basename -- "${DOCK}")
    DOCKNAME_LOWER=$(echo "$DOCKNAME" | tr '[:upper:]' '[:lower:]')
    echo "Processing container ${DOCKNAME}"

    sudo docker build --no-cache --rm --build-arg git_branch=${GIT_BRANCH} \
     --file ${DOCK} --tag ${DOCKNAME_LOWER}-img ${DOCKER_DIR} > /tmp/${DOCKNAME_LOWER}.log 2>&1
    
    if [[ "$?" -ne "0" ]]; then
        echo "Failed: Building container ${DOCKNAME} failed (see /tmp/${DOCKNAME_LOWER}.log for details)!"
    fi
done
echo "End of validation process"
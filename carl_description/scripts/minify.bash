#!/bin/bash

# go to the meshes folder
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MESHES="$DIR/../meshes"
MATERIALS="$MESHES/materials"

# remove old minified files
cd $MESHES
rm *.min.dae

# re-minify
jpegoptim $MATERIALS
for f in *.dae ; do xmllint --noblanks $f > "${f/.dae/.min.dae}" ; done

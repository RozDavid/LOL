#!/usr/bin/env bash
COMPILER="$1"
echo "#include <string>" | ${COMPILER} -x c++ -E -dM - | fgrep _GLIBCXX_USE_CXX11_ABI
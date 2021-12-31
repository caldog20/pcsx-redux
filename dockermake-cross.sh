#!/bin/sh

ROOT=$(dirname $0)
CWD=$(pwd)
cd $ROOT
ROOT=$(pwd)
cd $CWD

docker pull caldog20/aarch64cc:latest
docker run --rm --env-file ${ROOT}/cross-env.list -i -w/project${CWD#$ROOT} -v "${ROOT}:/project" -u `id -u`:`id -g` caldog20/aarch64cc:latest make --makefile=Makefile-CROSS $@
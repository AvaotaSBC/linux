#!/usr/bin/env bash

__usage="
Usage: init [OPTIONS]
Build Kernel repo.

Options:
  -u, --url        URL               The url of kernel
  -v, --version    VERSION           The version of kernel
  -p, --patch      PATCHFILE         The patch file of kernel
  -h, --help                         Show command help.
"

help()
{
    echo "$__usage"
    exit $1
}

default_param() {
    URL=https://cdn.kernel.org/pub/linux/kernel/v5.x/
    PATCH=linux-5.15.patch
    VERSION=linux-5.15.154
}

parseargs()
{
    if [ "x$#" == "x0" ]; then
        return 0
    fi

    while [ "x$#" != "x0" ];
    do
        if [ "x$1" == "x-h" -o "x$1" == "x--help" ]; then
            return 1
        elif [ "x$1" == "x" ]; then
            shift
        elif [ "x$1" == "x-u" -o "x$1" == "x--url" ]; then
            URL=`echo $2`
            shift
            shift
        elif [ "x$1" == "x-v" -o "x$1" == "x--version" ]; then
            VERSION=`echo $2`
            shift
            shift
        elif [ "x$1" == "x-p" -o "x$1" == "x--patch" ]; then
            PATCH=`echo $2`
            shift
            shift
        else
            echo `date` - ERROR, UNKNOWN params "$@"
            return 2
        fi
    done
}

HOST_ARCH=$(arch)
ROOT_PATH=$(pwd)
PATCH_PATH="patches"
TARBALL="tar.xz"

default_param
parseargs "$@" || help $?

echo You are running this scipt on a ${HOST_ARCH} mechine....

mkdir -p kernel/${VERSION}

echo Downloading kernel source code

wget ${URL}/${VERSION}.${TARBALL} -O kernel/${VERSION}/${VERSION}.${TARBALL}

echo Unarchive Kernel 

cd kernel/${VERSION}
tar xf ${VERSION}.${TARBALL}
cd ${VERSION}

cp -raf ${ROOT_PATH}/${PATCH_PATH}/${PATCH} .
patch --binary -p1 < ${PATCH}


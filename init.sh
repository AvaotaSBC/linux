#!/usr/bin/env bash

# SPDX-License-Identifier: GPL-2.0

# Usage message for the init script
__usage="
Usage: init [OPTIONS]
Build Kernel repo.

Options:
  -u, --url        URL               The url of kernel
  -v, --version    VERSION           The version of kernel
  -b, --branch     BRANCH NAME       The branch of dst kernel
  -t, --type       KERNEL TYPE       The kernel type: LTS from kernel.org, AOSP kernel
  -h, --help                         Show command help.
"

# Function to display help message
help()
{
    echo "$__usage"
    exit $1
}

# Function to set default parameters
default_param() {
    URL=https://cdn.kernel.org/pub/linux/kernel/v5.x/
    VERSION=linux-5.15.154
    BRANCH=linux-5.15
    TYPE=LTS
}

# Function to parse command line arguments
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
        elif [ "x$1" == "x-b" -o "x$1" == "x--branch" ]; then
            BRANCH=`echo $2`
            shift
            shift
        elif [ "x$1" == "x-t" -o "x$1" == "x--type" ]; then
            BRANCH=`echo $2`
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
DATE=$(date)
PATCH_PATH="patches"

# Set default parameters
default_param
# Parse command line arguments
parseargs "$@" || help $?

# Display system architecture
echo You are running this script on a ${HOST_ARCH} machine....

# Create directory for kernel version
mkdir -p kernel/${VERSION}

# Download kernel source code
echo Downloading kernel source code
if [ "$TYPE" = "LTS" ]; then
    TARBALL="tar.xz"
    wget ${URL}/${VERSION}.${TARBALL} -qO kernel/${VERSION}/${VERSION}.${TARBALL}
else
    TARBALL="tar.gz"
    wget ${URL} -qO kernel/${VERSION}/${VERSION}.${TARBALL}
fi

# Unarchive Kernel
echo Unarchive Kernel
cd kernel/${VERSION}
tar xf ${VERSION}.${TARBALL}
cd ${VERSION}

# Copy BSP files
echo Copying BSP files
cp -raf ${ROOT_PATH}/bsp .

# Applying Patches and Copy overlays, CI
echo Applying Patches and overlays
cp -raf ${ROOT_PATH}/${PATCH_PATH}/${BRANCH}/*.patch .
cp -raf ${ROOT_PATH}/${PATCH_PATH}/${BRANCH}/overlays/* .
for patchfile in *.patch; do
    patch --binary -p1 < "$patchfile"
done
rm -rf *.patch
cd ${ROOT_PATH}

# Clone old kernel
echo Cloning old kernel
git clone https://github.com/AvaotaSBC/linux.git -b ${BRANCH} kernel/dst/${BRANCH}/${BRANCH}
cd kernel/dst/${BRANCH}/${BRANCH}

# Merge old kernel
echo Merging old kernel
mv .git ../
rm -rf *
cp -raf ${ROOT_PATH}/kernel/${VERSION}/${VERSION}/. .
mv ../.git .

# Git commit to archive
echo Git commit to archive
git add .
git add -f .github
git commit -m "${DATE} Kernel update"

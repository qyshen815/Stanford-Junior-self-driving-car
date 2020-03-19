#! /bin/sh

#
# Usage: build.sh {gcc}
#

ARCH=linuxem64t

# uncomment below line if you want to link with static IPP libraries
#LINKAGE=static


IPPENV=${IPPROOT}/tools/env/ippvarsem64t.sh

if [ ! -x "${IPPENV}" ]; then
  IPPENV=/opt/intel/ipp/5.0/em64t/tools/env/ippvarsem64t.sh
fi

if [ ! -x "${IPPENV}" ]; then
  echo -e "*************************************************************************"
  echo -e "Intel(R) IPP is not found!"
  echo -e "Please install Intel(R) IPP or set IPPROOT environment variable correctly."
  echo -e "*************************************************************************"
  exit
fi

ARG=$1

CC=gcc
CXX=g++

if [ "${ARG}" == "" ]; then

  CCFIND=/opt/intel/cce/9.0/bin/iccvars.sh
  if [ -x "${CCFIND}" ]; then
    CCENV=${CCFIND}
  fi

  if [ "${CCENV}" ]; then
    . ${CCENV}
    CC=icc
    CXX=icc
  fi
fi

. ${IPPENV}

make ARCH=${ARCH} clean
make ARCH=${ARCH} CC=${CC} CXX=${CXX}

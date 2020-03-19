#! /bin/sh

#
# Usage: build32.sh {gcc}
#

ARCH=linux32

# uncomment below line if you want to link with static IPP libraries
LINKAGE=static


IPPENV=${IPPROOT}/tools/env/ippvars32.sh

if [ ! -x "${IPPENV}" ]; then
  IPPENV=/opt/intel/ipp/5.0/ia32/tools/env/ippvars32.sh
fi

if [ ! -x "${IPPENV}" ]; then
  echo -e "*************************************************************************"
  echo -e "Intel(R) IPP is not found!"
  echo -e "Please install Intel(R) IPP or set IPPROOT environment variable correctly."
  echo -e "*************************************************************************"
  exit
fi

ARG=$1

CC=cc
CXX=c++

if [ "${ARG}" == "" ]; then

  CCFIND=/opt/intel/cc/9.0/bin/iccvars.sh
  if [ -x "${CCFIND}" ]; then
    CCENV=${CCFIND}
  fi

  CCFIND=/opt/intel_cc_80/bin/iccvars.sh
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

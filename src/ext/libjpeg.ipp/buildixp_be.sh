#! /bin/sh

#
# Usage: build.sh {gcc}
#

ARCH=linuxixp_be

# only static IPP libraries are available for IPP for IXP
LINKAGE=static


IPPENV=${IPPROOT}/tools/env/ippvarsixp.sh

if [ ! -x "${IPPENV}" ]; then
  IPPENV=/opt/intel/ipp/5.0/ixp/tools/env/ippvarsixp.sh
fi

if [ ! -x "${IPPENV}" ]; then
  echo -e "*************************************************************************"
  echo -e "Intel(R) IPP is not found!"
  echo -e "Please install Intel(R) IPP or set IPPROOT environment variable correctly."
  echo -e "*************************************************************************"
  exit
fi

ARG=$1

CC=/opt/montavista/pro/devkit/arm/xscale_be/bin/xscale_be-gcc
CXX=/opt/montavista/pro/devkit/arm/xscale_be/bin/xscale_be-g++

#if [ "${ARG}" == "" ]; then
#
#  CCFIND=/opt/intel_cc_80/bin/iccvars.sh
#  if [ -x "${CCFIND}" ]; then
#    CCENV=${CCFIND}
#  fi
#  
#  CCFIND=/opt/intel/cc/9.0/bin/iccvars.sh
#  if [ -x "${CCFIND}" ]; then
#    CCENV=${CCFIND}
#  fi
#  
#  if [ "${CCENV}" ]; then
#    . ${CCENV}
#    CC=icc
#    CXX=icc
#  fi
#
#fi

. ${IPPENV}

make ARCH=${ARCH} clean
make ARCH=${ARCH} CC=${CC} CXX=${CXX} LINKAGE=${LINKAGE}

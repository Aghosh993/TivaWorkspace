#******************************************************************************
#
# Makefile - Rules for building the project example.
#
# Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the  
#   distribution.
# 
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# This is part of revision 2.1.2.111 of the Tiva Firmware Development Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PZ

#
# J-Link driver package root directory:
#
JLINK_ROOT=../JLink_Linux_V480_x86_64/

#
# The base directory for TivaWare.
#
ROOT=../tivaware_old

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH=$(ROOT)

#
# The default rule, which causes the project example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/project.axf
md5sums: all
	md5sum $(COMPILER)/project.axf > tm4c_fsw_md5sum
check: all
	md5sum $(COMPILER)/project.axf > tmp && diff tmp tm4c_fsw_md5sum && rm tmp
load: all
	./load_fw.sh $(JLINK_ROOT)

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the project.
#
${COMPILER}/project.axf: ${COMPILER}/project.o
${COMPILER}/project.axf: ${COMPILER}/serial_comms_highlevel.o
${COMPILER}/project.axf: ${COMPILER}/serial_comms_highlevel_hal.o
${COMPILER}/project.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/project.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/project.axf: project.ld
SCATTERgcc_project=project.ld
ENTRY_project=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RA3

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

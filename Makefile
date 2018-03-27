include $(EPICS_ENV_PATH)/module.Makefile

EXCLUDE_ARCHS = eldk

STARTUPS = startup/qepro.cmd
DOC      = doc
OPIS     = opi

EXCLUDE_ARCHS = eldk
DBDS = src/drvUSBQEProSupport.dbd

AUTO_DEPENDENCIES = NO
USR_DEPENDENCIES += asyn,4.3+

#ifndef EPICSVERSION
#build: ${BUILD_PATH}/misc/OOI_HOME
#
#${BUILD_PATH}/misc/OOI_HOME: OOI_HOME
#	${MKDIR} ${BUILD_PATH}/misc
#	${CP} -a $< $@
#endif



#USR_CPPFLAGS += -I$(abspath ../../include) -DLINUX
USR_CPPFLAGS += -I/vagrant/omniDrv/include -DLINUX
#USR_CPPFLAGS += -I$(abspath ../../java/include)
USR_CPPFLAGS += -I/usr/java/latest/include -I/usr/java/latest/include
#USR_CPPFLAGS += -I$(abspath ../../java/include/linux)
USR_CPPFLAGS += -I/usr/java/latest/include -I/usr/java/latest/include/linux

#PROD_SYS_LIBS += libcommon.so
#PROD_SYS_LIBS += libOmniDriver.so
#PROD_SYS_LIBS += libjvm.so

USR_LDFLAGS += -L/vagrant/omniDrv/OOI_HOME -lcommon -lOmniDriver
USR_LDFLAGS += -L/usr/java/latest/jre/lib/amd64/server -ljvm
USR_LDFLAGS += -lcommon -lOmniDriver -ljvm
USR_LDFLAGS += -lusb

#OMNIDRV_DIR = $(abspath ../../OOI_HOME)
#USR_LDFLAGS += -L$(OMNIDRV_DIR) -lcommon -lOmniDriver
#USR_LDFLAGS += -Wl,--rpath=$(OMNIDRV_DIR)

#JAVALIB_DIR = $(abspath ../../OOI_HOME/_jvm/lib/amd64/server)
#USR_LDFLAGS += -L$(JAVALIB_DIR) -ljvm
#USR_LDFLAGS += -Wl,--rpath=$(JAVALIB_DIR)

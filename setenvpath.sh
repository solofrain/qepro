#!/bin/bash

##############################################################
###### if you want to use local
 #JAVA_JRE="/usr/java/jdk1.8.0_131/jre/lib/amd64/server"
 JAVA_JRE="/usr/java/jdk1.8.0_171-amd64/jre/lib/amd64/server"
 #OOI_HOME="/vagrant/modules/m-epics-qepro/omniDrv/OOI_HOME"
 OOI_HOME="/home/waynelewis/git/m-epics-qeproasyn/omniDrv/OOI_HOME"

 export LD_LIBRARY_PATH=$JAVA_JRE:$OOI_HOME
 export OOI_HOME=$OOI_HOME

##############################################################
###### if you want to use from /opi/epics/modules.....
#JAVA_JRE="/opt/epics/java/jre/lib/amd64/server"
#OOI_HOME="/opt/epics/modules/qepro/tomaszbrys/opi/omniDrv/OOI_HOME"

#export LD_LIBRARY_PATH=$JAVA_JRE:$OOI_HOME
#export OOI_HOME=$OOI_HOME

#require qeproAsyn,tomaszbrys
require qeproasyn,waynelewis

#epicsEnvSet(EPICS_CA_ADDR_LIST,10.0.2.15)
 epicsEnvSet(EPICS_CA_MAX_ARRAY_BYTES,64000000)

epicsEnvSet("PREFIX", "Sp1")
epicsEnvSet("PORT",   "Flame")
epicsEnvSet("SIZE",   "3648")
epicsEnvSet("LASER",  "478")

drvUSBQEProConfigure("$(PORT)","$(SIZE)")

asynSetTraceMask("$(PORT)", -1, 0x9)
asynSetTraceIOMask("$(PORT)", -1, 0x2)

dbLoadRecords(qepro.template, "PREFIX=$(PREFIX), PORT=$(PORT), ADDR=0, TIMEOUT=1, SIZE=$(SIZE), LASER=$(LASER)")



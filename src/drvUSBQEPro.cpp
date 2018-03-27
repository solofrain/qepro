#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include <epicsExport.h>

#include <libusb-1.0/libusb.h>

#include "drvUSBQEPro.h"


#define NUM_QEPRO_PARAMS ((int)(&LAST_QEPRO_PARAM - &FIRST_QEPRO_PARAM + 1))

static const char *driverName="drvUSBQEPro";

int drvUSBQEPro::zeroIndex = 0;
bool drvUSBQEPro::connected = false;

drvUSBQEPro::drvUSBQEPro(const char *portName, int maxPoints)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    (int)NUM_QEPRO_PARAMS,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    asynStatus status;
    const char *functionName = "drvUSBQEPro";

    // Initialize connection status
    drvUSBQEPro::connected = false;

   eventId = epicsEventCreate(epicsEventEmpty);
   createParam( QEProNumSpecs,            asynParamInt32,        &P_numSpecs);		  //  0 
   createParam( QEProId,                  asynParamInt32,        &P_nrBoard);		  //  1
   createParam( QEProName,                asynParamOctet,        &P_name);		  //  2
   createParam( QEProFirmwareVersion,     asynParamOctet,        &P_firmwareVersion);	  //  3
   createParam( QEProFirmwareModel,       asynParamOctet,        &P_firmwareModel);       //  4  
   createParam( QEProSerialNumber,        asynParamOctet,        &P_serialNumber);	  //  5
   createParam( QEProNumberOfPixels,      asynParamInt32,        &P_numberOfPixels);	  //  6
   createParam( QEProNumberOfDarkPixels,  asynParamInt32,        &P_numberOfDarkPixels);  //  7
   createParam( QEProIntegrationTime,     asynParamInt32,        &P_integrationTime); 	  //  8
   createParam( QEProMaxIntegrationTime,  asynParamInt32,        &P_maxIntegrationTime);  //  9
   createParam( QEProMinIntegrationTime,  asynParamInt32,        &P_minIntegrationTime);  // 10
   createParam( QEProMaxIntensity,  	  asynParamInt32,        &P_maxIntensity);        // 11
   createParam( QEProBoxcarWidth,  	  asynParamInt32,        &P_boxcarWidth);         // 12
   createParam( QEProElectricDark,  	  asynParamInt32,        &P_electricDark);        // 13
   createParam( QEProDetectorTemperature, asynParamInt32,        &P_detectorTemperature); // 14
   createParam( QEProBoardTemperature,    asynParamInt32,        &P_boardTemperature);    // 15
   createParam( QEProTempSetPoint,        asynParamInt32,        &P_tempSetPoint);        // 16
   createParam( QEProTriggerMode,         asynParamInt32,        &P_triggerMode);         // 17
   createParam( QEProNonLinearity,	  asynParamInt32,        &P_nonLinearity);        // 18
   createParam( QEProDecouple,            asynParamInt32,        &P_decouple);            // 19
   createParam( QEProLEDIndicator,        asynParamInt32,        &P_ledIndicator);        // 20
   createParam( QEProAverages,            asynParamInt32,        &P_averages);            // 21  
   createParam( QEProXAxisNm,             asynParamFloat64Array, &P_xAxisNm);             // 22
   createParam( QEProXAxisRs,             asynParamFloat64Array, &P_xAxisRs);             // 23
   createParam( QEProSpectrum,            asynParamFloat64Array, &P_spectrum);            // 24 
   createParam( QEProLaser,               asynParamFloat64,      &P_laser);               // 25 


   status = connectSpec();
   if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s:  camera connection failed (%d)\n", driverName, functionName, status);
        // Call report() to get a list of available cameras
        report(stdout, 1);
        return;
   }
    
    
}
//-----------------------------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::connectSpec(){

    int nrSpecs;
    asynStatus status = asynSuccess;
    static const char *functionName = "ConnectSpectrometer";

    // Set up USB hotplug callbacks
    registerUSBCallbacks();

    nrSpecs = wrapper.openAllSpectrometers();
    if (nrSpecs == -1){ 
         JString Jname =  wrapper.getLastException();
         asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s:%s:\n",  driverName, functionName, Jname.getASCII()) ;
         drvUSBQEPro::connected = false;
         return asynError;
    }

    if (nrSpecs == 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: no spectrometers found\n", driverName, functionName);
        drvUSBQEPro::connected = false;
    }
    else{
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: there are %d spectrometers\n", driverName, functionName, nrSpecs);
        drvUSBQEPro::connected = true;
        index = zeroIndex++;
        for(int i = 0; i < nrSpecs; i++){
           JString Jname = wrapper.getFirmwareModel(i);
           printf("[Info]:Spec index= %d, Model: %s\n", index, Jname.getASCII());
           } 
    } 

    return status;
}

asynStatus drvUSBQEPro::registerUSBCallbacks() {
    int rc;
    rc = libusb_init(NULL);
    if (rc < 0) {
        printf("Failed to initialise libusb: %s\n", 
                libusb_error_name(rc));
        return asynError;
    }

    rc = libusb_hotplug_register_callback(
            NULL, 
            LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
            (libusb_hotplug_flag) 0,
            OOI_VENDOR_ID,
            LIBUSB_HOTPLUG_MATCH_ANY,
            LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback,
            NULL, 
            &hp[0]);
    if (LIBUSB_SUCCESS != rc) {
        printf("Error registering connect callback\n");
        libusb_exit(NULL);
        return asynError;
    }

    rc = libusb_hotplug_register_callback(
            NULL, 
            LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
            (libusb_hotplug_flag) 0,
            OOI_VENDOR_ID,
            LIBUSB_HOTPLUG_MATCH_ANY,
            LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback_detach,
            NULL, 
            &hp[0]);
    if (LIBUSB_SUCCESS != rc) {
        printf("Error registering connect callback\n");
        libusb_exit(NULL);
        return asynError;
    }


    return asynSuccess;
}

//-----------------------------------------------------------------------------------------------------------------
asynStatus drvUSBQEPro::disconnectSpec(){
    asynStatus status = asynSuccess;
    return status;
}

//-----------------------------------------------------------------------------------------------------------------
asynStatus drvUSBQEPro::readStatus(){
    asynStatus status = asynSuccess;
    return status;
}
        
//--------------------------------------------------------------------------------------------
asynStatus drvUSBQEPro::readOctet (asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){
 
   int addr=0;
   int function = pasynUser->reason;
   asynStatus status = asynSuccess;
//   int index ;
//   const char *paramName;
//   getParamName(function, &paramName);
//   static const char *functionName = "readOctet";

    this->getAddress(pasynUser, &addr);

//    getIntegerParam(addr, nrBoard, &index);

    if (function == P_name) {
       JString Jname = wrapper.getName(index);
       strcpy(value, Jname.getASCII());
       *nActual = strlen(Jname.getASCII());
       *eomReason = 0;
    } else if (function == P_firmwareVersion) {
       JString Jname = wrapper.getFirmwareVersion(index);
       strcpy(value, Jname.getASCII());
       *nActual = strlen(Jname.getASCII());
       *eomReason = 0;

    } else if (function == P_firmwareModel) {
       JString Jname = wrapper.getFirmwareModel(index);
       strcpy(value, Jname.getASCII());
       *nActual = strlen(Jname.getASCII());
       *eomReason = 0;

    } else if (function == P_serialNumber) {
       JString Jname = wrapper.getSerialNumber(index);
       strcpy(value, Jname.getASCII());
       *nActual = strlen(Jname.getASCII());
       *eomReason = 0;


    } else {
        // All other parameters just get set in parameter list, no need to
        //  act on them here 
    }



    callParamCallbacks(addr);
    return status;
}
//--------------------------------------------------------------------------------------------
asynStatus drvUSBQEPro::readInt32 (asynUser *pasynUser, epicsInt32 *value){

    int addr;
    int function;
    asynStatus status = asynSuccess;
    int rval; 
    const char* functionName = "readInt32";


    function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);
//    getIntegerParam(addr, nrBoard, &index);

    if (function == P_numSpecs) {
       rval = wrapper.getNumberOfSpectrometersFound();
       *value = rval;
       setIntegerParam(addr, P_numSpecs, *value);
    }

    else if (function == P_numberOfPixels) {
       rval = wrapper.getNumberOfPixels(index);
       *value = rval;
       setIntegerParam(addr, P_numberOfPixels, *value);
    }

    else if (function == P_maxIntensity) {
       rval = wrapper.getMaximumIntensity(index);
       *value = rval;
       setIntegerParam(addr, P_maxIntensity, *value);
    }

    else if (function == P_numberOfDarkPixels) {
       rval = wrapper.getNumberOfDarkPixels(index);
       *value = rval;
       setIntegerParam(addr, P_numberOfDarkPixels, *value);
    }

    else if (function == P_integrationTime) {
       rval = wrapper.getIntegrationTime(index);
       int b =  wrapper.getMinimumIntegrationTime(index);

        if(rval < b){
           wrapper.setIntegrationTime(index, b);
           rval = wrapper.getIntegrationTime(index);
        }

       rval /= 1000; // function return in microseconds, we want to have in miliseconds
       *value = rval;
       setIntegerParam(addr, P_integrationTime, *value);
    }
    else if (function == P_maxIntegrationTime) {
       rval = wrapper.getMaximumIntegrationTime(index);
       *value = rval;
       setIntegerParam(addr, P_maxIntegrationTime, *value);
    }
    else if (function == P_minIntegrationTime) {
       rval = wrapper.getMinimumIntegrationTime(index);
       *value = rval;
       setIntegerParam(addr, P_minIntegrationTime, *value);
    }

    else if (function == P_boxcarWidth) {
       rval = wrapper.getBoxcarWidth(index);
       *value = rval;
       setIntegerParam(addr, P_boxcarWidth, *value);
    }
// this is only 0/1 value, should be in readInt32Digital???
    else if (function == P_electricDark) {
       rval = wrapper.getCorrectForElectricalDark(index);
       *value = rval;
       setIntegerParam(addr, P_boxcarWidth, *value);
    }

    else if (function == P_detectorTemperature) {
        if(wrapper.isFeatureSupportedThermoElectric(index)){
           ThermoElectricWrapper thermoElectric;
           thermoElectric = wrapper.getFeatureControllerThermoElectric(index);
           rval = thermoElectric.getDetectorTemperatureCelsius();
           *value = rval;
           setIntegerParam(addr, P_detectorTemperature, *value);
        } 
    }


    else if (function == P_boardTemperature) {
        if (wrapper.isFeatureSupportedBoardTemperature(index)) {
           BoardTemperature boardTemp;
           boardTemp = wrapper.getFeatureControllerBoardTemperature(index);
           rval = boardTemp.getBoardTemperatureCelsius();
           *value = rval;
           setIntegerParam(addr, P_boardTemperature, *value);
        } 
    }

   else if (function == P_triggerMode) {
       rval = wrapper.getExternalTriggerMode(index);
       *value = rval;
       setIntegerParam(addr, P_triggerMode, *value);
    }

    else if (function == P_averages) {
       rval = wrapper.getScansToAverage(index);
       *value = rval;
       setIntegerParam(addr, P_averages, *value);
    }
    else if (function == P_decouple) {
       rval = wrapper.getScansToAverage(index);
       *value = rval;
       setIntegerParam(addr, P_decouple, *value);
    }
    else if (function == P_ledIndicator) {
       rval = wrapper.getScansToAverage(index);
       *value = rval;
       setIntegerParam(addr, P_ledIndicator, *value);
    }
    else if (function == P_nonLinearity) {
       rval = wrapper.getScansToAverage(index);
       *value = rval;
       setIntegerParam(addr, P_nonLinearity, *value);
    }

    else {
      status = asynPortDriver::readInt32(pasynUser, value); 
    }

    callParamCallbacks(addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%d, addr=%d, status=%d\n",
                  driverName, functionName, this->portName, *value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%d, addr=%d\n",
                  driverName, functionName, this->portName, *value, addr);
    return status;
    
return status;
}

//--------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::readFloat64Array (asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn ){

    //const char* functionName = "readFloat64Array";
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr;
    int arraySize; 
    double *wavelengths;
    double laser;
 
    this->getAddress(pasynUser, &addr);

    if (function == P_xAxisNm) {
        DoubleArray arrayX = wrapper.getWavelengths(index);
        wavelengths = arrayX.getDoubleValues();
        arraySize = arrayX.getLength();
         
        for(int i = 0; i < arraySize; i++) {
                value[i] = wavelengths[i];
        }

      //  value = arrayX.getDoubleValues();
        *nIn = arraySize;
    }
    else if (function == P_xAxisRs) {
        getDoubleParam(addr, P_laser, &laser);
        DoubleArray arrayX = wrapper.getWavelengths(index);
        wavelengths = arrayX.getDoubleValues();
        arraySize = arrayX.getLength();
         
        for(int i = 0; i < arraySize; i++) {
                value[i] = (1./laser - 1./wavelengths[i]) *10e7; // Raman shift in cm-1
        }

       // value = arrayX.getDoubleValues();
        *nIn = arraySize;
 
    }


return status;
}
 
//--------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeInt32";
    int addr;
    int boxcar, averages, electricDark, nonLinearity, triggerMode;
    int decouple, ledIndicator, integrationTime;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    this->getAddress(pasynUser, &addr);
//    getIntegerParam(addr, nrBoard, &index);


    if (function == P_integrationTime) {
        getIntegerParam(P_integrationTime, &integrationTime);
        wrapper.setBoxcarWidth(index, integrationTime);
    }
    else if (function == P_averages) {
        getIntegerParam(P_averages, &averages);
        wrapper.setScansToAverage(index, boxcar);
    }
    else if (function == P_boxcarWidth) {
        getIntegerParam(P_boxcarWidth, &boxcar);
        wrapper.setBoxcarWidth(index, boxcar);
    }
    else if (function == P_electricDark) {
        getIntegerParam(P_electricDark, &electricDark);
        wrapper.setCorrectForElectricalDark(index, electricDark);
    }
    else if (function == P_nonLinearity) {
        getIntegerParam(P_nonLinearity, &nonLinearity);
        bool retval = wrapper.setCorrectForDetectorNonlinearity(index, nonLinearity);
        if(retval == 0){
           setIntegerParam(P_nonLinearity, value);
	   return asynError;
        }

    }
    else if (function == P_triggerMode) {
        getIntegerParam(P_triggerMode, &triggerMode);
        wrapper.setExternalTriggerMode(index, triggerMode);
    }
//    in this version setPoint is not supported....
//    other parameters for controling temperature and fan also not supported....
//    else if (function == setPonit) {
//        ThermoElectricWrapper thermoElectric = getFeatureControllerThermoElectric(index);
//        if(thermoElectric)
//           getIntegerParam(addr, setPoint, &setPoint);
//           thermoElectric.setTECEnable(true);
//           thermoElectric.setDetectorSetPointCelsius(temp);
//        } else return statusError;
//    }
    else if (function == decouple) {
        getIntegerParam(P_decouple, &decouple);
        wrapper.setBoxcarWidth(index, decouple);
    }
    else if (function == ledIndicator) {
        getIntegerParam(P_ledIndicator, &ledIndicator);
        wrapper.setBoxcarWidth(index, ledIndicator);
    }

    else {
        /* All other parameters just get set in parameter list, no need to
 *          * act on them here */
    }

    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%d, addr=%d, status=%d\n",
                  driverName, functionName, this->portName, value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%d, addr=%d\n",
                  driverName, functionName, this->portName, value, addr);
    return status;

}

int LIBUSB_CALL drvUSBQEPro::hotplug_callback(
        libusb_context *ctx, 
        libusb_device *dev, 
        libusb_hotplug_event event, 
        void *user_data)
{
	//(void)ctx;
	//(void)dev;
	//(void)event;
	//(void)user_data;

	printf ("Device attached\n");

    drvUSBQEPro::connected = true;

	return 0;
}

int LIBUSB_CALL drvUSBQEPro::hotplug_callback_detach(
        libusb_context *ctx, 
        libusb_device *dev, 
        libusb_hotplug_event event, 
        void *user_data)
{
	//(void)ctx;
	//(void)dev;
	//(void)event;
	//(void)user_data;

	printf ("Device detached\n");

    drvUSBQEPro::connected = false;

	return 0;
}

extern "C" {

/** EPICS iocsh callable function to call constructor for the drvUSBQEPro class.
 *   * \param[in] portName The name of the asyn port driver to be created.
 *     * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
int drvUSBQEProConfigure(const char *portName, int maxPoints) {
    new drvUSBQEPro(portName, maxPoints);
    return(asynSuccess);
}


/* EPICS iocsh shell commands */

static const iocshArg initArg0 = { "portName", iocshArgString};
static const iocshArg initArg1 = { "max points",iocshArgInt};
static const iocshArg * const initArgs[] = { 
					   &initArg0,
                                           &initArg1 
					   };

static const iocshFuncDef initFuncDef = {"drvUSBQEProConfigure",2,initArgs};

static void initCallFunc(const iocshArgBuf *args) { 
    drvUSBQEProConfigure(args[0].sval, args[1].ival);
}

void drvUSBQEProRegister(void) {
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvUSBQEProRegister);

}




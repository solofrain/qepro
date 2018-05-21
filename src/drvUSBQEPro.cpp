#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <epicsExport.h>

#include <libusb-1.0/libusb.h>

#include "drvUSBQEPro.h"
#include "api/seabreezeapi/SeaBreezeAPI.h"

#define NUM_QEPRO_PARAMS ((int)(&LAST_QEPRO_PARAM - &FIRST_QEPRO_PARAM + 1))

static const char *driverName="drvUSBQEPro";

int drvUSBQEPro::zeroIndex = 0;
//bool drvUSBQEPro::connected = false;

drvUSBQEPro::drvUSBQEPro(const char *portName, int maxPoints, double laser)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    (int)NUM_QEPRO_PARAMS,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/
    m_laser(laser),
    m_poll_time(POLL_TIME)
{
    asynStatus status;
    const char *functionName = "drvUSBQEPro";

    // Initialize connection status
    connected = false;

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
    createParam( QEProMaxIntensity,  	  asynParamFloat64,      &P_maxIntensity);        // 11
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
        asynPrint(
                pasynUserSelf, 
                ASYN_TRACE_ERROR, 
                "%s:%s:  spectrometer connection failed (%d)\n", 
                driverName, 
                functionName, 
                status);
        report(stdout, 1);
        return;
    }

    epicsThreadCreate("drvUSBQEProThread",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)worker,
            this);
}
//-----------------------------------------------------------------------------------------------------------------

void drvUSBQEPro::getSpectrumThread(void *priv){

    //drvUSBQEPro *ptr = (drvUSBQEPro *)priv;
    int arrayXElements = 0;

    //static const char *functionName = "getSpectrumThread";
    //while(m_condition){
    while(1){
      lock();

      DoubleArray da = wrapper.getSpectrum(index);

      if( wrapper.isSpectrumValid(index) != FALSE && wrapper.isSaturated(index) != TRUE){

         double *values = da.getDoubleValues();
         arrayXElements = da.getLength();
         for(int i = 0; i < arrayXElements; i++) {
             m_dataSpectrum[i] = values[i];
         }
      }
      unlock();

      doCallbacksFloat64Array(m_dataSpectrum, arrayXElements, P_spectrum, 0);

      epicsThreadSleep(m_poll_time);
    }
}

asynStatus drvUSBQEPro::connectSpec(){
    asynStatus status = asynSuccess;
    //static const char *functionName = "ConnectSpectrometer";

    // Set up USB hotplug callbacks
    registerUSBCallbacks();

    api = SeaBreezeAPI::getInstance();

    // Set up the seabreeze driver
    //sbapi_initialize();
    api->probeDevices();

    test_connection();

    allocate_spectrum_buffer();

    return status;
}

void drvUSBQEPro::allocate_spectrum_buffer() {
    if (connected) {
        num_pixels = api->spectrometerGetFormattedSpectrumLength(
                device_id,
                spectrometer_feature_id,
                &error);
        spectrum_buffer = (double *)calloc(num_pixels * sizeof(double));
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
    else {
        printf("Registered USB connect callback\n");
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
        printf("Error registering detach callback\n");
        libusb_exit(NULL);
        return asynError;
    }
    else {
        printf("Registered USB detach callback\n");
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

    this->getAddress(pasynUser, &addr);

    char buffer[80];
    int error;

    // TODO: Add connection check.
    // TODO: Set invalid if disconnected.
    if (drvUSBQEPro::connected) {
        if (function == P_name) {
            //sbapi_get_device_type(device_id, &error, buffer, 79);
            api->getDeviceType(device_id, &error, buffer, 79);
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
        } 
        else if (function == P_firmwareVersion) {
            strcpy(buffer, "Not available");
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
        } 
        else if (function == P_firmwareModel) {
            strcpy(buffer, "Not available");
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
        } 
        else if (function == P_serialNumber) {
            //sbapi_get_serial_number(
            api->getSerialNumber(
                    device_id,
                    serial_number_feature_id,
                    &error,
                    buffer,
                    79);
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
        } else {
            // All other parameters just get set in parameter list, no need to
            //  act on them here 
        }
    }
    else {
        // TODO: Set PV invalid here
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
    int error;

    function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);

    if (function == P_numSpecs) {
        //rval = sbapi_get_number_of_device_ids();
        rval = api->getNumberOfDeviceIDs();
        *value = rval;
        setIntegerParam(addr, P_numSpecs, *value);
    }

    else if (function == P_numberOfPixels) {
        //rval = sbapi_spectrometer_get_formatted_spectrum_length(
        rval = api->spectrometerGetFormattedSpectrumLength(
                device_id,
                spectrometer_feature_id,
                &error);
        *value = rval;
        setIntegerParam(addr, P_numberOfPixels, *value);
        num_pixels = rval;
    }

    else if (function == P_numberOfDarkPixels) {
        //rval = sbapi_spectrometer_get_electric_dark_pixel_count(
        rval = api->spectrometerGetElectricDarkPixelCount(
                device_id,
                spectrometer_feature_id,
                &error);
        *value = rval;
        setIntegerParam(addr, P_numberOfDarkPixels, *value);
    }

    else if (function == P_integrationTime) {
        rval = integration_time;
        // function return in microseconds, we want to have in miliseconds
        rval /= 1000; 
        *value = rval;
        setIntegerParam(addr, P_integrationTime, *value);
    }
    else if (function == P_maxIntegrationTime) {
        rval = api->spectrometerGetMaximumIntegrationTimeMicros(
                device_id,
                spectrometer_feature_id,
                &error);
        // function return in microseconds, we want to have in miliseconds
        rval /= 1000; 
        *value = rval;
        setIntegerParam(addr, P_maxIntegrationTime, *value); 
    }
    else if (function == P_minIntegrationTime) {
        //rval = sbapi_spectrometer_get_minimum_integration_time_micros(
        rval = api->spectrometerGetMinimumIntegrationTimeMicros(
                device_id,
                spectrometer_feature_id,
                &error);
        // function return in microseconds, we want to have in miliseconds
        rval /= 1000; 
        *value = rval;
        setIntegerParam(addr, P_minIntegrationTime, *value); 
    }
    else if (function == P_boxcarWidth) {
        // TODO: Investigate whether device supports boxcar averaging
        *value = 0;
        setIntegerParam(addr, P_triggerMode, *value);
    }
    // this is only 0/1 value, should be in readInt32Digital???
    else if (function == P_electricDark) {
        // TODO: Investigate dark correction
        //rval = wrapper.getCorrectForElectricalDark(index);
        //*value = rval;
        //setIntegerParam(addr, P_electricDark, *value);
    }

    else if (function == P_detectorTemperature) {
        // TODO: Investigate this more. test program shows no temperatures.
        // Feature not implemented in QEPro
        *value = 0;
        setIntegerParam(addr, P_triggerMode, *value);
    }

    else if (function == P_boardTemperature) {
        // TODO: Investigate this more. test program shows no temperatures.
        // Feature not implemented in QEPro
        *value = 0;
        setIntegerParam(addr, P_triggerMode, *value);
    }

    else if (function == P_triggerMode) {
        rval = trigger_mode;
        *value = rval;
        setIntegerParam(addr, P_triggerMode, *value);
    }

    else if (function == P_averages) {
        // Feature not implemented in QEPro
        *value = 0;
        setIntegerParam(addr, P_triggerMode, *value);
    }
    else if (function == P_decouple) {
        // Feature not implemented in QEPro
        *value = 0;
        setIntegerParam(addr, P_triggerMode, *value);
    }
    else if (function == P_ledIndicator) {
        *value = 0;
        setIntegerParam(addr, P_ledIndicator, *value);
    }

    else {
        status = asynPortDriver::readInt32(pasynUser, value); 
    }

    // TODO: Add connection test and set of PV status
    // to invalid if disconnected

    callParamCallbacks(addr);

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%d, addr=%d, status=%d\n",
                driverName, functionName, this->portName, *value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%d, addr=%d\n",
                driverName, functionName, this->portName, *value, addr);

    return status;
}

//--------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::readFloat64(asynUser *pasynUser, epicsFloat64 *value){
    int addr;
    int function;
    asynStatus status = asynSuccess;
    double rval; 
    const char* functionName = "readFloat64";
    int error;

    function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);

    // TODO: Add connection test
    //
    if (function == P_maxIntensity) {
        //rval = sbapi_spectrometer_get_maximum_intensity(
        rval = api->spectrometerGetMaximumIntensity(
                device_id,
                spectrometer_feature_id,
                &error);
        *value = rval;
        setDoubleParam(addr, P_maxIntensity, *value);
    }
    else if (function == P_nonLinearity) {
        double buffer;
        //sbapi_nonlinearity_coeffs_get(
        api->nonlinearityCoeffsGet(
                device_id,
                nonlinearity_feature_id,
                &error,
                &buffer,
                1);
        *value = buffer;
        setDoubleParam(addr, P_nonLinearity, *value);
    }
    else {
        status = asynPortDriver::readFloat64(pasynUser, value); 
    }


    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%f, addr=%d, status=%d\n",
                driverName, functionName, this->portName, *value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%f, addr=%d\n",
                driverName, functionName, this->portName, *value, addr);

    return status;
}

asynStatus drvUSBQEPro::readFloat64Array (asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn ){

    const char* functionName = "readFloat64Array";
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr;
    int error;
    double laser = 400e-9;

    this->getAddress(pasynUser, &addr);

    if (function == P_xAxisNm) {
        double *wavelengths = (double *)calloc(num_pixels, sizeof(double));
        int num_wavelengths = api->spectrometerGetWavelengths(
                device_id, 
                spectrometer_feature_id, 
                &error, 
                wavelengths, 
                num_pixels);

           for(int i = 0; i < num_wavelengths; i++)
               value[i] = wavelengths[i];

         *nIn = num_wavelengths;
    }
    else if (function == P_xAxisRs) {
        double *wavelengths = (double *)calloc(num_pixels, sizeof(double));
        int num_wavelengths = api->spectrometerGetWavelengths(
                device_id, 
                spectrometer_feature_id, 
                &error, 
                wavelengths, 
                num_pixels);

        for(int i = 0; i < num_wavelengths; i++)
            value[i] = (1./laser - 1./wavelengths[i]) *10e7; // Raman shift in cm-1

        *nIn = num_wavelengths;
    }
    else if (function == P_spectrum) {
        // Ensure we are getting a consistent data set
        lock();

        memcpy(value, spectrum_buffer, num_pixels * sizeof(double));
        *nIn = num_pixels;
        
        unlock();
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%f, addr=%d, status=%d\n",
                driverName, functionName, this->portName, *value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%f, addr=%d\n",
                driverName, functionName, this->portName, *value, addr);

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
    int error;
    int boxcar, averages, electricDark, nonLinearity, triggerMode;
    int decouple, ledIndicator;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    this->getAddress(pasynUser, &addr);
    //    getIntegerParam(addr, nrBoard, &index);

    if (function == P_integrationTime) {
        int tmp;
        getIntegerParam(P_integrationTime, &tmp);
        // Assume we are getting the integration time in ms
        integration_time = tmp * 1000;
        //sbapi_spectrometer_set_integration_time_micros(
        api->spectrometerSetIntegrationTimeMicros(
                device_id,
                spectrometer_feature_id,
                &error,
                integration_time);
    }
    else if (function == P_averages) {
        // Check if implmemented in QEPro
    }
    else if (function == P_boxcarWidth) {
        // Check if implmemented in QEPro
    }
    else if (function == P_electricDark) {
        // Check if implmemented in QEPro
        //getIntegerParam(P_electricDark, &electricDark);
        //wrapper.setCorrectForElectricalDark(index, electricDark);
    }
    else if (function == P_nonLinearity) {
        getIntegerParam(P_nonLinearity, &nonLinearity);
        //bool retval = wrapper.setCorrectForDetectorNonlinearity(index, nonLinearity);
        //if(retval == 0){
        //setIntegerParam(P_nonLinearity, value);
        //return asynError;
        //}

    }
    else if (function == P_triggerMode) {
        getIntegerParam(P_triggerMode, &triggerMode);
        api->spectrometerSetTriggerMode(
                device_id,
                spectrometer_feature_id,
                &error,
                triggerMode);
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
    //wrapper.setBoxcarWidth(index, decouple);
}
else if (function == ledIndicator) {
    getIntegerParam(P_ledIndicator, &ledIndicator);
    //wrapper.setBoxcarWidth(index, ledIndicator);
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

// Test whether spectrometer is connected. If it is was previously
// disconnected, read the various id values and store to avoid needing to 
// read each time.
void drvUSBQEPro::test_connection() {
    int error;

    api->probeDevices();
    int number_of_devices = api->getNumberOfDeviceIDs();
    if (number_of_devices == 0) 
        connected = false;
    else {
        // Check if we were disconnected previously 
        if (!connected) {
            // Read device IDs
            long * device_ids = (long *)calloc(number_of_devices, sizeof(long));
            //sbapi_get_device_ids(device_ids, number_of_devices);
            api->getDeviceIDs(device_ids, number_of_devices);
            // Assume only one device
            device_id = device_ids[0];

            // Read spectrometer feature ID
            int num_spectrometer_features = 
                api->getNumberOfSpectrometerFeatures(device_id, &error);
            //sbapi_get_number_of_spectrometer_features(device_id, &error);

            if (num_spectrometer_features > 0) {
                long * spectrometer_feature_ids = 
                    (long *)calloc(num_spectrometer_features, sizeof(long));

                //sbapi_get_spectrometer_features(
                api->getSpectrometerFeatures(
                        device_ids[0],
                        &error,
                        spectrometer_feature_ids, 
                        num_spectrometer_features);

                spectrometer_feature_id = spectrometer_feature_ids[0];
            }

            // Read serial number feature ID
            int num_serial_number_features = 
                //sbapi_get_number_of_serial_number_features(
                api->getNumberOfSerialNumberFeatures(
                        device_id, 
                        &error);

            if (num_serial_number_features > 0) {
                long * serial_number_feature_ids = 
                    (long *)calloc(
                            num_serial_number_features, 
                            sizeof(long));

                //sbapi_get_serial_number_features(
                api->getSerialNumberFeatures(
                        device_id,
                        &error,
                        serial_number_feature_ids, 
                        num_serial_number_features);

                serial_number_feature_id = 
                    serial_number_feature_ids[0];
            }

            // Read non-linearity coefficients feature ID
            int num_nonlinearity_features = 
                //sbapi_get_number_of_nonlinearity_coeffs_features(
                api->getNumberOfNonlinearityCoeffsFeatures(
                        device_id, 
                        &error);

            if (num_nonlinearity_features > 0) {
                long * nonlinearity_feature_ids = 
                    (long *)calloc(
                            num_nonlinearity_features, 
                            sizeof(long));

                //sbapi_get_nonlinearity_coeffs_features(
                api->getNonlinearityCoeffsFeatures(
                        device_id,
                        &error,
                        nonlinearity_feature_ids, 
                        num_nonlinearity_features);

                nonlinearity_feature_id = 
                    nonlinearity_feature_ids[0];
            }

            allocate_spectrum_buffer();

            connected = true;
        }
    }
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
    //printf ("Device connected = %d\n", drvUSBQEPro::connected);
    //drvUSBQEPro::connected = true;
    //printf ("Device connected = %d\n", drvUSBQEPro::connected);

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
    //printf ("Device connected = %d\n", drvUSBQEPro::connected);
    //drvUSBQEPro::connected = false;
    //printf ("Device connected = %d\n", drvUSBQEPro::connected);

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




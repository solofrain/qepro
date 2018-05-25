#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>

#include "drvUSBQEPro.h"
#include "drvUSBQEProWorker.h"
#include "api/SeaBreezeWrapper.h"

#define NUM_QEPRO_PARAMS ((int)(&LAST_QEPRO_PARAM - &FIRST_QEPRO_PARAM + 1))

static const char *driverName="drvUSBQEPro";

int drvUSBQEPro::zeroIndex = 0;

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
    spec_index = 0;

    // Initialize connection status
    connected = false;

    //eventId = epicsEventCreate(epicsEventEmpty);
    createParam( QEProNumSpecs,             asynParamInt32,         &P_numSpecs);		    //  0 
    createParam( QEProId,                   asynParamInt32,         &P_nrBoard);		    //  1
    createParam( QEProName,                 asynParamOctet,         &P_name);		        //  2
    createParam( QEProFirmwareVersion,      asynParamOctet,         &P_firmwareVersion);    //  3
    createParam( QEProFirmwareModel,        asynParamOctet,         &P_firmwareModel);      //  4  
    createParam( QEProSerialNumber,         asynParamOctet,         &P_serialNumber);	    //  5
    createParam( QEProNumberOfPixels,       asynParamInt32,         &P_numberOfPixels);	    //  6
    createParam( QEProNumberOfDarkPixels,   asynParamInt32,         &P_numberOfDarkPixels); //  7
    createParam( QEProIntegrationTime,      asynParamInt32,         &P_integrationTime);    //  8
    createParam( QEProMaxIntegrationTime,   asynParamInt32,         &P_maxIntegrationTime); //  9
    createParam( QEProMinIntegrationTime,   asynParamInt32,         &P_minIntegrationTime); // 10
    createParam( QEProMaxIntensity,  	    asynParamFloat64,       &P_maxIntensity);       // 11
    createParam( QEProBoxcarWidth,  	    asynParamInt32,         &P_boxcarWidth);        // 12
    createParam( QEProElectricDark,  	    asynParamInt32,         &P_electricDark);       // 13
    createParam( QEProDetectorTemperature,  asynParamInt32,         &P_detectorTemperature);// 14
    createParam( QEProBoardTemperature,     asynParamInt32,         &P_boardTemperature);   // 15
    createParam( QEProTempSetPoint,         asynParamInt32,         &P_tempSetPoint);       // 16
    createParam( QEProTriggerMode,          asynParamInt32,         &P_triggerMode);        // 17
    createParam( QEProNonLinearity,	        asynParamInt32,         &P_nonLinearity);       // 18
    createParam( QEProDecouple,             asynParamInt32,         &P_decouple);           // 19
    createParam( QEProLEDIndicator,         asynParamInt32,         &P_ledIndicator);       // 20
    createParam( QEProAverages,             asynParamInt32,         &P_averages);           // 21  
    createParam( QEProXAxisNm,              asynParamFloat64Array,  &P_xAxisNm);            // 22
    createParam( QEProXAxisRs,              asynParamFloat64Array,  &P_xAxisRs);            // 23
    createParam( QEProSpectrum,             asynParamFloat64Array,  &P_spectrum);           // 24 
    createParam( QEProLaser,                asynParamFloat64,       &P_laser);              // 25 
    createParam( QEProConnected,            asynParamInt32,         &P_connected);          // 26
    createParam( QEProAcqMode,              asynParamInt32,         &P_acqMode);            // 27
    createParam( QEProAcqStart,             asynParamInt32,         &P_acqStart);           // 28
    createParam( QEProAcqStop,              asynParamInt32,         &P_acqStop);            // 29
    createParam( QEProAcqSts,               asynParamInt32,         &P_acqSts);             // 30
    createParam( QEProFileWrite,            asynParamInt32,         &P_fileWrite);          // 31
    createParam( QEProFilePath,             asynParamOctet,         &P_filePath);           // 32
    createParam( QEProFileName,             asynParamOctet,         &P_fileName);           // 33
    createParam( QEProFullFileName,         asynParamOctet,         &P_fullFileName);       // 34
    createParam( QEProFullFilePath,         asynParamOctet,         &P_fullFilePath);       // 35
    createParam( QEProFileIndex,            asynParamInt32,         &P_fileIndex);          // 36
    createParam( QEProXAxisMode,            asynParamInt32,         &P_xAxisMode);          // 37
    createParam( QEProXAxis,                asynParamFloat64Array,  &P_xAxis);              // 38
    createParam( QEProCPUTemperature,       asynParamFloat64,       &P_cpuTemperature);     // 39
    createParam( QEProPCBTemperature,       asynParamFloat64,       &P_pcbTemperature);     // 40
    createParam( QEProDetTemperature,       asynParamFloat64,       &P_detTemperature);     // 41
    createParam( QEProROI0LowWavelength,    asynParamFloat64,       &P_roi0LowWavelength);  // 42
    createParam( QEProROI0HighWavelength,   asynParamFloat64,       &P_roi0HighWavelength); // 43
    createParam( QEProROI1LowWavelength,    asynParamFloat64,       &P_roi1LowWavelength);  // 44
    createParam( QEProROI1HighWavelength,   asynParamFloat64,       &P_roi1HighWavelength); // 45
    createParam( QEProROI0Sum,              asynParamFloat64,       &P_roi0Sum);            // 46
    createParam( QEProROI1Sum,              asynParamFloat64,       &P_roi1Sum);            // 47
    createParam( QEProROIRatio,             asynParamFloat64,       &P_roiRatio);           // 48
    createParam( QEProDarkAcq,              asynParamInt32,         &P_darkAcq);            // 49
    createParam( QEProDarkSubtract,         asynParamInt32,         &P_darkSubtract);       // 50
    createParam( QEProDarkSpectrum,         asynParamFloat64Array,  &P_darkSpectrum);       // 51
    createParam( QEProDarkValid,            asynParamInt32,         &P_darkValid);          // 52

    // Set up initial USB context. Must be done before starting thread,
    // or attempting comms to device.
    int rc = 0;
    context = NULL;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "drvUSBQEPro: init usb\n");
    rc = libusb_init(&context);
    assert(rc == 0);

    // General initialisation
    dark_valid = false;

    // Set up connection
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

    asynPrint(pasynUserSelf, 
            ASYN_TRACE_FLOW, 
            "drvUSBQEPro: create spectrum readout thread\n");
    epicsThreadCreate("drvUSBQEProThread",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)worker,
            this);
}
//-----------------------------------------------------------------------------------------------------------------

void drvUSBQEPro::getSpectrumThread(void *priv){

    int error;
    int acq_mode;
    int start;
    int stop;
    int boxcar_half_width;
    int file_write;
    int x_axis_mode;
    bool run = false;

    while(1){

        // Get acquisition mode and control status from PVs
        getIntegerParam(P_acqMode, &acq_mode);
        getIntegerParam(P_acqStart, &start);
        getIntegerParam(P_acqStop, &stop);
        getIntegerParam(P_boxcarWidth, &boxcar_half_width);
        getIntegerParam(P_fileWrite, &file_write);

        asynPrint(pasynUserSelf, 
                ASYN_TRACE_FLOW, 
                "getSpectrumThread: acquisition mode = %d\n",
                acq_mode);
        asynPrint(pasynUserSelf, 
                ASYN_TRACE_FLOW, 
                "getSpectrumThread: device_id = 0x%lx\n",
                device_id);
        asynPrint(pasynUserSelf, 
                ASYN_TRACE_FLOW, 
                "getSpectrumThread: spectrometer_feature_id = 0x%lx\n",
                spectrometer_feature_id);
        asynPrint(pasynUserSelf, 
                ASYN_TRACE_FLOW, 
                "getSpectrumThread: num_pixels = %d\n",
                num_pixels);
        asynPrint(pasynUserSelf, 
                ASYN_TRACE_FLOW, 
                "getSpectrumThread: boxcar_half_width = %d\n",
                boxcar_half_width);

        test_connection();

        if (start && 
                (acq_mode == QEPRO_ACQ_MODE_SINGLE ||
                 acq_mode == QEPRO_ACQ_MODE_CONTINUOUS )) {
            run = true;
        }
        else if (start &&
                acq_mode == QEPRO_ACQ_MODE_OFF) {
            run = false;
        }
        else if (stop) {
            run = false;
        }

        // Collect a dark spectra if requested
        if (dark_acquire) {
            test_connection();
            if (connected) {
                int min_integration_time;
                int error;

                asynPrint(pasynUserSelf, 
                        ASYN_TRACE_FLOW, 
                        "getSpectrumThread: acquiring dark spectrum\n");

                getIntegerParam(P_minIntegrationTime, &min_integration_time);

                // Restart acquisition
                abort();
                clear_buffers();
                start_acquisition();

                api->spectrometerGetFormattedSpectrum(
                        device_id, 
                        spectrometer_feature_id,
                        &error, 
                        dark_buffer, 
                        num_pixels);

                dark_valid = true;
                setIntegerParam(P_darkValid, dark_valid);
                callParamCallbacks();
                dark_acquire = false;

                // Send the data to the dark spectrum PV
                doCallbacksFloat64Array(
                        dark_buffer, 
                        num_pixels, 
                        P_darkSpectrum, 
                        0);
            }
        }

        // Decide if we should acquire or not
        if (run) {
            if (connected) {
                lock();

                // If not yet acquiring, start the detector
                start_acquisition();

                // Set acquisition status PV
                acquiring = true;
                setIntegerParam(P_acqSts, acquiring);
                callParamCallbacks();

                // NOTE: This function blocks until a new spectrum is available
                api->spectrometerGetFormattedSpectrum(
                        device_id, 
                        spectrometer_feature_id,
                        &error, 
                        spectrum_buffer, 
                        num_pixels);

                num_wavelengths = api->spectrometerGetWavelengths(
                        device_id, 
                        spectrometer_feature_id,
                        &error, 
                        wavelength_buffer, 
                        num_pixels);

                unlock();

                assert(num_wavelengths == num_pixels);

                convert_nm_to_raman_shift(
                        raman_shift_buffer, 
                        wavelength_buffer, 
                        num_wavelengths);

                doCallbacksFloat64Array(
                        raman_shift_buffer, 
                        num_wavelengths, 
                        P_xAxisRs, 
                        0);

                doCallbacksFloat64Array(
                        wavelength_buffer, 
                        num_wavelengths, 
                        P_xAxisNm, 
                        0);

                getIntegerParam(P_xAxisMode, &x_axis_mode);
                // Send the values to the PV used for the plot x axis
                if (x_axis_mode == QEPRO_XAXIS_RAMAN_SHIFT) {
                    doCallbacksFloat64Array(
                            raman_shift_buffer, 
                            num_wavelengths, 
                            P_xAxis, 
                            0);
                }
                else {
                    doCallbacksFloat64Array(
                            wavelength_buffer, 
                            num_wavelengths, 
                            P_xAxis, 
                            0);
                }


                if (dark_subtract && dark_valid) {
                    asynPrint(
                            pasynUserSelf,
                            ASYN_TRACE_FLOW,
                            "getSpectrumThread: doing dark subtraction\n");

                    for (int i = 0; i < num_pixels; i++) {
                        spectrum_buffer[i] -= dark_buffer[i];
                        if (spectrum_buffer[i] < 0)
                            spectrum_buffer[i] = 0;
                    }
                }

                integrate_rois();

                if (boxcar_half_width > 0) {
                    double *process_buffer;
                    process_buffer = (double *)calloc(
                            num_pixels, 
                            sizeof(double));
                    boxcar(spectrum_buffer, 
                            process_buffer, 
                            boxcar_half_width);
                    doCallbacksFloat64Array(
                            process_buffer, 
                            num_pixels, 
                            P_spectrum, 
                            0);
                    if (file_write) {
                        if (x_axis_mode == QEPRO_XAXIS_RAMAN_SHIFT)
                            write_file(raman_shift_buffer, process_buffer);
                        else
                            write_file(wavelength_buffer, process_buffer);
                    }
                    free(process_buffer);
                }
                else {
                    doCallbacksFloat64Array(
                            spectrum_buffer, 
                            num_pixels, 
                            P_spectrum, 
                            0);
                    if (file_write) {
                        if (x_axis_mode == QEPRO_XAXIS_RAMAN_SHIFT)
                            write_file(raman_shift_buffer, spectrum_buffer);
                        else
                            write_file(wavelength_buffer, spectrum_buffer);
                    }
                }
            }
        }
        else {
            // Stop the acquisition
            test_connection();
            if (acquiring && connected) {
                abort();
                clear_buffers();
            }
            // Set acquisition status PV
            acquiring = false;
            setIntegerParam(P_acqSts, acquiring);
            callParamCallbacks();
        }

        // Do a single acquisition
        if (acq_mode == QEPRO_ACQ_MODE_SINGLE) {
            run = false;
        }
        //TODO: Check if we actually need this
        epicsThreadSleep(m_poll_time);
    }
}

// TODO: Make this work properly with the boxcar averaging
void drvUSBQEPro::integrate_rois() {
    for (int i = 0; i < NUM_ROIS; i++) {
        roi_sum[i] = 0;
        for (int j = 0; j < num_pixels; j++) {
            if (wavelength_buffer[j] > roi_low[i] 
                    && wavelength_buffer[j] < roi_high[i]) {
                roi_sum[i] += spectrum_buffer[j];
            }
        }
    }

    roi_ratio = roi_sum[0] / roi_sum[1];

    setDoubleParam(P_roiRatio, roi_ratio);
    setDoubleParam(P_roi0Sum, roi_sum[0]);
    setDoubleParam(P_roi1Sum, roi_sum[1]);
    callParamCallbacks();
}

void drvUSBQEPro::convert_nm_to_raman_shift(
        double *raman_shift_buffer,
        const double *wavelength_buffer,
        int num_wavelengths) {
    // Calculate Raman shift in cm-1
    for(int i = 0; i < num_wavelengths; i++)
        raman_shift_buffer[i] = 
            (1./m_laser - 1./wavelength_buffer[i]) *10e7; 
}

asynStatus drvUSBQEPro::connectSpec(){
    asynStatus status = asynSuccess;

    test_connection();

    return status;
}

void drvUSBQEPro::allocate_spectrum_buffer() {
    int error;
    if (connected) {
        num_pixels = api->spectrometerGetFormattedSpectrumLength(
                device_id,
                spectrometer_feature_id,
                &error);
        spectrum_buffer = (double *)calloc(num_pixels, sizeof(double));
        wavelength_buffer = (double *)calloc(num_pixels, sizeof(double));
        raman_shift_buffer = (double *)calloc(num_pixels, sizeof(double));
        dark_buffer = (double *)calloc(num_pixels, sizeof(double));
    }
}

void drvUSBQEPro::deallocate_spectrum_buffer() {
    if (!connected) {
        if (spectrum_buffer)
            free(spectrum_buffer);
        spectrum_buffer = NULL;

        if (wavelength_buffer)
            free(wavelength_buffer);
        wavelength_buffer = NULL;

        if (raman_shift_buffer)
            free(raman_shift_buffer);
        raman_shift_buffer = NULL;

        if (dark_buffer)
            free(dark_buffer);
        dark_buffer = NULL;
    }
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
// No specific function needed for writeOctet. Use base class implementation.
// asynStatus drvUSBQEPro::writeOctet (asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){
// }

asynStatus drvUSBQEPro::readOctet (asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){

    int addr=0;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    this->getAddress(pasynUser, &addr);

    char buffer[80];
    int error;

    test_connection();
    if (connected) {
        if (function == P_name) {
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
            api->getSerialNumber(
                    device_id,
                    serial_number_feature_id,
                    &error,
                    buffer,
                    79);
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
        } 
        else {
            // All other parameters just get set in parameter list, no need to
            //  act on them here 
        }
    }
    else {
        // Use this status return if the device is not connected
        status = asynDisconnected;
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

    // Handle all device related parameters. Must test connection first
    // before attempting to read.
    test_connection();
    if (connected) {
        if (function == P_numSpecs) {
            rval = api->getNumberOfDeviceIDs();
            *value = rval;
            setIntegerParam(addr, P_numSpecs, *value);
        }

        else if (function == P_numberOfPixels) {
            rval = api->spectrometerGetFormattedSpectrumLength(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            *value = rval;
            setIntegerParam(addr, P_numberOfPixels, *value);
            num_pixels = rval;
        }

        else if (function == P_numberOfDarkPixels) {
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
            rval = api->spectrometerGetMinimumIntegrationTimeMicros(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            // function return in microseconds, we want to have in miliseconds
            rval /= 1000; 
            *value = rval;
            setIntegerParam(addr, P_minIntegrationTime, *value); 
        }
        // this is only 0/1 value, should be in readInt32Digital???
        else if (function == P_electricDark) {
            // TODO: Investigate dark correction
            //rval = wrapper.getCorrectForElectricalDark(index);
            //*value = rval;
            //setIntegerParam(addr, P_electricDark, *value);
        }


        else if (function == P_triggerMode) {
            rval = trigger_mode;
            *value = rval;
            setIntegerParam(addr, P_triggerMode, *value);
        }

        else if (function == P_averages) {
            // Feature not yet implemented in QEPro seabreeze driver
        }
        else if (function == P_decouple) {
            // Feature not implemented in QEPro?
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
    }
    else
        status = asynDisconnected;

    // Handle parameters that are not read from the device
    if (function == P_boxcarWidth) {
        // TODO: Investigate whether device supports boxcar averaging
        getIntegerParam(P_boxcarWidth, &rval);
        *value = rval;
        status = asynSuccess;
    }
    else if (function == P_connected) {
        *value = connected;
        status = asynSuccess;
    }
    else if (function == P_averages) {
        getIntegerParam(P_averages, &rval);
        *value = rval;
        status = asynSuccess;
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

    test_connection();
    if (connected) {
        if (function == P_maxIntensity) {
            rval = api->spectrometerGetMaximumIntensity(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            *value = rval;
            setDoubleParam(addr, P_maxIntensity, *value);
        }
        else if (function == P_nonLinearity) {
            double buffer;
            api->nonlinearityCoeffsGet(
                    device_id,
                    nonlinearity_feature_id,
                    &error,
                    &buffer,
                    1);
            *value = buffer;
            setDoubleParam(addr, P_nonLinearity, *value);
        }
        else if (function == P_cpuTemperature) {
            read_temperatures();
            *value = temperatures[CPU_TEMPERATURE];
        }
        else if (function == P_pcbTemperature) {
            *value = temperatures[PCB_TEMPERATURE];
        }
        else if (function == P_detTemperature) {
            *value = temperatures[TEC_TEMPERATURE];
        }
        else {
            status = asynPortDriver::readFloat64(pasynUser, value); 
        }
    }
    else
        status = asynDisconnected;

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
    //int function = pasynUser->reason;
    int addr;
    //int error;

    this->getAddress(pasynUser, &addr);

    asynPrint(pasynUser, 
            ASYN_TRACE_FLOW, 
            "readFloat64Array: entering\n");

    test_connection();
    if (connected) {
        /*
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

         asynPrint(pasynUser, 
         ASYN_TRACE_FLOW, 
         "readFloat64Array: reading wavelengths\n");
         asynPrint(pasynUser, 
         ASYN_TRACE_FLOW, 
         "readFloat64Array: num_wavelengths = %d\n",
         num_wavelengths);
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
         value[i] = (1./m_laser - 1./wavelengths[i]) *10e7; // Raman shift in cm-1

         *nIn = num_wavelengths;
         }
         */
    }
    else {
        status = asynDisconnected;
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
    //int boxcar, averages, electricDark, nonLinearity, triggerMode;
    int triggerMode;
    int decouple, ledIndicator;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    this->getAddress(pasynUser, &addr);
    //    getIntegerParam(addr, nrBoard, &index);

    test_connection();
    if (connected) {
        if (function == P_integrationTime) {
            int tmp;
            getIntegerParam(P_integrationTime, &tmp);
            // We are getting the integration time in microseconds
            integration_time = tmp;
            api->spectrometerSetIntegrationTimeMicros(
                    device_id,
                    spectrometer_feature_id,
                    &error,
                    integration_time);
            // Invalidate the current dark spectra
            dark_valid = false;
            setIntegerParam(P_darkValid, dark_valid);
            callParamCallbacks();
        }
        else if (function == P_averages) {
            // Check if implmemented in QEPro
        }
        else if (function == P_boxcarWidth) {
            // Not implmemented in QEPro. Just store the value
            // in the parameter list.
        }
        else if (function == P_electricDark) {
            // Check if implmemented in QEPro
        }
        else if (function == P_nonLinearity) {
            // Check if implmemented in QEPro
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
        else if (function == decouple) {
            // Check if implmemented in QEPro
        }
        else if (function == ledIndicator) {
            // Check if implmemented in QEPro
        }

        else {
            /* All other parameters just get set in parameter list, no need to
             *          * act on them here */
        }
    }

    // Do functions that are not dependent on connection
    if (function == P_darkSubtract) {
        if (value == 0)
            dark_subtract = false;
        else
            dark_subtract = true;
    }
    else if (function == P_darkAcq) {
        if (value == 1)
            dark_acquire = true;
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

asynStatus drvUSBQEPro::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char* functionName = "writeFloat64";
    int addr;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setDoubleParam(function, value);

    this->getAddress(pasynUser, &addr);

    if (function == P_roi0LowWavelength) {
        roi_low[0] = value;
    }
    else if (function == P_roi0HighWavelength) {
        roi_high[0] = value;
    }
    else if (function == P_roi1LowWavelength) {
        roi_low[1] = value;
    }
    else if (function == P_roi1HighWavelength) {
        roi_high[1] = value;
    }

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s: port=%s, value=%f, addr=%d, status=%d\n",
                driverName, functionName, this->portName, value, addr, (int)status);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%f, addr=%d\n",
                driverName, functionName, this->portName, value, addr);
    return status;

}


// Test whether spectrometer is connected. If it is was previously
// disconnected, read the various id values and store to avoid needing to 
// read each time.
void drvUSBQEPro::test_connection() {
    int error;
    bool found_ooi_spectrometer = false;

    // Check the USB device list to see if the Ocean Optics spectrometer
    //context = NULL;
    libusb_device **list;
    int rc = 0;
    ssize_t count = 0;

    count = libusb_get_device_list(context, &list);
    assert(count > 0);

    for (ssize_t idx = 0; idx < count; ++idx) {
        libusb_device *device = list[idx];
        libusb_device_descriptor desc = {0};

        rc = libusb_get_device_descriptor(device, &desc);
        assert(rc == 0);

        if (desc.idVendor == drvUSBQEPro::OOI_VENDOR_ID) {
            found_ooi_spectrometer = true;
        }
    }
    libusb_free_device_list(list, 1);

    asynPrint(pasynUserSelf, 
            ASYN_TRACE_FLOW, 
            "test_connection: found_ooi_spectrometer = %d\n",
            found_ooi_spectrometer);

    // Restart the connection to the spectrometer
    if (found_ooi_spectrometer && !connected) {
        if (api) {
            api->shutdown();
        }
        api = SeaBreezeAPI::getInstance();
        api->probeDevices();
    }

    if (!found_ooi_spectrometer) {
        connected = false;
        deallocate_spectrum_buffer();
    }
    else {
        // Check if we were disconnected previously 
        if (!connected) {
            // Read device IDs
            int number_of_devices = api->getNumberOfDeviceIDs();
            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: number of Ocean Optics devices = %d\n",
                    number_of_devices);

            long * device_ids = (long *)calloc(number_of_devices, sizeof(long));
            api->getDeviceIDs(device_ids, number_of_devices);
            // Assume only one device
            device_id = device_ids[0];

            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: device ID = %ld\n",
                    device_id);

            api->openDevice(device_id, &error);

            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: opened device ID = %ld. Code = %d [%s]\n",
                    device_id,
                    error,
                    sbapi_get_error_string(error));

            // Read spectrometer feature ID
            int num_spectrometer_features = 
                api->getNumberOfSpectrometerFeatures(device_id, &error);

            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: number of spectrometer features = %d\n",
                    num_spectrometer_features);

            if (num_spectrometer_features > 0) {
                long * spectrometer_feature_ids = 
                    (long *)calloc(num_spectrometer_features, sizeof(long));

                api->getSpectrometerFeatures(
                        device_ids[0],
                        &error,
                        spectrometer_feature_ids, 
                        num_spectrometer_features);

                spectrometer_feature_id = spectrometer_feature_ids[0];
            }
            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: spectrometer feature id = 0x%lx\n", 
                    spectrometer_feature_id);

            // Read USB feature ID
            int num_usb_features = 
                api->getNumberOfRawUSBBusAccessFeatures(device_id, &error);

            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: number of usb features = %d\n",
                    num_usb_features);

            if (num_usb_features > 0) {
                long * usb_feature_ids = 
                    (long *)calloc(num_usb_features, sizeof(long));

                api->getRawUSBBusAccessFeatures(
                        device_ids[0],
                        &error,
                        usb_feature_ids, 
                        num_usb_features);

                usb_feature_id = usb_feature_ids[0];
            }
            asynPrint(pasynUserSelf, 
                    ASYN_TRACE_FLOW, 
                    "test_connection: usb feature id = 0x%lx\n", 
                    usb_feature_id);

            // Read serial number feature ID
            int num_serial_number_features = 
                api->getNumberOfSerialNumberFeatures(
                        device_id, 
                        &error);

            if (num_serial_number_features > 0) {
                long * serial_number_feature_ids = 
                    (long *)calloc(
                            num_serial_number_features, 
                            sizeof(long));

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
                api->getNumberOfNonlinearityCoeffsFeatures(
                        device_id, 
                        &error);

            if (num_nonlinearity_features > 0) {
                long * nonlinearity_feature_ids = 
                    (long *)calloc(
                            num_nonlinearity_features, 
                            sizeof(long));

                api->getNonlinearityCoeffsFeatures(
                        device_id,
                        &error,
                        nonlinearity_feature_ids, 
                        num_nonlinearity_features);

                nonlinearity_feature_id = 
                    nonlinearity_feature_ids[0];
            }
            connected = true;
            allocate_spectrum_buffer();
        }
    }

    // Update the connected PV status
    setIntegerParam(P_connected, connected);
    callParamCallbacks();
}

void drvUSBQEPro::boxcar(
        const double *spectrum_buffer,
        double *process_buffer,
        int boxcar_half_width) {

    int boxcar_width = 2 * boxcar_half_width + 1;
    double sum;
    double average;

    // TODO: Fix algorithm to keep running total. Only need one addition and one 
    // subtraction per calculation.
    for (int i = 0; i < num_pixels; i++) {
        sum = 0;
        if (i < boxcar_half_width) {
            for (int j = 0; j < i + boxcar_half_width + 1; j++) 
                sum += spectrum_buffer[j];
            average = sum / (double) (boxcar_half_width + i + 1);
        }
        else if (i >= num_pixels - boxcar_half_width) {
            for (int j = i - boxcar_half_width;
                    j < num_pixels;
                    j++)
                sum += spectrum_buffer[j];
            average = sum / (double) (num_pixels - i + boxcar_half_width);
        }
        else {
            for (int j = i - boxcar_half_width;
                    j < i + boxcar_half_width + 1;
                    j++)
                sum += spectrum_buffer[j];
            average = sum / (double) boxcar_width;
        }
        process_buffer[i] = average;
    }
}

void drvUSBQEPro::write_file(
        double *x_axis_buffer,
        double *data_buffer) {

    const char *functionName = "write_file";
    //const int BUF_SIZE = 80;
    char file_path[BUF_SIZE + 1];
    char file_name[BUF_SIZE + 1];
    char full_file_name[BUF_SIZE + 1];
    char full_file_path[2 * BUF_SIZE + 1];

    int file_index;

    std::ofstream outfile;

    getStringParam(P_fileName, BUF_SIZE, file_name);
    getStringParam(P_filePath, BUF_SIZE, file_path);
    getIntegerParam(P_fileIndex, &file_index);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: file name = %s\n",
            functionName,
            file_name);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: file path = %s\n",
            functionName,
            file_path);

    snprintf(full_file_name,
            BUF_SIZE,
            "%s_%05d.txt",
            file_name,
            file_index);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: file path = %s\n",
            functionName,
            file_path);

    strcpy(full_file_path, file_path);
    strcat(full_file_path, "/");
    strcat(full_file_path, full_file_name);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: full file path = %s\n",
            functionName,
            full_file_path);

    outfile.open(full_file_path, std::ofstream::out);

    write_header(outfile, full_file_name);

    outfile.precision(4);

    for (int i = 0; i < num_pixels; i++) {
        outfile << std::left << std::setw(12) << std::fixed;
        outfile << x_axis_buffer[i];
        outfile << std::left << std::setw(12) << std::fixed;
        outfile << data_buffer[i];
        outfile << std::endl;
    }

    outfile.close();

    // Increment the index and save back to the record
    file_index++;
    setStringParam(P_fullFileName, full_file_name);
    setStringParam(P_filePath, file_path);
    setStringParam(P_fullFilePath, full_file_path);
    setIntegerParam(P_fileIndex, file_index);
    callParamCallbacks();
}

void drvUSBQEPro::write_header(std::ofstream &outfile, char *full_file_name) {
    int integration_time;
    char serial_number[BUF_SIZE + 1];
    char text_buffer[BUF_SIZE + 1];
    int trigger_mode;
    int electric_dark_correction;
    int nonlinearity_correction;
    int boxcar_width;
    int x_axis_mode;
    int scans_to_average = 1;

    outfile << "QEPro datafile: " << full_file_name << std::endl;

    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(text_buffer,
            BUF_SIZE,
            "%FT%T%Z",
            timeinfo);

    outfile << "Date: " << text_buffer << std::endl;
    outfile << "User: " << std::endl;
    getStringParam(P_serialNumber, BUF_SIZE, serial_number);
    outfile << "Spectrometer: " << serial_number << std::endl;

    getIntegerParam(P_triggerMode, &trigger_mode);
    outfile << "Trigger mode: " << trigger_mode << std::endl;

    outfile.precision(5);
    getIntegerParam(P_integrationTime, &integration_time);
    outfile << "Integration time (s): " << std::scientific;
    outfile <<  integration_time/1000000 << std::endl;

    getIntegerParam(P_averages, &scans_to_average);
    outfile << "Scans to average: " << scans_to_average << std::endl;

    getIntegerParam(P_electricDark, &electric_dark_correction);
    outfile << "Electric dark correction enabled: ";
    outfile << ((electric_dark_correction==0)?"false":"true");
    outfile << std::endl;

    getIntegerParam(P_nonLinearity, &nonlinearity_correction);
    outfile << "Nonlinearity correction enabled: ";
    outfile << ((nonlinearity_correction==0)?"false":"true");
    outfile << std::endl;

    getIntegerParam(P_boxcarWidth, &boxcar_width);
    outfile << "Boxcar width: " << boxcar_width << std::endl;

    getIntegerParam(P_xAxisMode, &x_axis_mode);
    outfile << "XAxis mode: ";
    outfile << ((x_axis_mode==0)?"Wavelength (nm)":"Raman Shifts");
    outfile << std::endl;

    outfile << "Number of pixels in spectrum: " << num_pixels << std::endl;

    outfile << ">>>>> Begin Spectral Data <<<<<" << std::endl;
}

extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvUSBQEPro class.
     *   * \param[in] portName The name of the asyn port driver to be created.
     *     * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
    int drvUSBQEProConfigure(const char *portName, int maxPoints, double laser) {
        new drvUSBQEPro(portName, maxPoints, laser);
        return(asynSuccess);
    }


    /* EPICS iocsh shell commands */

    static const iocshArg initArg0 = { "portName", iocshArgString};
    static const iocshArg initArg1 = { "max points",iocshArgInt};
    static const iocshArg initArg2 = { "laser",iocshArgDouble};
    static const iocshArg * const initArgs[] = { 
        &initArg0,
        &initArg1,
        &initArg2
    };

    static const iocshFuncDef initFuncDef = {"drvUSBQEProConfigure",3,initArgs};

    static void initCallFunc(const iocshArgBuf *args) { 
        drvUSBQEProConfigure(args[0].sval, args[1].ival, args[2].dval);
    }

    void drvUSBQEProRegister(void) {
        iocshRegister(&initFuncDef, initCallFunc);
    }

    epicsExportRegistrar(drvUSBQEProRegister);
}

static void worker(void *pPvt) {
    drvUSBQEPro *ptr = (drvUSBQEPro *)pPvt;
    ptr->getSpectrumThread(ptr);
}

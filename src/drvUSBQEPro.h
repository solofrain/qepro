#ifndef DRVUSBQEPRO_H
#define DRVUSBQEPRO_H
#include <iocsh.h>
#include <epicsExport.h>
#include <asynPortDriver.h>

#include <libusb-1.0/libusb.h>

#include <fstream>

#include "drvUSBQEProOBP.h"
#include "api/seabreezeapi/SeaBreezeAPI.h"

#define QEProNumSpecs           "NumSpecs"              /* asynInt32        ro  0 */
#define QEProId                 "NrBoard"               /* asynInt32        ro  1 */
#define QEProName               "Name"                  /* asynOctet        ro  2 */ 
#define QEProFirmwareVersion    "FirmwareVersion"       /* asynOctet        ro  3 */
#define QEProFirmwareModel      "FirmwareModel"         /* asynOctet        ro  4 */    
#define QEProSerialNumber       "SerialNumber"          /* asynOctet        ro  5 */
#define QEProNumberOfPixels     "NumberOfPixels"        /* asynInt32        rw  6 */
#define QEProNumberOfDarkPixels "NumberOfDarkPixels"    /* asynInt32        rw  7 */
#define QEProIntegrationTime    "IntegrationTime"       /* asynInt32        rw  8 */
#define QEProMaxIntegrationTime "MaxIntegrationTime"    /* asynInt32        rw  9 */
#define QEProMinIntegrationTime "MinIntegrationTime"    /* asynInt32        rw 10 */
#define QEProMaxIntensity       "MaxIntensity"          /* asynInt32        rw 11 */
#define QEProBoxcarWidth        "BoxcarWidth"           /* asynInt32        rw 12 */
#define QEProElectricDark       "ElectricDark"          /* asynInt32        rw 13 */
#define QEProDetectorTemperature "DetectorTemperature"  /* asynInt32        rw 14 */
#define QEProBoardTemperature   "BoardTemperature"      /* asynInt32        rw 15 */
#define QEProTempSetPoint       "TempSetPoint"          /* asynInt32        rw 16 */
#define QEProTriggerMode        "TriggerMode"           /* asynInt32        rw 17 */
#define QEProNonLinearity       "NonLinearity"          /* asynInt32        rw 18 */
#define QEProDecouple           "Decouple"              /* asynInt32        rw 19 */
#define QEProLEDIndicator       "LEDIndicator"          /* asynInt32        rw 20 */
#define QEProAverages           "Averages"              /* asynInt32        rw 21 */
#define QEProXAxisNm            "XAxisNm"               /* asynFloat64Array ro 22 */
#define QEProXAxisRs            "XAxisRs"               /* asynFloat64Array ro 23 */
#define QEProSpectrum           "Spectrum"              /* asynFloat64Array ro 24 */
#define QEProLaser              "Laser"                 /* asynFloat64      ro 25 */
#define QEProConnected          "Connected"             /* asynInt32        ro 26 */
#define QEProAcqMode            "AcqMode"               /* asynInt32        rw 27 */
#define QEProAcqStart           "AcqStart"              /* asynInt32        rw 28 */
#define QEProAcqStop            "AcqStop"               /* asynInt32        rw 29 */
#define QEProAcqSts             "AcqSts"                /* asynInt32        ro 30 */
#define QEProFileWrite          "FileWrite"             /* asynInt32        rw 31 */
#define QEProFilePath           "FilePath"              /* asynOctet        rw 32 */
#define QEProFileName           "FileName"              /* asynOctet        rw 33 */
#define QEProFullFileName       "FullFileName"          /* asynOctet        rw 34 */
#define QEProFullFilePath       "FullFilePath"          /* asynOctet        rw 35 */
#define QEProFileIndex          "FileIndex"             /* asynInt32        rw 36 */
#define QEProXAxisMode          "XAxisMode"             /* asynInt32        rw 37 */
#define QEProXAxis              "XAxis"                 /* asynFloat64Array ro 38 */
#define QEProCPUTemperature     "CPUTemperature"        /* asynFloat64      ro 39 */
#define QEProPCBTemperature     "PCBTemperature"        /* asynFloat64      ro 40 */
#define QEProDetTemperature     "DetTemperature"        /* asynFloat64      ro 41 */
#define QEProROI1LowWavelength  "ROI1LowWavelength"     /* asynFloat64      ro 42 */
#define QEProROI1HighWavelength "ROI1HighWavelength"    /* asynFloat64      ro 43 */
#define QEProROI2LowWavelength  "ROI2LowWavelength"     /* asynFloat64      ro 44 */
#define QEProROI2HighWavelength "ROI2HighWavelength"    /* asynFloat64      ro 45 */
#define QEProDarkAcq            "DarkAcq"               /* asynInt32        rw 46 */
#define QEProDarkSubtract       "DarkSubtract"          /* asynInt32        rw 47 */
#define QEProDarkSpectrum       "DarkSpectrum"          /* asynFloat64Array ro 48 */
#define QEProDarkValid          "DarkValid"             /* asynInt32        ro 49 */

#define POLL_TIME 0.5

#define QEPRO_ACQ_MODE_OFF          0
#define QEPRO_ACQ_MODE_SINGLE       1
#define QEPRO_ACQ_MODE_CONTINUOUS   2

#define QEPRO_XAXIS_WAVELENGTH      0
#define QEPRO_XAXIS_RAMAN_SHIFT     1

#define BUF_SIZE                    80

#define STS_REQUEST_ENDPOINT        0x01
#define STS_RESPONSE_ENDPOINT       0x81

#define NUM_TEMP_SENSORS            8
#define CPU_TEMPERATURE             0
#define PCB_TEMPERATURE             2
#define TEC_TEMPERATURE             3

class drvUSBQEPro : public asynPortDriver {

public:
    drvUSBQEPro(const char *portName, int maxArraySize, double laser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    virtual asynStatus readFloat64Array (asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn );     

    /* Thread function to read spectra from device */
    virtual void getSpectrumThread(void *);

protected:
    int         P_numSpecs;
    #define     FIRST_QEPRO_PARAM P_numSpecs
    int         P_nrBoard;
    int         P_name;
    int         P_firmwareVersion;
    int         P_firmwareModel;
    int         P_serialNumber;
    int         P_numberOfPixels;
    int         P_numberOfDarkPixels;
    int         P_integrationTime;
    int         P_maxIntegrationTime;
    int         P_minIntegrationTime;
    int         P_maxIntensity;
    int         P_boxcarWidth;
    int         P_electricDark;
    int         P_detectorTemperature;
    int         P_boardTemperature;
    int         P_tempSetPoint;
    int         P_triggerMode;
    int         P_nonLinearity;
    int         P_decouple;
    int         P_ledIndicator;
    int         P_averages; 
    int         P_xAxisNm;   // x-Axis in nanometers 
    int         P_xAxisRs;   // x-Axis in Raman shift
    int         P_spectrum;  // Raman spectrum (y-Axis)
    int         P_laser;     // laser wavelength needed for Raman shift calculation
    int         P_connected;     
    int         P_acqMode;     
    int         P_acqStart;     
    int         P_acqStop;     
    int         P_acqSts;     
    int         P_fileWrite;
    int         P_filePath;
    int         P_fileName;
    int         P_fullFileName;
    int         P_fullFilePath;
    int         P_fileIndex;
    int         P_xAxisMode;
    int         P_xAxis;
    int         P_cpuTemperature;
    int         P_pcbTemperature;
    int         P_detTemperature;
    int         P_roi1LowWavelength;
    int         P_roi1HighWavelength;
    int         P_roi2LowWavelength;
    int         P_roi2HighWavelength;
    int         P_darkAcq;
    int         P_darkSubtract;
    int         P_darkSpectrum;
    int         P_darkValid;
    #define LAST_QEPRO_PARAM P_darkValid

private:
    //Wrapper wrapper;
    SeaBreezeAPI *api;
    //epicsEventId eventId;
    static int zeroIndex;
    //int index;

    libusb_hotplug_callback_handle hp[2];
    libusb_context *context;

    static const int OOI_VENDOR_ID = 0x2457;

    long *device_ids;
    long device_id;
    long serial_number_feature_id;
    long spectrometer_feature_id;
    long usb_feature_id;
    long nonlinearity_feature_id;
    int spec_index;

    double temperatures[NUM_TEMP_SENSORS];

    bool connected;
    bool acquiring;

    void test_connection();
    void allocate_spectrum_buffer();
    void deallocate_spectrum_buffer();
    void boxcar(
            const double *spectrum_buffer,
            double *process_buffer,
            int boxcar_width);
    void write_file(
            double *x_axis_buffer,
            double *data_buffer);
    void write_header(
            std::ofstream &outfile,
            char *full_file_path);
    void convert_nm_to_raman_shift(
            double *raman_shift_buffer,
            const double *wavelength_buffer,
            int num_wavelengths);

    // QEPro functions using OBP
    int abort();
    int clear_buffers();
    int start_acquisition();
    void read_temperatures();

    // OBP support functions
    int sendOBPMessage(OBPExchange *xfer);
    const char* getOBPError(unsigned err_no);
    void write_buffer(unsigned char *request, size_t len);
    int read_buffer(unsigned char *response, size_t len);

    unsigned long integration_time;
    int trigger_mode;
    int num_pixels;
    int num_wavelengths;
    bool dark_valid;
    bool dark_subtract;
    bool dark_acquire;

    double *spectrum_buffer;
    double *wavelength_buffer;
    double *raman_shift_buffer;
    double *dark_buffer;

    double m_laser;
    double m_poll_time;

    asynStatus connectSpec();
    asynStatus disconnectSpec();
    asynStatus readStatus();

};


//static void worker(void *pPvt);
//static void worker(void *pPvt) {
    //drvUSBQEPro *ptr = (drvUSBQEPro *)pPvt;
    //ptr->getSpectrumThread(ptr);
//}

#endif

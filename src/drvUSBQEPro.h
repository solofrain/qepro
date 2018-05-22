#include <iocsh.h>
#include <epicsExport.h>
#include <asynPortDriver.h>
#include <libusb-1.0/libusb.h>

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
#define QEProLEDIndicator       "LEDIndicator"          /* asynInt32    rw 20 */
#define QEProAverages           "Averages"              /* asynInt32        rw 21 */
#define QEProXAxisNm            "XAxisNm"               /* asynFloat64Array ro 22 */
#define QEProXAxisRs            "XAxisRs"               /* asynFloat64Array ro 23 */
#define QEProSpectrum           "Spectrum"              /* asynFloat64Array ro 24 */
#define QEProLaser              "Laser"                 /* asynFloat64      ro 25 */
#define QEProConnected          "Connected"             /* asynInt32        ro 26 */
#define QEProAcqMode            "AcqMode"               /* asynInt32        ro 27 */
#define QEProAcqCtl             "AcqCtl"                /* asynInt32        ro 28 */

#define POLL_TIME 0.01

#define QEPRO_ACQ_MODE_OFF          0
#define QEPRO_ACQ_MODE_SINGLE       1
#define QEPRO_ACQ_MODE_CONTINUOUS   2

class drvUSBQEPro : public asynPortDriver {

public:
    drvUSBQEPro(const char *portName, int maxArraySize, double laser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    virtual asynStatus readFloat64Array (asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn );     

    virtual void getSpectrumThread(void *);
    /* Thread function to read spectra from device */
    void getSpectrumTask(void);

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
    int         P_acqCtl;     
    #define LAST_QEPRO_PARAM P_acqCtl

private:
    //Wrapper wrapper;
    SeaBreezeAPI *api;
    epicsEventId eventId;
    static int zeroIndex;
    int index;

    libusb_hotplug_callback_handle hp[2];
    libusb_context *context;

    static const int OOI_VENDOR_ID = 0x2457;

    long *device_ids;
    long device_id;
    long serial_number_feature_id;
    long spectrometer_feature_id;
    long nonlinearity_feature_id;

    bool connected;
    bool run;

    void test_connection();
    void allocate_spectrum_buffer();
    void deallocate_spectrum_buffer();
    void boxcar(
            const double *spectrum_buffer,
            double *process_buffer,
            int boxcar_width,
            int num_pixels);

    unsigned long integration_time;
    int trigger_mode;
    int num_pixels;

    double *spectrum_buffer;

    double m_laser;
    double m_poll_time;

    asynStatus connectSpec();
    asynStatus disconnectSpec();
    asynStatus readStatus();

    static int LIBUSB_CALL hotplug_callback(
            libusb_context *ctx, 
            libusb_device *dev, 
            libusb_hotplug_event event, 
                void *user_data);
    static int LIBUSB_CALL hotplug_callback_detach(
            libusb_context *ctx, 
            libusb_device *dev, 
            libusb_hotplug_event event, 
            void *user_data);
    asynStatus registerUSBCallbacks();



};

static void worker(void *pPvt) {
    drvUSBQEPro *ptr = (drvUSBQEPro *)pPvt;
    ptr->getSpectrumThread(ptr);
}

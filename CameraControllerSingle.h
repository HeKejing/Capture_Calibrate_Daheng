#pragma once

#include <queue>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>

#include "CameraDevice.h"


class CameraControllerSingle
{
public:
    typedef enum SE_STATUS_LIST
    {
        SE_STATUS_SUCCESS                   = 0,            ///< Operation was successful
        SE_STATUS_ERROR                     = -1,           ///< runtime error.
        SE_STATUS_TIMEOUT                   = -2,           ///< An operation's timeout time expired before it could be completed.     
        SE_STATUS_FAILED_LACK_CONFIG_FILES  = -3,           ///< lack config files
        SE_STATUS_FAILED_CAMERA             = -4,           ///< failed because of camera
        SE_STATUS_FAILED_PROJECTOR          = -5,           ///< failed because of projector
        SE_STATUS_FAILED_NO_DEVICE          = -6,           ///< failed because of no this device
    } SE_STATUS_LIST;

    CameraControllerSingle();
    ~CameraControllerSingle();
    typedef std::shared_ptr<CameraControllerSingle> Ptr;
    
    using callbackProcess = std::function<void(void)> ;

    /**
     * @brief       initialization
     * @return      result of the initialization. Please refer to enumeration SE_STATUS_LIST.
     * @attention   please call this function before other camera operations.
     */
    int init();

    /**
     * @brief       set the information of cameras controlled by CameraController
     * @param       camSerialNumber : serial number of controlled camera    
     * @attention   connectCamera() must be called before operating the CameraController
     */
    void connectCamera(std::string camSerialNumber);
    void disconnectCameras();

    /**
     * @param       setTriggerSource : 0 for SOURCE_SOFTWARE, 3 for SOURCE_LINE2. SOURCE_LINE2 is default source.
     * @attention   setTriggerSource() must be called before start()
     */
    bool setTriggerSource(int triggerSource);

    /**
     * @brief       start snap
     * @return      result. Please refer to enumeration SE_STATUS_LIST.
     * @param       isTriggerMode. true : trigger mode;  false : non-trigger mode(continuous mode)
     */
    int start(bool isTriggerMode=true);
    void stop();
    bool isOpen();

    /**
     * @attention   This function is valid in software trigger mode
     */
    bool softTrigger();

    // following functions are used in trigger mode
    /**
     * @brief       register the callback
     * @param       recvImagesCB : callback function in the form of "void(void)"
     */
    void registerCallBackProcess(callbackProcess recvImagesCB);
    void receiveCaptureImages();
    void processCaptureImages(std::vector<cv::Mat> &images);
    void setNeedImagesCount(int imageCount);
    /**
     * @brief       set the flag that whether the has finished capture
     * @param       isFinished. 
     * @attention   set false before capture a group of new images.
     */
    void setCapture3DModelFinishFlag(bool isFinished);
    void captureOneImage(cv::Mat& image);
    void captureUnrectifiedImages(cv::Mat &image);
    /**
     * @brief       clear the images that stored before
     * @attention   called before capture a group of new images.
     */
    void clearImages();
    

    // following functions are used in non-trigger mode(continuous mode)
    /**
    * @brief       get new images in non-trigger mode.
    * @param       image: image of captured image.
    * @return      false: no new images. true: has new images
    */
    bool getCapturedImage(cv::Mat &image);

    // common used functions
    void setExposureTime2D(int exposureTime2D, bool saveFlag = true);
    int getExposureTime2D();
    void setExposureTime3D(int exposureTime3D, bool saveFlag = true);
    int getExposureTime3D();
    void setPatternsNum(int patternsNum);
    bool getConnectedCameraSerialNumberString(std::string &connectedCamSN);
    int getSensorWidth();
    int getSensorHeight();

    void enumDevices();
    
    int needCaptureImagesCount;                     // the number of 3D models that will capture. default is 1. set this variable when continuous acquisition
    bool continuousFlag;                            // the flag of continuous acquisition
    std::vector<std::string>    allCamSNStrings;    // all camera serial number strings. not only the two using cameras.

private:
    int openDeviceForGetDeviceInfo();
    
    int                         exposureTime2D;
    int                         exposureTime3D;
    int                         patternsNum;

    CameraDevice*               m_pDeviceProcess;       // device object pointer
    std::string                 camSerialNumber;        // device serial number

    callbackProcess             recvImagesCB;
    int                         capFinishCamNum;        // number of camares that have finished capture
};


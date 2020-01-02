
#include "CameraControllerSingle.h"
#include <chrono>
#include <thread>

CameraControllerSingle::CameraControllerSingle()
{
    recvImagesCB = NULL;

    capFinishCamNum         = 0;
    needCaptureImagesCount  = 1;
    continuousFlag          = false;
    exposureTime2D          = 8000;         // initial default value
    exposureTime3D          = 500;          // initial default value
    
    m_pDeviceProcess = new CameraDevice;
    m_pDeviceProcess->registerCallBackCameraController(std::bind(&CameraControllerSingle::receiveCaptureImages, this));
    patternsNum = 1;
    setPatternsNum(patternsNum);

    // init the GxIAPI and enum all cameras
    GX_STATUS status = GX_STATUS_ERROR;
    status = GXInitLib();
    GX_VERIFY(status);
    
    // enum the devices
    enumDevices();
}

CameraControllerSingle::~CameraControllerSingle()
{
    if (NULL != m_pDeviceProcess) {
        delete m_pDeviceProcess;
        m_pDeviceProcess = NULL;
    }

    // uninit the GxIAPI
    GXCloseLib();
}

int CameraControllerSingle::init()
{
    // read the values in registry    
    //exposureTime2D = RegistryManager::getInstance().value(CONFIG_TDIMAGE, KEY_TDIMAGE_EXPOSURETIME, exposureTime2D).toInt();
    //PRINTF_INFO("exposureTime2D changed to : %d", exposureTime2D);
    setExposureTime2D(exposureTime2D);
    
    //exposureTime3D = RegistryManager::getInstance().value(CONFIG_CAPTURE, KEY_CAPTURE_EXPOSURETIME3DSPINBOX, exposureTime3D).toInt();
    //PRINTF_INFO("exposureTime3D changed to : %d", exposureTime3D);
    setExposureTime3D(exposureTime3D);

    // connect default cameras
    if (allCamSNStrings.size() <= 0) {
        PRINTF_ERROR("there are no cameras. Please check the hardware.");
        return SE_STATUS_FAILED_NO_DEVICE;
    }
    camSerialNumber = allCamSNStrings[0];
    connectCamera(camSerialNumber);
    
    return SE_STATUS_SUCCESS;    
}

void CameraControllerSingle::connectCamera(std::string camSerialNumber)
{
    disconnectCameras();

    this->camSerialNumber = camSerialNumber;
    PRINTF_SMARTEYE("camSerialNumbers : \t%s", camSerialNumber.c_str());

    // set pParameter->resolutionWidth, pParameter->resolutionHeight and get m_strDeviceSN
    openDeviceForGetDeviceInfo();
}

void CameraControllerSingle::disconnectCameras()
{
    camSerialNumber.clear();
    
    // close cameras
    if (isOpen()) {
        stop();
    }
}

bool CameraControllerSingle::setTriggerSource(int triggerSource)
{
    return m_pDeviceProcess->setTriggerSource(triggerSource);
}

int CameraControllerSingle::start(bool isTriggerMode)
{
    // open the device if not open before
    if (false == m_pDeviceProcess->IsOpen()) {
        m_pDeviceProcess->OpenDevice(camSerialNumber, 0);
    }

    if (isTriggerMode) {
        std::queue<cv::Mat> clearImage;
        m_pDeviceProcess->myCapturedImage.swap(clearImage);  // empty the myCapturedImage          
        m_pDeviceProcess->captureImagesCount = 0;
        m_pDeviceProcess->setCaptureMode(true);              // set trigger mode
    }
    else {
        m_pDeviceProcess->setCaptureMode(false);              // set non-trigger mode
    }

    // start snap
    m_pDeviceProcess->StartSnap();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));     // verify that if we can remove this delay 

    return SE_STATUS_SUCCESS;
}

void CameraControllerSingle::stop()
{
    // stop snap
    if (m_pDeviceProcess->IsSnap())
        m_pDeviceProcess->StopSnap();

    // close devices
    if (m_pDeviceProcess->IsOpen())
        m_pDeviceProcess->CloseDevice();
}

bool CameraControllerSingle::isOpen()
{
    return m_pDeviceProcess->IsOpen();
}

bool CameraControllerSingle::softTrigger()
{
    return m_pDeviceProcess->softTrigger();
}

void CameraControllerSingle::registerCallBackProcess(callbackProcess recvImagesCB)
{
    this->recvImagesCB = recvImagesCB;
}

void CameraControllerSingle::receiveCaptureImages()
{ 
    // callback to ProcessController
    if (!this->recvImagesCB) {
        PRINTF_ERROR("recvImagesCB is null.");
        return;
    }
    this->recvImagesCB();
}

void CameraControllerSingle::processCaptureImages(std::vector<cv::Mat> &images)
{
    images.reserve(patternsNum); 
    images.clear(); 

    // push_back the images need for reconstruction
    for (int i = 0; i < patternsNum; i++) {
        cv::Mat currentProcessCvImage = m_pDeviceProcess->myCapturedImage.front();
        m_pDeviceProcess->myCapturedImage.pop();

        images.push_back(currentProcessCvImage);
    }

    PRINTF_SMARTEYE("leave."); 
}

/*****************************************************************************
 * Function      :  CameraController.setNeedImagesCount
 * Description   :  set capturing image count
 * Input         :  int imageCoount  
 * Output        :  None
 * Return        :  void
*****************************************************************************/
void CameraControllerSingle::setNeedImagesCount(int imageCount)
{
    m_pDeviceProcess->needImagesCount = imageCount;
}

/*****************************************************************************
 * Function      :  CameraController.setCapture3DModelFinishFlag
 * Description   :  set capture3DModelFinishFlag
 * Input         :  bool isFinished       
 * Output        :  None
 * Return        :  void
*****************************************************************************/
void CameraControllerSingle::setCapture3DModelFinishFlag(bool isFinished)
{
    m_pDeviceProcess->capture3DModelFinishFlag = isFinished;
}

void CameraControllerSingle::captureOneImage(cv::Mat& image)
{
    m_pDeviceProcess->isCapturingFlag = true;
    m_pDeviceProcess->setCaptureMode(false);

    while (true) {
        if ((NULL != m_pDeviceProcess->myCurrentCapturingImage.data) && (false == m_pDeviceProcess->isCapturingFlag)) {
            image = m_pDeviceProcess->myCurrentCapturingImage;
            break;
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    m_pDeviceProcess->setCaptureMode(true);
}

void CameraControllerSingle::captureUnrectifiedImages(cv::Mat &image)
{
    m_pDeviceProcess->isCapturingFlag = true;
    m_pDeviceProcess->setCaptureMode(false);

    while (true) {
        if ((NULL != m_pDeviceProcess->myCurrentCapturingImage.data) && (false == m_pDeviceProcess->isCapturingFlag)) {
            image = m_pDeviceProcess->myCurrentCapturingImage;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    m_pDeviceProcess->setCaptureMode(true);
}

void CameraControllerSingle::clearImages()
{
    if (NULL != m_pDeviceProcess) {
        std::queue<cv::Mat> empty;
        std::swap(m_pDeviceProcess->myCapturedImage, empty);
    }
}

bool CameraControllerSingle::getCapturedImage(cv::Mat &image)
{
    cv::Mat image_temp;

    m_pDeviceProcess->isCapturingFlag = true;

    auto start = std::chrono::steady_clock::now();

    while (true) {
        if ((NULL != m_pDeviceProcess->myCurrentCapturingImage.data) && (false == m_pDeviceProcess->isCapturingFlag)) {
            image_temp = m_pDeviceProcess->myCurrentCapturingImage;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // time-out detection
        auto end = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed_ms.count() > 2000) { // over 2000ms means no images
            return false;
        }
    }

    image = image_temp.clone();
    return true;
}

/*****************************************************************************
 * Function      :  CameraController.setExposureTime2D
 * Description   :  set exposure time
 * Input         :  int exposureTime2D
                    bool saveFlag     true(default) save to the registry ; false = do not save to the registry
 * Output        :  None
 * Return        :  void
*****************************************************************************/
void CameraControllerSingle::setExposureTime2D(int exposureTime2D, bool saveFlag)
{    
    this->exposureTime2D = exposureTime2D;
    //if (saveFlag) {
    //    QString exposureTime2DString = QString("%1").arg(exposureTime2D);
    //    RegistryManager::getInstance().setValue(CONFIG_TDIMAGE, KEY_TDIMAGE_EXPOSURETIME, exposureTime2DString);
    //}

    // set exposureTime2D to camera
    m_pDeviceProcess->setExposureTime2D(exposureTime2D);
}

int CameraControllerSingle::getExposureTime2D()
{
    return exposureTime2D;
}

/*****************************************************************************
 * Function      :  CameraController.setExposureTime3D
 * Description   :  set exposure time
 * Input         :  int exposureTime3D
                    bool saveFlag     true(default) save to the registry ; false = do not save to the registry
 * Output        :  None
 * Return        :  void
*****************************************************************************/
void CameraControllerSingle::setExposureTime3D(int exposureTime3D, bool saveFlag)
{    
    this->exposureTime3D = exposureTime3D;  
    //if (saveFlag) {
    //    QString exposureTime3DString = QString("%1").arg(exposureTime3D);
    //    RegistryManager::getInstance().setValue(CONFIG_CAPTURE, KEY_CAPTURE_EXPOSURETIME3DSPINBOX, exposureTime3DString);
    //}
    
    // set exposureTime3D to camera
    m_pDeviceProcess->setExposureTime3D(exposureTime3D);
}

int CameraControllerSingle::getExposureTime3D()
{
    return exposureTime3D;
}

void CameraControllerSingle::setPatternsNum(int patternsNum)
{
    this->patternsNum = patternsNum;

    // set patternsNum to camera
    m_pDeviceProcess->setPatternsNum(patternsNum);
}

bool CameraControllerSingle::getConnectedCameraSerialNumberString(std::string &connectedCamSN)
{
    if (camSerialNumber.empty()) {
        return false;
    }
    
    connectedCamSN = camSerialNumber;

    return true;
}

int CameraControllerSingle::getSensorWidth()
{
    return m_pDeviceProcess->getSensorWidth();
}

int CameraControllerSingle::getSensorHeight()
{
    return m_pDeviceProcess->getSensorHeight();
}

void CameraControllerSingle::enumDevices()
{
    GX_STATUS status = GX_STATUS_ERROR;

    unsigned int device_number = 0;
    status = GXUpdateDeviceList(&device_number, 200);
    GX_VERIFY(status);
    if (device_number <= 0) {
        status = GXUpdateDeviceList(&device_number, 1000);
        GX_VERIFY(status);
    }

    GX_DEVICE_BASE_INFO *  m_baseinfo = new GX_DEVICE_BASE_INFO[device_number];     // test if change ptr to array
    if (NULL == m_baseinfo) {
        PRINTF_ERROR("error : m_baseinfo apply memory failed.");
        return;
    }
    size_t size = device_number * sizeof(GX_DEVICE_BASE_INFO);
    status = GXGetAllDeviceBaseInfo(m_baseinfo, &size);
    if (status != GX_STATUS_SUCCESS) {
        delete[]m_baseinfo;                                                        // change to array. delete if statement
        m_baseinfo = NULL;
        GX_VERIFY(status);
    }

    // convert the camera SN to std::string
    allCamSNStrings.clear();
    for (int i = 0; i < device_number; i++) {
        allCamSNStrings.push_back((std::string)m_baseinfo[i].szSN);
    }
    delete[]m_baseinfo;
    m_baseinfo = NULL;
}

int CameraControllerSingle::openDeviceForGetDeviceInfo()
{
    // get the device information
    m_pDeviceProcess->OpenDevice(camSerialNumber, 0);
    m_pDeviceProcess->getDeviceInfo();
    
    // set pParameter->resolutionWidth and pParameter->resolutionHeight        
    //pParameter->resolutionWidth = m_pDeviceProcess[0]->getSensorWidth();
    //pParameter->resolutionHeight = m_pDeviceProcess[0]->getSensorHeight();          
    //PRINTF_INFO("pParameter->resolutionWidth : %d", pParameter->resolutionWidth); 
    //PRINTF_INFO("pParameter->resolutionHeight : %d", pParameter->resolutionHeight);

    return 0;
}



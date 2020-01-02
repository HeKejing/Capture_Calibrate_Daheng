
#include "CameraController.h"
#include <chrono>
#include <thread>

CameraController::CameraController()
{
    recvImagesCB = NULL;

    capFinishCamNum         = 0;
    needCaptureImagesCount  = 1;
    continuousFlag          = false;
    exposureTime2D          = 8000;         // initial default value
    exposureTime3D          = 500;          // initial default value
    
    for (int i = 0; i < NUM_CAMERA; i++) {
        m_pDeviceProcess[i] = new CameraDevice;
        // register callback function
        m_pDeviceProcess[i]->registerCallBackCameraController(std::bind(&CameraController::receiveCaptureImages, this));
    }
    patternsNum = 11;
    setPatternsNum(patternsNum);

    // init the GxIAPI and enum all cameras
    GX_STATUS status = GX_STATUS_ERROR;
    status = GXInitLib();
    GX_VERIFY(status);
    
    // enum the devices
    enumDevices();
}

CameraController::~CameraController()
{
    for (int i = 0; i < NUM_CAMERA; i++) {
        if (0 != m_pDeviceProcess[i]) {
            delete m_pDeviceProcess[i];
            m_pDeviceProcess[i] = 0;
        }
    }

    // uninit the GxIAPI
    GXCloseLib();
}

int CameraController::init()
{
    // read the values in registry    
    //exposureTime2D = RegistryManager::getInstance().value(CONFIG_TDIMAGE, KEY_TDIMAGE_EXPOSURETIME, exposureTime2D).toInt();
    //PRINTF_INFO("exposureTime2D changed to : %d", exposureTime2D);
    setExposureTime2D(exposureTime2D);
    
    //exposureTime3D = RegistryManager::getInstance().value(CONFIG_CAPTURE, KEY_CAPTURE_EXPOSURETIME3DSPINBOX, exposureTime3D).toInt();
    //PRINTF_INFO("exposureTime3D changed to : %d", exposureTime3D);
    setExposureTime3D(exposureTime3D);

    // connect default cameras
    if (allCamSNStrings.size() < NUM_CAMERA) {
        PRINTF_ERROR("there are no %d cameras. Please check the hardware.", (int)allCamSNStrings.size());
        return SE_STATUS_FAILED_NO_DEVICE;
    }
    for (int i = 0; i < NUM_CAMERA; ++i) {
        camSerialNumbers.push_back(allCamSNStrings[i]);
    }
    connectCameras(camSerialNumbers);
    
    return SE_STATUS_SUCCESS;    
}

void CameraController::connectCameras(std::vector<std::string> camSerialNumbers)
{
    disconnectCameras();

    this->camSerialNumbers.assign(camSerialNumbers.begin(), camSerialNumbers.end()); 
    for(int i=0; i<camSerialNumbers.size(); i++) {
        PRINTF_SMARTEYE("camSerialNumbers : \t%s", camSerialNumbers[i].c_str());
    }

    // set pParameter->resolutionWidth, pParameter->resolutionHeight and get m_strDeviceSN
    openDeviceForGetDeviceInfo();
}

void CameraController::disconnectCameras()
{
    camSerialNumbers.clear();
    
    // close cameras
    if (isOpen()) {
        stop();
    }
}

bool CameraController::setTriggerSource(int triggerSource)
{
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        if (false == m_pDeviceProcess[idx]->setTriggerSource(triggerSource)) {
            return false;
        }
    }
    return true;
}

int CameraController::start(bool isTriggerMode)
{
    for (int i = 0; i < NUM_CAMERA; i++) {
        // open the device if not open before
        if (false == m_pDeviceProcess[i]->IsOpen()) {
            m_pDeviceProcess[i]->OpenDevice(camSerialNumbers[i], i);
        }
        
        if (isTriggerMode) {
            std::queue<cv::Mat> clearImage;
            m_pDeviceProcess[i]->myCapturedImage.swap(clearImage);  // empty the myCapturedImage          
            m_pDeviceProcess[i]->captureImagesCount = 0;
            m_pDeviceProcess[i]->setCaptureMode(true);              // set trigger mode
        }
        else {
            m_pDeviceProcess[i]->setCaptureMode(false);              // set non-trigger mode
        }
        
        // start snap
        m_pDeviceProcess[i]->StartSnap();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));     // verify that if we can remove this delay
    }

    return SE_STATUS_SUCCESS;
}

void CameraController::stop()
{
    for (int i = 0; i < NUM_CAMERA; i++) {
        // stop snap
        if (m_pDeviceProcess[i]->IsSnap())
            m_pDeviceProcess[i]->StopSnap();
        
        // close devices
        if (m_pDeviceProcess[i]->IsOpen())
            m_pDeviceProcess[i]->CloseDevice();
    }
}

bool CameraController::isOpen()
{
    for (int idx=0; idx<NUM_CAMERA; idx++) {
        if(false == m_pDeviceProcess[idx]->IsOpen()) {
            return false;
        }
    }
    return true;
}

bool CameraController::softTrigger()
{
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        if (false == m_pDeviceProcess[idx]->softTrigger()) {
            return false;
        }
    }
    return true;
}

void CameraController::registerCallBackProcess(callbackProcess recvImagesCB)
{
    this->recvImagesCB = recvImagesCB;
}

void CameraController::receiveCaptureImages()
{     
    // add lock to prevent resource preemption
    std::lock_guard<std::mutex> lck(mtx_capFinishCamNum);
    if (++capFinishCamNum < NUM_CAMERA) return;
    
    capFinishCamNum = 0;
    // callback to ProcessController
    if (!this->recvImagesCB) {
        PRINTF_ERROR("recvImagesCB is null.");
        return;
    }
    this->recvImagesCB();
}

void CameraController::processCaptureImages(std::vector<cv::Mat> &leftImages, std::vector<cv::Mat> &rightImages)
{
    leftImages.reserve(patternsNum); rightImages.reserve(patternsNum);
    leftImages.clear(); rightImages.clear();
    
    // correct the left and right camera
    CameraDevice* lcam, *rcam;
    rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
    lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    if (m_pDeviceProcess[0]->getLongIntSN() < m_pDeviceProcess[1]->getLongIntSN()) {
        lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
        rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    }

    // push_back the images need for reconstruction
    for (int i = 0; i < patternsNum; i++) {
        cv::Mat currentProcessCvImageL = lcam->myCapturedImage.front();
        lcam->myCapturedImage.pop();
        cv::Mat currentProcessCvImageR = rcam->myCapturedImage.front();
        rcam->myCapturedImage.pop();

        leftImages.push_back(currentProcessCvImageL);
        rightImages.push_back(currentProcessCvImageR);
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
void CameraController::setNeedImagesCount(int imageCount)
{
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        m_pDeviceProcess[idx]->needImagesCount = imageCount;
    }
}

/*****************************************************************************
 * Function      :  CameraController.setCapture3DModelFinishFlag
 * Description   :  set capture3DModelFinishFlag
 * Input         :  bool isFinished       
 * Output        :  None
 * Return        :  void
*****************************************************************************/
void CameraController::setCapture3DModelFinishFlag(bool isFinished)
{
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        m_pDeviceProcess[idx]->capture3DModelFinishFlag = isFinished;
    }
}

void CameraController::captureOneImage(cv::Mat& image)
{
    // use the left camera 
    CameraDevice* lcam;
    lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    if (m_pDeviceProcess[0]->getLongIntSN() < m_pDeviceProcess[1]->getLongIntSN()) {
        lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
    }

    lcam->isCapturingFlag = true;
    lcam->setCaptureMode(false);

    while (true) {
        if ((NULL != lcam->myCurrentCapturingImage.data) && (false == lcam->isCapturingFlag)) {
            image = lcam->myCurrentCapturingImage;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    lcam->setCaptureMode(true); 
}

void CameraController::captureUnrectifiedImages(cv::Mat &imageLeft, cv::Mat &imageRight)
{
    CameraDevice* lcam, *rcam;
    rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
    lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    if (m_pDeviceProcess[0]->getLongIntSN() < m_pDeviceProcess[1]->getLongIntSN()) {
        lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
        rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    }

    lcam->isCapturingFlag = true;
    lcam->setCaptureMode(false);
    rcam->isCapturingFlag = true;
    rcam->setCaptureMode(false);

    while (true) {
        if ((NULL != lcam->myCurrentCapturingImage.data) && (false == lcam->isCapturingFlag)
            && (NULL != rcam->myCurrentCapturingImage.data) && (false == rcam->isCapturingFlag)) { 
            imageLeft = lcam->myCurrentCapturingImage;
            imageRight = rcam->myCurrentCapturingImage;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    lcam->setCaptureMode(true);
    rcam->setCaptureMode(true);
}

void CameraController::clearImages()
{
    for (int i = 0; i < NUM_CAMERA; i++) {
        if (0 != m_pDeviceProcess[i]) {
            std::queue<cv::Mat> empty;
            std::swap( m_pDeviceProcess[i]->myCapturedImage, empty );
        }
    }
}

bool CameraController::getCapturedImages(cv::Mat &leftImage, cv::Mat &rightImage)
{
    cv::Mat leftImage_temp, rightImage_temp;

    CameraDevice* lcam, *rcam;
    rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
    lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    if (m_pDeviceProcess[0]->getLongIntSN() < m_pDeviceProcess[1]->getLongIntSN()) {
        lcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[0] : m_pDeviceProcess[1];
        rcam = CameraDevice::isDeviceDirectionLeftCamLowSn() ? m_pDeviceProcess[1] : m_pDeviceProcess[0];
    }

    lcam->isCapturingFlag = true;
    rcam->isCapturingFlag = true;
    
    auto start = std::chrono::steady_clock::now();

    while (true) {
        if ((NULL != lcam->myCurrentCapturingImage.data) && (false == lcam->isCapturingFlag)
            && (NULL != rcam->myCurrentCapturingImage.data) && (false == rcam->isCapturingFlag)) {
            leftImage_temp = lcam->myCurrentCapturingImage;
            rightImage_temp = rcam->myCurrentCapturingImage;
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

    leftImage = leftImage_temp.clone();
    rightImage = rightImage_temp.clone();
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
void CameraController::setExposureTime2D(int exposureTime2D, bool saveFlag)
{    
    this->exposureTime2D = exposureTime2D;
    //if (saveFlag) {
    //    QString exposureTime2DString = QString("%1").arg(exposureTime2D);
    //    RegistryManager::getInstance().setValue(CONFIG_TDIMAGE, KEY_TDIMAGE_EXPOSURETIME, exposureTime2DString);
    //}

    // set exposureTime2D to both cameras
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        m_pDeviceProcess[idx]->setExposureTime2D(exposureTime2D);
    }
}

int CameraController::getExposureTime2D()
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
void CameraController::setExposureTime3D(int exposureTime3D, bool saveFlag)
{    
    this->exposureTime3D = exposureTime3D;  
    //if (saveFlag) {
    //    QString exposureTime3DString = QString("%1").arg(exposureTime3D);
    //    RegistryManager::getInstance().setValue(CONFIG_CAPTURE, KEY_CAPTURE_EXPOSURETIME3DSPINBOX, exposureTime3DString);
    //}
    
    // set exposureTime3D to both cameras
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        m_pDeviceProcess[idx]->setExposureTime3D(exposureTime3D);
    }
}

int CameraController::getExposureTime3D()
{
    return exposureTime3D;
}

void CameraController::setPatternsNum(int patternsNum)
{
    this->patternsNum = patternsNum;
    // set patternsNum to both cameras
    for (int idx = 0; idx < NUM_CAMERA; idx++) {
        m_pDeviceProcess[idx]->setPatternsNum(patternsNum);
    }
}

bool CameraController::getConnectedCameraSerialNumberStrings(std::vector<std::string>& connectedCamSNs)
{
    if (camSerialNumbers.empty()) {
        return false;
    }
    
    connectedCamSNs.clear();
    connectedCamSNs = camSerialNumbers;

    return true;
}

int CameraController::getSensorWidth()
{
    return m_pDeviceProcess[0]->getSensorWidth();
}

int CameraController::getSensorHeight()
{
    return m_pDeviceProcess[0]->getSensorHeight();
}

void CameraController::enumDevices()
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

int CameraController::openDeviceForGetDeviceInfo()
{
    // get the device information
    for(int i=0; i<NUM_CAMERA; i++) {
        m_pDeviceProcess[i]->OpenDevice(camSerialNumbers[i], i);
        m_pDeviceProcess[i]->getDeviceInfo();
    }
    
    // set pParameter->resolutionWidth and pParameter->resolutionHeight        
    //pParameter->resolutionWidth = m_pDeviceProcess[0]->getSensorWidth();
    //pParameter->resolutionHeight = m_pDeviceProcess[0]->getSensorHeight();          
    //PRINTF_INFO("pParameter->resolutionWidth : %d", pParameter->resolutionWidth); 
    //PRINTF_INFO("pParameter->resolutionHeight : %d", pParameter->resolutionHeight);

    return 0;
}



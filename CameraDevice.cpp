
#include "CameraDevice.h"
#include <chrono>
#include <thread>

CameraDevice::CameraDevice()
{
    m_pCaptureEventHandle       = NULL;
    m_bIsOpen                   = false;
    m_bIsSnap                   = false;
    m_nDeviceIndex              = 0;
    m_strDeviceSN               = "";
    m_pCaptureEventHandle       = new CaptureEventHandler();

    isTriggerMode               = false;
    isCapturingFlag             = false;
    capture3DModelFinishFlag    = false;
    needImagesCount             = 1;

    cameraControllerFunc        = NULL;

    exposureTime2D              = 8000;  // default value. no special use
    exposureTime3D              = 500;   // default value. no special use

    m_device_handle             = NULL;
    status                      = GX_STATUS_ERROR;
    patternsNum                 = 11;
}


CameraDevice::~CameraDevice()
{
    if (m_pCaptureEventHandle != NULL) {
        delete m_pCaptureEventHandle;
        m_pCaptureEventHandle = NULL;
    }
}

//------------------------------------------------------------
/**
\brief   Open Device 
\param   strDeviceSN   [in]         设备序列号
\param   nDeviceIndex  [in]         设备序号
\return  void
*/
//------------------------------------------------------------
void CameraDevice::OpenDevice(std::string& strDeviceSN, int nDeviceIndex)
{
    if (m_bIsOpen || strDeviceSN == "")  {
        PRINTF_ERROR("strDeviceSN is null");
        throw "strDeviceSN is null";
    }
    
    m_strDeviceSN = strDeviceSN;
    m_nDeviceIndex  = nDeviceIndex;

    //打开设备
    GX_OPEN_PARAM open_param;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode   = GX_OPEN_SN;
    open_param.pszContent = (char*)strDeviceSN.c_str();
    
    //若相机已被打开则先关闭
    if (NULL != m_device_handle) {
        status = GXCloseDevice(m_device_handle);
        GX_VERIFY(status);
        m_device_handle = NULL;
    }

    //打开相机
    status = GXOpenDevice(&open_param, &m_device_handle);
    GX_VERIFY(status);
    m_bIsOpen = true;

    // 设置是否是触发模式
    setCaptureMode(isTriggerMode);
}

//------------------------------------------------------------
/**
\brief   Close Device 
\return  void
*/
//------------------------------------------------------------
void CameraDevice::CloseDevice()
{
    if (!m_bIsOpen) {
        return;
    }
    
    //判断是否停止采集
    if (m_bIsSnap) {
        //发送停采命令
        status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_STOP);
        GX_VERIFY(status);

        //注销回调函数
        status = GXUnregisterCaptureCallback(m_device_handle);
        GX_VERIFY(status);
    }

    //关闭设备
    status = GXCloseDevice(m_device_handle);
    GX_VERIFY(status);
    m_device_handle = NULL;

    m_bIsSnap = false;
    m_bIsOpen = false;
}

//------------------------------------------------------------
/**
\brief   Start Snap 
\return  void
*/
//------------------------------------------------------------
void CameraDevice::StartSnap()
{
    //判断设备是否已打开
    if (!m_bIsOpen) {
        return;
    }

    //注册回调函数
    status = GXRegisterCaptureCallback(m_device_handle, this, m_pCaptureEventHandle->OnFrameCallbackFun);
    GX_VERIFY(status);

    //发送开采命令
    status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_START);
    GX_VERIFY(status);

    m_bIsSnap = true;
}

//------------------------------------------------------------
/**
\brief   Stop Snap 
\return  void
*/
//------------------------------------------------------------
void CameraDevice::StopSnap()
{
    //判断设备是否已打开
    if (!m_bIsOpen || !m_bIsSnap) {
        return;
    }

    //发送停采命令
    status = GXSendCommand(m_device_handle, GX_COMMAND_ACQUISITION_STOP);
    GX_VERIFY(status);

    //注销回调函数
    status = GXUnregisterCaptureCallback(m_device_handle);
    GX_VERIFY(status);

    m_bIsSnap = false;
}

//------------------------------------------------------------
/**
\brief   Get Device Open Flag 
\return  void
*/
//------------------------------------------------------------
bool CameraDevice::IsOpen() const
{
    return m_bIsOpen;
}

//------------------------------------------------------------
/**
\brief   Get Device Snap Flag 

\return  void
*/
//------------------------------------------------------------
bool CameraDevice::IsSnap() const
{
    return m_bIsSnap;
}

long CameraDevice::getLongIntSN()
{
    std::string temp_str = m_strDeviceSN.substr(m_strDeviceSN.length() - 3);
    m_lnDeviceSN = std::stoi(temp_str);
    return m_lnDeviceSN;
}

std::string CameraDevice::getSNStr()
{
    return m_strDeviceSN;
}

bool CameraDevice::isDeviceDirectionLeftCamLowSn()
{
    return (CAM_DIRECTION == LEFT_CAMERA_LOW_SN);
}

std::string CameraDevice::getDeviceModelName()
{
    return m_DeviceModelName;
}

void CameraDevice::setExposureTime2D(int exposureTime2D)
{
    this->exposureTime2D = exposureTime2D;
    if (IsOpen() && (!isTriggerMode)) {
        status = GXSetFloat(m_device_handle, GX_FLOAT_EXPOSURE_TIME, (double)exposureTime2D);
        GX_VERIFY(status);
    }      
}

void CameraDevice::setExposureTime3D(int exposureTime3D)
{
    this->exposureTime3D = exposureTime3D;
    if (IsOpen() && isTriggerMode) {
        status = GXSetFloat(m_device_handle, GX_FLOAT_EXPOSURE_TIME, (double)exposureTime3D);
        GX_VERIFY(status);
    }
}

void CameraDevice::setPatternsNum(int patternsNum)
{
    this->patternsNum = patternsNum;
}

int CameraDevice::getSensorWidth()
{
    return m_SensorWidth;
}

int CameraDevice::getSensorHeight()
{
    return m_SensorHeight;
}

void CameraDevice::getDeviceInfo()
{    
    // get m_SensorWidth and m_SensorHeight
    int64_t sensorWidth, sensorHeight;
    status = GXGetInt(m_device_handle, GX_INT_SENSOR_WIDTH, &sensorWidth);
    GX_VERIFY(status);
    status = GXGetInt(m_device_handle, GX_INT_SENSOR_HEIGHT, &sensorHeight);
    GX_VERIFY(status);
    m_SensorWidth = (int)sensorWidth;
    m_SensorHeight = (int)sensorHeight;

    // get device model name
    size_t nSize = 0;
    status = GXGetStringMaxLength(m_device_handle, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY(status);    
    char pszText[256] = {0};     
    status = GXGetString(m_device_handle, GX_STRING_DEVICE_MODEL_NAME, pszText, &nSize);
    GX_VERIFY(status);
    m_DeviceModelName = (std::string)pszText;
}


void CameraDevice::registerCallBackCameraController(callbackCameraController func)
{
    this->cameraControllerFunc = func;
}

bool CameraDevice::setTriggerSource(int triggerSource)
{
    if (triggerSource == GX_TRIGGER_SOURCE_SOFTWARE) {
        trigger_source = GX_TRIGGER_SOURCE_SOFTWARE;
        PRINTF_INFO("trigger source: TRIGGER_SOURCE_SOFTWARE");
    }
    else if (triggerSource == GX_TRIGGER_SOURCE_LINE2) {
        trigger_source = GX_TRIGGER_SOURCE_LINE2;
        PRINTF_INFO("trigger source: TRIGGER_SOURCE_LINE2");
    }
    else if (triggerSource == GX_TRIGGER_SOURCE_LINE0) {
        trigger_source = GX_TRIGGER_SOURCE_LINE0;
        PRINTF_INFO("trigger source: TRIGGER_SOURCE_LINE0");
    }
    else if (triggerSource == GX_TRIGGER_SOURCE_LINE1) {
        trigger_source = GX_TRIGGER_SOURCE_LINE1;
        PRINTF_INFO("trigger source: TRIGGER_SOURCE_LINE1");
    }
    else if (triggerSource == GX_TRIGGER_SOURCE_LINE3) {
        trigger_source = GX_TRIGGER_SOURCE_LINE3;
        PRINTF_INFO("trigger source: TRIGGER_SOURCE_LINE3");
    }
    else {
        PRINTF_INFO("trigger source: invalid");
        return false;
    }

    return true;
}

void CameraDevice::setCaptureMode(bool isTriggerMode)
{
    this->isTriggerMode = isTriggerMode;
    if (isTriggerMode) {
        status = GXSetFloat(m_device_handle, GX_FLOAT_EXPOSURE_TIME, (double)exposureTime3D);
        GX_VERIFY(status);
        
        // set capture mode to trigger mode
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_SOURCE, trigger_source);
        GX_VERIFY(status);
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
        GX_VERIFY(status);
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        GX_VERIFY(status);
    } 
    else {              
        status = GXSetEnum(m_device_handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        GX_VERIFY(status);
        
        // set exposure time
        status = GXSetFloat(m_device_handle, GX_FLOAT_EXPOSURE_TIME, (double)exposureTime2D);
        GX_VERIFY(status);
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(exposureTime2D, exposureTime3D) / 1000 + 100));
        // PRINTF_INFO("exposureTime2D : %d", exposureTime2D);
        
        // set capture mode to continuous mode  
        status = GXSetEnum(m_device_handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        GX_VERIFY(status);
    }
}

bool CameraDevice::softTrigger()
{
    if (trigger_source != GX_TRIGGER_SOURCE_SOFTWARE) { 
        PRINTF_ERROR("current trigger source is not GX_TRIGGER_SOURCE_SOFTWARE");
        return false; 
    }

    status = GX_STATUS_ERROR;
    status = GXSendCommand(m_device_handle, GX_COMMAND_TRIGGER_SOFTWARE);
    GX_VERIFY(status);

    return true;
}

void CameraDevice::showErrorString(GX_STATUS error_status)
{
    char*     error_info = NULL;
    size_t    size        = 0;
    GX_STATUS status     = GX_STATUS_ERROR;

    // get the length of the error message and requests memory space 
    status = GXGetLastError(&error_status, NULL, &size);
    error_info = new char[size];
    if (NULL == error_info) {
        PRINTF_ERROR("Error : error_info apply memory failed");
        return;
    }

    // show the error information
    status = GXGetLastError (&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS) {
        PRINTF_ERROR("Error : GXGetLastError call failed!");
    }
    else {
        PRINTF_ERROR("Error : %s!", error_info);
    }

    // free the memory of error_info
    if (NULL != error_info) {
        delete[] error_info;
        error_info = NULL;
    }
}

void GX_STDC CameraDevice::CaptureEventHandler::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
        CameraDevice* pDeviceProcess = (CameraDevice*)pFrame->pUserParam;
        pDeviceProcess->m_pCaptureEventHandle->DoOnImageCaptured(pFrame);
    }    
    else {
        PRINTF_ERROR("OnFrameCallbackFun status is not GX_FRAME_STATUS_SUCCESS");
    }
}

void CameraDevice::CaptureEventHandler::DoOnImageCaptured(GX_FRAME_CALLBACK_PARAM * pFrame)
{
    CameraDevice* pDeviceProcess = (CameraDevice*)pFrame->pUserParam;
    int cols = (int)pFrame->nWidth;
    int rows = (int)pFrame->nHeight;
    BYTE* pBuffer = (BYTE*)pFrame->pImgBuf;
    cv::Mat cvImage = cv::Mat(rows, cols, CV_8UC1, pBuffer);
    if (pDeviceProcess->isTriggerMode)  {
        if (!pDeviceProcess->capture3DModelFinishFlag) {   //如果获取完成，则再trigger也没用,再次获取需要设置
            pDeviceProcess->myCapturedImage.push(cvImage.clone());
            if (pDeviceProcess->myCapturedImage.size() == (pDeviceProcess->patternsNum*pDeviceProcess->needImagesCount)) {
                pDeviceProcess->capture3DModelFinishFlag = true;
                // callback to CameraControllers 
                if(!(pDeviceProcess->cameraControllerFunc)) {
                    PRINTF_SMARTEYE("registerCallBackCameraController has not been called before.");
                    return;
                }                           
                std::thread((pDeviceProcess->cameraControllerFunc)).detach(); // use thread and detach to make a asynchronous call. fix error of stop camera after single 3D capture
            }
        }
    }
    else {
        pDeviceProcess->isCapturingFlag = true;
        pDeviceProcess->myCurrentCapturingImage = cvImage;
        //pDeviceProcess->myCurrentCapturingImage = cvImage.clone();
        pDeviceProcess->isCapturingFlag = false;
    }
}



#pragma once

#include <queue>
#include <memory>
#include <functional>
#include <stdio.h>

#include "GxIAPI.h"
#include <opencv2/opencv.hpp>


// output switch
#ifdef  QT_NO_DEBUG
    #define PRINTF_SMARTEYE(fmt, ...) /* do nothing */
#else
    #define PRINTF_SMARTEYE(fmt, ...) printf(("%s(%d)\t" fmt "\n"), __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#define PRINTF_INFO(fmt, ...) printf(("%s(%d)\t" fmt "\n"), __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define PRINTF_ERROR(fmt, ...) printf(("%s(%d)\t" fmt "\n"), __FUNCTION__, __LINE__, ##__VA_ARGS__)


#define LEFT_CAMERA_LOW_SN      0                       //装配时左相机的序列号较小
#define LEFT_CAMERA_GREAT_SN    1                       //装配时左相机的序列号较大
#define CAM_DIRECTION LEFT_CAMERA_GREAT_SN              //定义装配时左相机的序列号较大 (因为实验设备目前如此)


/// 错误提示函数宏定义
#define  GX_VERIFY(status) \
    if(status != GX_STATUS_SUCCESS) \
    { \
       PRINTF_ERROR("camera error"); \
       CameraDevice::showErrorString(status); \
       throw "camera error"; \
    }


class CameraDevice
{
public:
    CameraDevice();
    ~CameraDevice();
    using callbackCameraController = std::function<void(void)> ;

    //---------------------------------------------------------------------------------
    /**
    \brief   用户继承采集事件处理类
    */
    //----------------------------------------------------------------------------------
    class CaptureEventHandler
    {        
    public:
        static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
        
        void DoOnImageCaptured(GX_FRAME_CALLBACK_PARAM* pFrame);
    };
    
    /// 打开设备
    void OpenDevice(std::string& strDeviceSN, int nDeviceIndex);

    /// 关闭设备
    void CloseDevice();

    /// 开始采集
    void StartSnap();

    /// 停止采集
    void StopSnap();

    /// 获取设备打开标识
    bool IsOpen() const;

    /// 获取设备采集标识
    bool IsSnap() const;

    long getLongIntSN();
    std::string getSNStr();
    static bool isDeviceDirectionLeftCamLowSn();
    std::string getDeviceModelName();    

    void setExposureTime2D(int exposureTime2D);
    void setExposureTime3D(int exposureTime3D);
    void setPatternsNum(int patternsNum);
    
    int getSensorWidth();
    int getSensorHeight();
    void getDeviceInfo();

    void registerCallBackCameraController(callbackCameraController func);
    bool setTriggerSource(int triggerSource);
    void setCaptureMode(bool isTriggerMode);
    bool softTrigger();

    static void showErrorString(GX_STATUS error_status);

    GX_DEV_HANDLE               m_device_handle;                ///< 设备句柄
    CaptureEventHandler*        m_pCaptureEventHandle;          ///< 回调指针
    bool                        m_bIsOpen;                      ///< 设备是否打开标志
    bool                        m_bIsSnap;                      ///< 设备是否开采标志
    std::string                 m_strDeviceSN;                  ///< 设备序列号
    int                         m_nDeviceIndex;                 ///< 设备序号   
    int                         m_SensorWidth;                  ///< 传感器宽度
    int                         m_SensorHeight;                 ///< 传感器高度
    std::string                 m_DeviceModelName;              ///< 模型名称
    
    std::queue<cv::Mat>         myCapturedImage;
    bool                        isTriggerMode;
    int                         captureImagesCount;
    int                         needImagesCount;
    bool                        isCapturingFlag;                //是否正在获取，这个时候别的程序是不允许调用当前图像，否则出错
    bool                        capture3DModelFinishFlag;
    cv::Mat                     myCurrentCapturingImage;
    int                         exposureTime2D;
    int                         exposureTime3D;
    long                        m_lnDeviceSN;                   //m_strDeviceSN 以KR开头序列号的字符串转为数字

    callbackCameraController    cameraControllerFunc;    

private:    
    GX_STATUS status;
    GX_TRIGGER_SOURCE_ENTRY     trigger_source = GX_TRIGGER_SOURCE_LINE2;

    int patternsNum;
};


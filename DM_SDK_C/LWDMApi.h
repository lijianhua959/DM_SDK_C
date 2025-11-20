
//
// Created by ljh on 2024/8/15.
//

/**@file      LWDMApi.h
 *
 * @brief     includes camera control and data receiving interface, which supports configuration for image resolution,  frame rate, exposure time, gain, working mode,etc.
 *
 */

 /**@mainpage
 *
 * @copyright Copyright(C)2024-2026 Percipio All Rights Reserved
 *
 * @section sec1  Introduction
 *	Welcome to the D-series ToF camera API documentation. This documentation enables you to quickly get started in your development efforts to programmatically interact with the ToF Camera (eg:LWP-DM02C).
 *
 * @section sec2 Description
 *   The 3D camera, called "device", consists of several components. Each component is a hardware module or virtual module, such as RGB sensor, depth sensor(TOF sensor).
 *
 *   Each component has its own features, such as image width, exposure time, etc..
 *
 *   Please pay attention to the correspondence between frame rate or exposure time when setting them, otherwise unexpected things may occur. Suggest setting the exposure time around the frame rate.
 *
 *	 The depth sensor chip has a temperature detection module, so it can obtain temperature data from depth data. Due to the temperature detection function of the device, it will perform frame rate reduction data sampling when the device temperature exceeds the threshold.
 *
 * @section sec3 The relationship between points and frame rate
 *	 The corresponding relationship is as follows.
 *		\code
 *		-----------------------------------------------------
 *		Frame rate	|	Maximum exposure time of ToF sensor	|
 *		
 *		待定...
 *		\endcode
 *
 */

#ifndef LW_DM_API_H
#define LW_DM_API_H


#include "LWDMType.h"


/// @brief 资源初始化。必须在调用其他API接口之前先调用此函数。
/// @return 返回码。
LW_C_API LWReturnCode LWInitializeResources();

/// @brief 清理所有资源。调用此函数后，无法再调用其他API接口。
/// @return 返回码。
LW_C_API LWReturnCode LWCleanupResources();

/// @brief 获取所有可使用的设备信息列表。
/// @param[out] deviceInfoList 类型为“LWDeviceInfo”的数组。
/// @param[in] listLength 数组“deviceInfoList”的大小。
/// @param[out] fillCount 数组填充数即查询到的设备数量。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDeviceInfoList(LWDeviceInfo* deviceInfoList, int32_t listLength, int32_t* fillCount);

/// @brief 查找并获取所有可使用的设备句柄列表。从1.2.0.0版本开始，不再建议使用该函数，请改用“LWGetDeviceInfoList”函数。
/// @param[out] handleList 类型为“LWDeviceHandle”的数组。
/// @param[in] listLength 数组“handleList”的大小。
/// @param[out] fillCount 数组填充数即查询到的设备数量。
/// @return 返回码。
LW_C_API LWReturnCode LWFindDevices(LWDeviceHandle* handleList, int32_t listLength, int32_t* fillCount);

/// @brief 打开设备。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWOpenDevice(LWDeviceHandle handle);

/// @brief 关闭设备。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWCloseDevice(LWDeviceHandle handle);

/// @brief 当出现网络异常断开时，才可调用此函数进行设备重连。
/// @param[in] handle 设备描述符。
/// @param[in] msec 重连超时。单位：毫秒。
/// @return 返回码。
LW_C_API LWReturnCode LWReconnectDevice(LWDeviceHandle handle, uint32_t msec);

/// @brief 设备（系统）重启。设备重启后SDK不会重连设备，因此需要重新打开设备。
/// @note 不能在数据流开启的情况下调用此函数。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWRebootDevice(LWDeviceHandle handle);

/// @brief 保存设备当前的各种配置参数（比如：积分时间、触发模式、置信度滤波...），下次打开设备时以此配置信息运行。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWSaveConfigureInfo(LWDeviceHandle handle);

/// @brief 删除设备保存的配置参数。
/// @note 设备会自动重启，重新加载默认配置信息。设备重启后SDK不会重连设备，因此需要重新打开设备。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWRemoveConfigureInfo(LWDeviceHandle handle);

/// @brief 恢复出厂设置，与“LWRemoveConfigureInfo”函数相比只多了一项网络配置重置的额外操作。
/// @note 设备会自动重启，重新加载默认配置信息。设备重启后SDK不会重连设备，因此需要重新打开设备。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWRestoreFactoryConfigureInfo(LWDeviceHandle handle);

/// @brief 打开设备数据流。在此之前请先设置好相应配置（比如：各种滤波、触发模式、积分时间...），否则为之前保存的各种配置参数。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWStartStream(LWDeviceHandle handle);

/// @brief 关闭设备数据流。调用此函数之后便无法获取设备数据，除非再次打开设备数据流。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWStopStream(LWDeviceHandle handle);

/// @brief 查看设备是否具有RGB模块。
/// @param[in] handle 设备描述符。
/// @param[out] value 值为真时则设备具有RGB模块，反之则不具有。
/// @return 返回码。
LW_C_API LWReturnCode LWHasRgbModule(LWDeviceHandle handle, bool* value);

/// @brief 在开启设备数据流之后，每调用一次该函数时设备便发送一帧数据。
/// @note 设备必须处于软触发模式。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWSoftTrigger(LWDeviceHandle handle);

/// @brief 设置数据接收类型。默认接收类型为：LW_DEPTH_IR_RTY。
/// @note 设备只会发送该类型数据。例如，当设置为“LW_IR_RTY”类型时，就只能获得IR数据，而无法获取深度、点云等其他数据。
/// @param[in] handle 设备描述符。
/// @param[in] type 数据接收类型。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDataReceiveType(LWDeviceHandle handle, LWDataRecvType type);

/// @brief 设置各操作的执行超时时间。默认为3000毫秒。
/// @param[in] handle 设备描述符。
/// @param[in] msec 超时时间。单位：毫秒。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTimeout(LWDeviceHandle handle, uint32_t msec);

/// @brief 设置设备的触发模式。
/// @note 必须在开启设备数据流之前进行设置。
/// @param[in] handle 设备描述符。
/// @param[in] mode 触发模式。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTriggerMode(LWDeviceHandle handle, LWTriggerMode mode);

/// @brief 获取设备当前的触发模式。
/// @param[in] handle 设备描述符。
/// @param[out] mode 触发模式。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTriggerMode(LWDeviceHandle handle, LWTriggerMode* mode);

/// @brief 设置设备的曝光模式。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[in] mode 曝光模式。
/// @return 返回码。
LW_C_API LWReturnCode LWSetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode mode);

/// @brief 获取设备当前的曝光模式。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[out] mode 曝光模式。
/// @return 返回码。
LW_C_API LWReturnCode LWGetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode* mode);

/// @brief 设置TOF传感器的HDR模式。请注意每种模式下的最大帧率（详见：“LWHDRMode”枚举类型）。
/// @note 必须在开启设备数据流之前进行设置，以及设置相对应的曝光时间（详见：“LWHDRMode”枚举类型）。
/// @param[in] handle 设备描述符。
/// @param[in] mode HDR模式。
/// @return 返回码。
LW_C_API LWReturnCode LWSetHDRMode(LWDeviceHandle handle, LWHDRMode mode);

/// @brief 获取TOF传感器的HDR模式。
/// @param[in] handle 设备描述符。
/// @param[out] mode HDR模式。
/// @return 返回码。
LW_C_API LWReturnCode LWGetHDRMode(LWDeviceHandle handle, LWHDRMode* mode);

/// @brief 设置深度(或IR)数据映射到RGB的使能开关，默认不开启此项。
/// @note 映射后的深度图像的分辨率与RGB图像的分辨率相同。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTransformDepthToRgbEnable(LWDeviceHandle handle, bool enable);

/// @brief 设置RGB数据映射到深度(或IR)的使能开关，默认不开启此项。
/// @note 映射后的RGB图像的分辨率与深度图像的分辨率相同。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTransformRgbToDepthEnable(LWDeviceHandle handle, bool enable);

/// @brief 设置数据的发送帧率。由于网络（带宽、丢包...）、设备性能（CPU、内存...）等因素导致实际帧率会略低于此设定值。
/// @note 由于帧率与曝光时间呈负相关即曝光时间越大帧率就越低，因此需要理清曝光时间与帧率的对应关系。
/// @param[in] handle 设备描述符。
/// @param[in] value 帧率值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetFrameRate(LWDeviceHandle handle, int32_t value);

/// @brief 获取设备当前的数据发送帧率。
/// @param[in] handle 设备描述符。
/// @param[out] value 帧率值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetFrameRate(LWDeviceHandle handle, int32_t* value);

/// @brief 设置设备的曝光时间。请注意每种HDR模式下的曝光时间设置个数（详见：“LWHDRMode”枚举类型）。
/// @note 由于帧率与曝光时间呈负相关即曝光时间越大帧率就越低，因此需要理清曝光时间与帧率的对应关系。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[in] etArray 指向数组的指针。
/// @param[in] arraySize 数组“etArray”的大小。
/// @return 返回码。
LW_C_API LWReturnCode LWSetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, const int32_t* etArray, int32_t arraySize);

/// @brief 获取设备的曝光时间。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[out] etArray 指向数组的指针。
/// @param[in] arraySize 数组“etArray”的大小。
/// @param[out] filledCount 填充的个数。
/// @return 返回码。
LW_C_API LWReturnCode LWGetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, int32_t* etArray, int32_t arraySize, int32_t* filledCount);

/// @brief 设置时域滤波。取值范围：2-28，k1取值范围：1-10000。
/// @note k1默认值为：100，当前版本不需要更改其值。
/// @param[in] handle 设备描述符。
/// @param[in] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTimeFilterParams(LWDeviceHandle handle, LWFilterParam param);

/// @brief 获取时域均值滤波。
/// @param[in] handle 设备描述符。
/// @param[out] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTimeFilterParams(LWDeviceHandle handle, LWFilterParam* param);

/// @brief 设置时域中值滤波。取值范围：3, 5, 7, 9，k1取值范围：1-10000。
/// @note k1默认值为：100，当前版本不需要更改其值。
/// @param[in] handle 设备描述符。
/// @param[in] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam param);

/// @brief 获取时域中值滤波。
/// @param[in] handle 设备描述符。
/// @param[out] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam* param);

/// @brief 设置空间滤波。取值范围：3, 5, 7。
/// @param[in] handle 设备描述符。
/// @param[in] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam param);

/// @brief 获取空间滤波。
/// @param[in] handle 设备描述符。
/// @param[out] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam* param);

/// @brief 设置飞点滤波。取值范围：1-64。
/// @param[in] handle 设备描述符。
/// @param[in] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam param);

/// @brief 获取飞点滤波。
/// @param[in] handle 设备描述符。
/// @param[out] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam* param);

/// @brief 设置置信度滤波。取值范围：1-150。
/// @param[in] handle 设备描述符。
/// @param[in] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam param);

/// @brief 获取置信度滤波。
/// @param[in] handle 设备描述符。
/// @param[out] param 滤波值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam* param);

/// @brief 设置IR伽马值。取值范围：0-255。
/// @param[in] handle 设备描述符。
/// @param[in] value 伽马值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetIRGMMGain(LWDeviceHandle handle, int32_t value);

/// @brief 获取IR伽马值。
/// @param[in] handle 设备描述符。
/// @param[out] value 伽马值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetIRGMMGain(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的增益值。取值范围：16-988，默认值：0。
/// @param[in] handle 设备描述符。
/// @param[in] value 增益值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorGain(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的增益值。
/// @param[in] handle 设备描述符。
/// @param[out] value 增益值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorGain(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的伽马值。取值范围：64-300，默认值：110。
/// @param[in] handle 设备描述符。
/// @param[in] value 伽马值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorGamma(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的伽马值。
/// @param[in] handle 设备描述符。
/// @param[out] value 伽马值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorGamma(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的亮度值。取值范围：-64-64，默认值：0。
/// @param[in] handle 设备描述符。
/// @param[in] value 亮度值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorBrightness(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的亮度值。
/// @param[in] handle 设备描述符。
/// @param[out] value 亮度值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorBrightness(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的对比度。取值范围：0-95，默认值：0。
/// @param[in] handle 设备描述符。
/// @param[in] value 对比度。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的对比度。
/// @param[in] handle 设备描述符。
/// @param[out] value 对比度。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的饱和度。
/// @param handle 设备描述符。
/// @param value 饱和度。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorSaturation(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的饱和度。
/// @param handle 设备描述符。
/// @param value 饱和度。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorSaturation(LWDeviceHandle handle, int32_t* value);

/// @brief 设置RGB传感器的白平衡。
/// @param handle 设备描述符。
/// @param value 白平衡。
/// @return 返回码。
LW_C_API LWReturnCode LWSetRgbSensorWhiteBalance(LWDeviceHandle handle, int32_t value);

/// @brief 获取RGB传感器的白平衡。
/// @param handle 设备描述符。
/// @param value 白平衡。
/// @return 返回码。
LW_C_API LWReturnCode LWGetRgbSensorWhiteBalance(LWDeviceHandle handle, int32_t* value);

/// @brief 设置设备的网络配置信息。
/// @param[in] handle 设备描述符。
/// @param[in] info 网络配置信息。
/// @return 返回码。
LW_C_API LWReturnCode LWSetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo info);

/// @brief 获取设备当前的网络配置信息。
/// @param[in] handle 设备描述符。
/// @param[out] info 网络配置信息。
/// @return 返回码。
LW_C_API LWReturnCode LWGetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo* info);

/// @brief 设置设备编号。取值范围：1-255。
/// @param[in] handle 设备描述符。
/// @param[in] value 编号。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDeviceNumber(LWDeviceHandle handle, int32_t value);

/// @brief 获取设备当前的编号。
/// @param[in] handle 设备描述符。
/// @param[out] value 编号。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDeviceNumber(LWDeviceHandle handle, int32_t* value);

/// @brief 设置深度数据补偿值。取值范围：0-65535，默认值为：0。
/// @param[in] handle 设备描述符。
/// @param[in] value 补偿值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDepthCompensateValue(LWDeviceHandle handle, int32_t value);

/// @brief 获取设备当前的深度数据补偿值。
/// @param[in] handle 设备描述符。
/// @param[out] value 补偿值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDepthCompensateValue(LWDeviceHandle handle, int32_t* value);

/// @brief 设置硬件延时触发的滤波参数，仅在触发模式为“LW_TRIGGER_HARD_FILTER”时生效。一个信号周期为：t1 / 1000 + t2，单位：毫秒。
/// @param[in] handle 设备描述符。
/// @param[in] t1 触发信号（电平信号）的持续时间。单位：微秒，取值范围：1000~65535。
/// @param[in] t2 触发信号（电平信号）的间隔时间。单位：毫秒，取值范围：50~65535。
/// @return 返回码。
LW_C_API LWReturnCode LWSetHardTriggerFilterParams(LWDeviceHandle handle, int32_t t1, int32_t t2);

/// @brief 获取设备当前的硬触发滤波参数。
/// @param[in] handle 设备描述符。
/// @param[out] t1 触发信号（电平信号）的持续时间。单位：微秒。
/// @param[out] t2 触发信号（电平信号）的间隔时间。单位：毫秒。
/// @return 返回码。
LW_C_API LWReturnCode LWGetHardTriggerFilterParams(LWDeviceHandle handle, int32_t* t1, int32_t* t2);

/// @brief 获取传感器的分辨率。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[out] width 图像宽度即水平分辨率。
/// @param[out] height 图像高度即垂直分辨率。
/// @return 返回码。
LW_C_API LWReturnCode LWGetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t* width, int32_t* height);

/// @brief 获取传感器的内参。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型（TOF/RGB）。
/// @param[out] param 内参。
/// @return 返回码。
LW_C_API LWReturnCode LWGetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam* param);

/// @brief 获取传感器的外参。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 已忽略。包含该参数只是为了与之前发布的版本兼容。
/// @param[out] param 外参。
/// @return 返回码。
LW_C_API LWReturnCode LWGetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam* param);

/// @brief 获取IMU传感器的外参。
/// @param[in] handle 设备描述符。
/// @param[out] param 外参。
/// @return 返回码。
LW_C_API LWReturnCode LWGetIMUExtrinsicParam(LWDeviceHandle handle, LWIMUExtrinsicParam* param);

/// @brief 获取IMU传感器的位姿数据。
/// @param[in] handle 设备描述符。
/// @param[out] data IMU位姿数据。
/// @return 返回码。
LW_C_API LWReturnCode LWGetIMUData(LWDeviceHandle handle, LWIMUData* data);

/// @brief 设置IMU传感器的采样频率，取值范围：50、100、120、200、500、1000、2000，默认值：120。
/// @param[in] handle 设备描述符。
/// @param[in] value 频率(Hz)。
/// @return 返回码。
LW_C_API LWReturnCode LWSetIMUFrequency(LWDeviceHandle handle, int32_t value);

/// @brief 获取设备的SN号（产品序列号）。
/// @param[in] handle 设备描述符。
/// @param[out] buffer SN号的接收缓存区，建议不低于16字节。
/// @param[in] bufferLen 缓存区“buffer”的大小。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDeviceSN(LWDeviceHandle handle, char* buffer, int32_t bufferLen);

/// @brief 获取设备的类型。
/// @param[in] handle 设备描述符。
/// @param[out] buffer 类型的接收缓存区，建议不低于16字节。
/// @param[in] bufferLen 缓存区“buffer”的大小。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDeviceType(LWDeviceHandle handle, char* buffer, int32_t bufferLen);

/// @brief 获取设备当前的系统时间。
/// @param[in] handle 设备描述符。
/// @param[out] t 时间。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTimeStamp(LWDeviceHandle handle, LWTimeStamp* t);

/// @brief 获取当前SDK库的版本信息。
/// @param[out] version 版本信息。
/// @return 返回码。
LW_C_API LWReturnCode LWGetLibVersion(LWVersionInfo* version);

/// @brief 获取设备的版本信息。
/// @param[in] handle 设备描述符。
/// @param[out] fv 固件版本信息。
/// @param[out] dv 驱动版本信息。
/// @return 返回码。
LW_C_API LWReturnCode LWGetDeviceVersion(LWDeviceHandle handle, LWVersionInfo* fv, LWVersionInfo* dv);

/// @brief 注册网络异常检测的回调函数。当出现SDK与设备的网络连接异常时，该回调函数会被立即调用。
/// @param[in] pCallback 回调函数。当指针为空时即：nullptr，则将移除已注册的回调函数。
/// @param[in] pUserData 用户数据。
/// @return 返回码。
LW_C_API LWReturnCode LWRegisterNetworkMonitoringCallback(void(*pCallback)(LWDeviceHandle handle, const char* error, void* pUserData), void* pUserData);

/// @brief 注册探测新数据可用的回调函数。每当有新数据可用时，该回调函数会被立即调用。
/// @param[in] pCallback 回调函数。当指针为空时即：nullptr，则将移除已注册的回调函数。
/// @param[in] pUserData 用户数据。
/// @return 返回码。
LW_C_API LWReturnCode LWRegisterFrameReadyCallback(void(*pCallback)(LWDeviceHandle handle, void* pUserData), void* pUserData);

/// @brief 获取返回码对应的描述信息。
/// @param[in] code 返回码。
/// @return 返回码描述信息。
LW_C_API const char* LWGetReturnCodeDescriptor(LWReturnCode code);

/// @brief 从设备数据流里截取一帧数据，以供“LWGetFrame”函数的后续处理。
/// @param[in] handle 设备描述符。
/// @return 返回码。
LW_C_API LWReturnCode LWGetFrameReady(LWDeviceHandle handle);

/// @brief 获取指定类型的帧数据。
/// @note 在调用此函数之前必须成功调用“LWGetFrameReady”函数。
/// @param[in] handle 设备描述符。
/// @param[out] frame 指向存储帧数据的指针。
/// @param[in] type 要获取的帧类型。
/// @return 返回码。
LW_C_API LWReturnCode LWGetFrame(LWDeviceHandle handle, LWFrameData* frame, LWFrameType type);

/// @brief 根据IMU的位姿信息进行点云旋转，并输出旋转矩阵。
/// @note “frame”必须是从“LWGetFrame”函数中成功获取的数据帧。
/// @note “rotate_mat”为 3*3 的矩阵。
/// @param[in,out] frame 点云类型的帧数据。
/// @param[out] rotate_mat 旋转矩阵，如果为空指针时则不输出旋转矩阵。
/// @return 返回码。
LW_C_API LWReturnCode LWIMURotatePointCloud(LWFrameData* frame, float* rotate_mat);

/// @brief 将点云数据保存为PCD文件格式。
/// @note “frame”必须是从“LWGetFrame”函数中成功获取的数据帧。
/// @param[in] filename 文件名。比如："D:/data/0001.pcd"。
/// @param[in] frame 点云类型的数据帧。
/// @param[in] binary_mode 数据域的存储格式，值为“true”时，将以二进制的形式存储，反之以ASCII码的形式存储。
/// @return 返回码。
LW_C_API LWReturnCode LWSavePointCloudAsPCDFile(const char* filename, const LWFrameData* frame, bool binary_mode = false);

/// @brief 将点云数据保存为PLY文件格式。
/// @note “frame”必须是从“LWGetFrame”函数中成功获取的数据帧。
/// @param[in] filename 文件名。比如："D:/data/0001.ply"。
/// @param[in] frame 点云类型的数据帧。
/// @param[in] binary_mode 数据域的存储格式，值为“true”时，将以二进制的形式存储，反之以ASCII码的形式存储。
/// @return 返回码。
LW_C_API LWReturnCode LWSavePointCloudAsPLYFile(const char* filename, const LWFrameData* frame, bool binary_mode = false);

/// @brief 将数据帧保存为CSV文件格式。
/// @note “frame”必须是从“LWGetFrame”函数中成功获取的数据帧。
/// @param[in] filename 文件名。比如："D:/mydata.csv"。
/// @param[in] frame 数据帧。
/// @return 返回码。
LW_C_API LWReturnCode LWSaveDataAsCSVFile(const char* filename, const LWFrameData* frame);

/// @brief 将RGB数据保存为图像文件格式。
/// @note “frame”必须是从“LWGetFrame”函数中成功获取的数据帧。
/// @param[in] filename 文件名。比如："D:/data/0001.bmp"。
/// @param[in] frame RGB类型的数据帧。
/// @return 返回码。
LW_C_API LWReturnCode LWSaveRgbAsImageFile(const char* filename, const LWFrameData* frame);

/// @brief 更新设备固件。
/// @param[in] handle 设备描述符。
/// @param[in] filename 更新文件。
/// @return 返回码。
LW_C_API LWReturnCode LWUpdateFirmware(LWDeviceHandle handle, const char* filename);

/// @brief 更新设备固件（不需要打开设备）。
/// @note 在更新成功后，会经过短暂的延时，然后设备将会重新启动；如果重启失败，设备须掉电重启，否则无法进行其它任何操作。
/// @param ip 设备IP。
/// @param filename 更新文件。
/// @return 返回码。
LW_C_API LWReturnCode LWUpdateFirmware1(const char* ip, const char* filename);

/// @brief 设置设备的DO输出值。
/// @param[in] handle 设备描述符。
/// @param[in] index 通道号（从1开始，1表示DO1、2则表示DO2，以此类推）。
/// @param[in] value 只能是0或1（0表示低电平，1表示高电平）。
/// @return 返回码。
LW_C_API LWReturnCode LWSetOutputDO(LWDeviceHandle handle, int32_t channel, int32_t value);

/// @brief 获取已设置的DO输出值。
/// @param[in] handle 设备描述符。
/// @param[in] index 通道号（从1开始，1表示DO1、2则表示DO2，以此类推）。
/// @param[out] value 只能是0或1（0表示低电平，1表示高电平）。
/// @return 返回码。
LW_C_API LWReturnCode LWGetOutputDO(LWDeviceHandle handle, int32_t channel, int32_t* value);


/************************************ 附加模块 --- 托盘识别 ************************************/

/// @brief 用于判定当前设备是否支持托盘识别功能。
/// @param[in] handle 设备描述符。
/// @param[out] value 判定结果的真值。
/// @return 返回码。
LW_C_API LWReturnCode LWHasPalletIdentifyModule(LWDeviceHandle handle, bool* value);

/// @brief 上传托盘识别的模型文件(.rknn格式文件)。
/// @param[in] handle 设备描述符。
/// @param[in] filename 文件全名(含路径)。
/// @return 返回码。
LW_C_API LWReturnCode LWUploadRKNNFile(LWDeviceHandle handle, const char* filename);

/// @brief 设置与托盘识别相关的配置文件(内容为json格式)。
/// @param[in] handle 设备描述符。
/// @param[in] filename 文件全名(含路径)。
/// @return 返回码。
LW_C_API LWReturnCode LWSetPalletConfigureFile(LWDeviceHandle handle, const char* filename);

/// @brief 设置与托盘识别相关的配置信息(内容为json格式)。
/// @param[in] handle 设备描述符。
/// @param[out] buffer 含有配置信息的缓存区。
/// @param[in] bufferLen 缓存区“buffer”的大小（字节数）。
/// @return 返回码。
LW_C_API LWReturnCode LWSetPalletConfigureFileFromBuffer(LWDeviceHandle handle, const char* buffer, int32_t bufferLen);

/// @brief 获取与托盘识别相关的配置信息到指定文件(内容为json格式)。
/// @param[in] handle 设备描述符。
/// @param[in] filename 文件全名(含路径)，文件名自定义，文件格式建议为：.json 或 .txt。
/// @return 返回码。
LW_C_API LWReturnCode LWGetPalletConfigureFile(LWDeviceHandle handle, const char* filename);

/// @brief 获取与托盘识别相关的配置信息到指定的内存缓存区(内容为json格式)。
/// @param[in] handle 设备描述符。
/// @param[out] buffer 足够大的缓存区。
/// @param[in] bufferLen 缓存区“buffer”的大小（字节数）。
/// @return 返回码。
LW_C_API LWReturnCode LWGetPalletConfigureFileToBuffer(LWDeviceHandle handle, char* buffer, int32_t bufferLen);

/// @brief 设置当前识别的托盘类型，其值为配置文件里的第一层（最外层）的键。
/// @note 可通过“LWGetPalletConfigureFile”或“LWGetPalletConfigureFileToBuffer”查看所有的托盘类型。
/// @param[in] handle 设备描述符。
/// @param[in] type 托盘类型(C-字符串)。
/// @return 返回码。
LW_C_API LWReturnCode LWSetPalletIdentifyType(LWDeviceHandle handle, const char* type);

/// @brief 获取当前识别的托盘类型。
/// @param[in] handle 设备描述符。
/// @param[out] buffer 存储托盘类型(C-字符串)的缓存区。
/// @param[in] bufferLen 缓存区“buffer”的大小（字节数）。
/// @return 返回码。
LW_C_API LWReturnCode LWGetPalletIdentifyType(LWDeviceHandle handle, char* buffer, int32_t bufferLen);

/// @brief 设置托盘识别功能的使能开关。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetPalletIdentifyEnable(LWDeviceHandle handle, bool enable);

/// @brief 获取托盘识别功能的使能开关。
/// @param[in] handle 设备描述符。
/// @param[out] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWGetPalletIdentifyEnable(LWDeviceHandle handle, bool* enable);

/// @brief 设置托盘识别的匹配相似度阈值(取值范围：0~1，默认值：0.65)。
/// @param[in] handle 设备描述符。
/// @param[in] value 阈值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTRSDsimilarMax(LWDeviceHandle handle, float value);

/// @brief 获取托盘识别的匹配相似度阈值。
/// @param[in] handle 设备描述符。
/// @param[out] value 阈值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTRSDsimilarMax(LWDeviceHandle handle, float* value);

/// @brief 设置托盘识别的匹配重合度阈值(取值范围：0~1，默认值：0.8)。
/// @param[in] handle 设备描述符。
/// @param[in] value 阈值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTRSDpstMax(LWDeviceHandle handle, float value);

/// @brief 获取托盘识别的匹配重合度阈值。
/// @param[in] handle 设备描述符。
/// @param[out] value 阈值。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTRSDpstMax(LWDeviceHandle handle, float* value);

/// @brief 设置托盘识别的截断高度(取值范围：1~200，默认值：70)。
/// @param[in] handle 设备描述符。
/// @param[in] value 截断高(度单位：mm)。
/// @return 返回码。
LW_C_API LWReturnCode LWSetCutHeight(LWDeviceHandle handle, int32_t value);

/// @brief 获取托盘识别的截断高度。
/// @param[in] handle 设备描述符。
/// @param[out] value 截断高度(单位：mm)。
/// @return 返回码。
LW_C_API LWReturnCode LWGetCutHeight(LWDeviceHandle handle, int32_t* value);


#ifdef LW_INTERNAL_API

/// @brief 发送文件。
/// @param[in] handle 设备描述符。
/// @param[in] fullname 文件全名(含路径)。
/// @param[in] type 文件发送类型。
/// @return 返回码。
LW_C_API LWReturnCode LWSendFile(LWDeviceHandle handle, const char* fullname, LWFileType type);

/// @brief 设置设备的SN号。
/// @param[in] handle 设备描述符。
/// @param[in] buffer 含有SN信息的缓存区。
/// @param[in] bufferLen 缓存区“buffer”的大小（字节数）。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDeviceSN(LWDeviceHandle handle, const char* buffer, int bufferLen);

/// @brief 发送要在下位机中执行的命令行字符串。
/// @param[in] handle 设备描述符。
/// @param[in] comstr 命令行字符串(例如：'rm /etc/test.txt')。
/// @param[in] size 命令行字符串大小。
/// @return 返回码。
LW_C_API LWReturnCode LWSendOperateCommand(LWDeviceHandle handle, const char* comstr, int size);

/// @brief 设置DRNU使能开关。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDRNU(LWDeviceHandle handle, bool enable);

/// @brief 设置设备的Binning模式。
/// @param[in] handle 设备描述符。
/// @param[in] mode Binning模式。
/// @return 返回码。
LW_C_API LWReturnCode LWSetBinningMode(LWDeviceHandle handle, LWBinningMode mode);

/// @brief 设置传感器的分辨率。
/// @note 目前仅支持TOF传感器，可设置组合为：320*240、640*480。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型。
/// @param[in] width 图像宽度即水平分辨率。
/// @param[in] height 图像高度即垂直分辨率。
/// @return 返回码。
LW_C_API LWReturnCode LWSetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t width, int32_t height);

/// @brief 设置畸变校准使能开关。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDistortionCalibration(LWDeviceHandle handle, LWSensorType sensorType, bool enable);

/// @brief 设置工作频率。
/// @param[in] handle 设备描述符。
/// @param[in] array 指向数组指针。
/// @param[in] size 数组“array”的大小。
/// @return 返回码。
LW_C_API LWReturnCode LWSetLaserWorkFrequency(LWDeviceHandle handle, const uint8_t* array, int size);

/// @brief 设置自动曝光时间的起始默认值。
/// @param[in] handle 设备描述符。
/// @param[in] value 默认值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetAutoExposureDefaultValue(LWDeviceHandle handle, uint16_t value);

/// @brief 设置传感器的内参。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型。
/// @param[in] param 内参。
/// @return 返回码。
LW_C_API LWReturnCode LWSetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam param);

/// @brief 设置传感器的外参。
/// @param[in] handle 设备描述符。
/// @param[in] sensorType 传感器类型。
/// @param[in] param 外参。
/// @return 返回码。
LW_C_API LWReturnCode LWSetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam param);

/// @brief 设置IMU传感器的外参。
/// @param[in] handle 设备描述符。
/// @param[in] param 外参。
/// @return 返回码。
LW_C_API LWReturnCode LWSetIMUExtrinsicParam(LWDeviceHandle handle, LWIMUExtrinsicParam param);

/// @brief 设置温度补偿使能开关。
/// @param[in] handle 设备描述符。
/// @param[in] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTemperatureCompensation(LWDeviceHandle handle, bool enable);

/// @brief 获取温度补偿的使能开关。
/// @param[in] handle 设备描述符。
/// @param[out] enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWGetTemperatureCompensation(LWDeviceHandle handle, bool* enable);

/// @brief 设置TOF传感器的温度参数。
/// @param[in] handle 设备描述符。
/// @param[in] param 参数值。
/// @return 返回码。
LW_C_API LWReturnCode LWSetTemperatureParams(LWDeviceHandle handle, LWTemperatureParams param);

/// @brief 设置TOF相机的每个激光器的使能开关。
/// @param[in] handle 设备描述符。
/// @param[in] flag 使能开关（每个比特位代表一个激光器的使能开关，值为1表示打开，0则表示关闭；比特位0对应激光器1，比特位1对应激光器2，以此类推）。
/// @return 返回码。
LW_C_API LWReturnCode LWSetLaserEnableStatus(LWDeviceHandle handle, uint32_t flag);

/// @brief 获取TOF相机的每个激光器的使能开关。
/// @param[in] handle 设备描述符。
/// @param[out] flag 使能开关（每个比特位代表一个激光器的使能开关，值为1表示打开，0则表示关闭；比特位0对应激光器1，比特位1对应激光器2，以此类推）。
/// @return 返回码。
LW_C_API LWReturnCode LWGetLaserEnableStatus(LWDeviceHandle handle, uint32_t* flag);

/// @brief 设置TOF与RGB数据对齐的使能开关。
/// @param handle 设备描述符。
/// @param enable 使能开关。
/// @return 返回码。
LW_C_API LWReturnCode LWSetDataAlignEnable(LWDeviceHandle handle, bool enable);

#endif //LW_INTERNAL_API


#ifdef LW_ZHONGRILONG_PROJ

LW_C_API LWReturnCode LWSynchronizeDeviceSystemTime(LWDeviceHandle handle);
LW_C_API LWReturnCode LWHasSecurityAbility(LWDeviceHandle handle, bool *enable);
LW_C_API LWReturnCode LWGetSecurityEnable(LWDeviceHandle handle, bool* enable);
LW_C_API LWReturnCode LWSetSecurityCalibrationParams(LWDeviceHandle handle, const int32_t* _array, int32_t size);
LW_C_API LWReturnCode LWSecurityCancelCalibration(LWDeviceHandle handle);
LW_C_API LWReturnCode LWGetSecurityCalibrationEnable(LWDeviceHandle handle, bool* enable);
LW_C_API LWReturnCode LWSetSecurityAxialAdjustment(LWDeviceHandle handle, int flag, float value);
LW_C_API LWReturnCode LWResetSecurityConfigure(LWDeviceHandle handle);
// 3.0
LW_C_API LWReturnCode LWGetSecurityCalibrationMatrixParams(LWDeviceHandle handle, float* mat, int32_t* size);
LW_C_API LWReturnCode LWGetSecurityZHSAEnable(LWDeviceHandle handle, bool* enable);
LW_C_API LWReturnCode LWSetSecurityZHSAEnable(LWDeviceHandle handle, bool enable);

LW_C_API LWReturnCode LWGetSecurityConfigFile(LWDeviceHandle handle, int type, const char* filename);
LW_C_API LWReturnCode LWSetSecurityConfigFile(LWDeviceHandle handle, int type, const char* filename);
LW_C_API LWReturnCode LWGetSecurityConfigFileToBuffer(LWDeviceHandle handle, int type, char* buffer, int32_t bufLen);
LW_C_API LWReturnCode LWSetSecurityConfigFileFromBuffer(LWDeviceHandle handle, int type, const char* buffer, int32_t bufLen);


#endif //LW_ZHONGRILONG_PROJ


#endif //LW_DM_API_H

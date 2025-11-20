//
// Created by ljh on 2024/8/15.
//

/**@file LWDMType.h
* 
* @brief Definition of 3D camera data types and other data types.
* 
* @copyright  Copyright(C)2024-2026 Percipio All Rights Reserved.
* 
**/

#ifndef LW_DM_C_TYPE_H
#define LW_DM_C_TYPE_H


#if defined(_WIN32)
    #ifdef LW_INTERFACE_EXPORT
        #define LW_API __declspec(dllexport)
    #else
        #define LW_API __declspec(dllimport)
    #endif
#else
    #define LW_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
    #define LW_C_API extern "C" LW_API
#else
    #define LW_C_API LW_API
#endif


#include <cstdint>


//******************************************* 内部专用 *******************************************************************//
#define LW_INTERNAL_API
#ifdef  LW_INTERNAL_API

/// @brief 文件下发类型。
enum LWFileType : uint32_t
{
    LW_DRNU_HIGH            = 0x00, ///< 高频DRNU标定文件。
    LW_DRNU_LOW             = 0x01, ///< 低频DRNU标定文件。
    LW_BENDING_LUT          = 0x02, ///< 弯曲标定文件。
    LW_DISTORTION_LUT       = 0x03, ///< 畸变标定文件。
    LW_OFFSET_COMP          = 0x04, ///< 平整度校准文件。
    LW_PALLET_ARG           = 0x05, ///< 托盘参数文件。

    LW_SECURITY_ARG         = 0x06, ///< 安全防护区域参数文件。
    LW_SECURITY_PR_ARG      = 0x07, ///< 安全防护透视区域参数文件。
    LW_SECURITY_PR_MAT_ARG  = 0x08, ///< 安全防护透视矩阵参数文件。
    LW_SECURITY_CALIB       = 0x09, ///< 安全防护标定文件。

    LW_SECURITY_LOG         = 0xA1, ///< 安全防护日志文件。
    
    LW_OTHER                = 0xf0  ///< 其它自定义文件。
};

/// @brief Binning模式。
enum LWBinningMode : uint32_t
{
    LW_1X1BINNING = 0x00,
    LW_2X2BINNING = 0x01,
    LW_4X4BINNING = 0x02
};

/// @brief 温度标定参数。
struct LWTemperatureParams
{
    double coef1 = 0;
    double coef2 = 0;
    double temp1 = 0;
    double temp2 = 0;
};
#endif //LW_INTERNAL_API


/// @brief 设备句柄描述符，前四个字节为远端IPv4地址后四个字节为本机IPv4地址。
typedef uint64_t LWDeviceHandle;

///@brief 函数返回码，用于告知函数的执行结果。
typedef enum : uint32_t
{
    LW_RETURN_OK                    = 0x00, ///< 执行成功。
    LW_RETURN_COMMAND_UNDEFINED     = 0x03,	///< 命令未定义。
    LW_RETURN_COMMAND_ERROR         = 0x04,	///< 命令结构错误。
    LW_RETURN_ARG_OUT_OF_RANGE      = 0x05,	///< 参数设置超范围。
    LW_RETURN_FILE_LENGTH_ERROR     = 0x06,	///< 文件大小与实际传输大小不一致。
    LW_RETURN_FILE_MD5_ERROR        = 0x07,	///< 文件MD5校验失败。

    LW_RETURN_ACTION_INVALID        = 0x08,	///< 动作无效，已标定，请先取消标定再重新标定。
    LW_RETURN_REGION_INVALID        = 0x0b, ///< 安防标定区域无效，其区域内有效点云数低于阈值，请重新选择更合适的区域。
    LW_RETURN_AJSON_FORMAT_ERROR    = 0x0c, ///< 安防Json配置文件格式错误。
    LW_RETURN_AJSON_KEY_ERROR       = 0x0d, ///< 安防Json配置文件键值类型/个数不符合。
    LW_RETURN_AJSON_VALUE_ERROR     = 0x0e, ///< 安防Json配置文件键值超范围。
    LW_RETURN_AJSON_LOSS_ERROR      = 0x0f, ///< 安防Json配置文件键值缺失。
    LW_RETURN_AJSON_PORJ_ERROR      = 0x10, ///< 安防Json配置文件项目名错误。
    LW_RETURN_AJSON_VERSION_ERROR   = 0x11, ///< 安防Json配置文件版本号与固件支持版本不匹配。
    LW_RETURN_AJSON_KEY_NAME_ERROR  = 0x12, ///< 安防Json配置文件键名错误。

    LW_RETURN_TIMEOUT               = 0x20,	///< 执行超时。
    LW_RETURN_NETWORK_ERROR         = 0x21,	///< 网络错误，欲知详情请调用“LWGetReturnCodeDescriptor”函数。
    LW_RETURN_UNINITIALIZED         = 0x22, ///< SDK还未进行资源初始化（须调用“LWInitializeResources”函数来初始化资源）。
    LW_RETURN_UNOPENED              = 0x23, ///< 设备未打开（须调用“LWOpenDevice”函数来打开设备）。
    LW_RETURN_HANDLE_MISMATCH       = 0x24, ///< 传入的设备句柄无效，请检查该句柄是否是“LWFindDevices”函数调用返回的设备句柄。
    LW_RETURN_FILE_OPEN_ERROR       = 0x25, ///< 文件打开失败。
    LW_RETURN_NOT_SUPPORTED         = 0x26, ///< 当前设备尚不支持该功能。
    LW_RETURN_VERSION_ERROR         = 0x27, ///< 协议版本不匹配。
    LW_RETURN_OUT_OF_MEMORY         = 0x28, ///< 传入的数据缓存大小不足。
    LW_RETURN_TYPE_NOT_EXIST        = 0x29, ///< 类型错误，不存在该类型或是不支持该类型。
    LW_RETURN_TYPE_INPUT_ERROR      = 0x2a, ///< 数据类型错误，请传入正确类型的数据（例如：“LWSavePointCloudAsPCDFile”函数只能传入点云数据）。
    LW_RETURN_THREAD_QUIT_TIMEOUT   = 0x2b, ///< 线程退出超时。
    LW_RETURN_DATA_TYPE_MISMATCH    = 0x2c, ///< 无法获取该类型数据，请设置正确的数据接受类型。
    LW_RETURN_DATA_NOT_UPDATED      = 0x2d, ///< 数据接受缓存区未更新数据，在获取数据之前请先成功调用“LWGetFrameReady”函数。
    LW_RETURN_FILE_NOT_EXIST        = 0x2e, ///< 文件不存在。 
    LW_RETURN_DATA_SIZE_ERROR       = 0x2f, ///< 传入的数据大小不匹配。

    LW_RETURN_FIRMWARE_UPDATE_FAIL  = 0x30, ///< 设备固件更新失败。
    LW_RETURN_INDEX_NOT_EXIST       = 0x31, ///< 不存在该索引值，请传入正确的索引值。

    LW_RETURN_CUSTOM_ERROR          = 0xfa, ///< 自定义错误，欲知详情请调用“LWGetReturnCodeDescriptor”函数。

    LW_RETURN_UNDEFINED_ERROR       = 0xff, ///< 未定义错误。

} LWReturnCode;

/// @brief 数据帧类型。
typedef enum : uint32_t
{
    LW_TYPE_UNDEFINED       = 0B000000000,   ///< 类型未定义。

    LW_DEPTH_FRAME          = 0B000000001,   ///< 深度数据类型，单位：毫米。
    LW_AMPLITUDE_FRAME      = 0B000000010,   ///< 幅度数据类型。
    LW_IR_FRAME             = 0B000000100,   ///< IR数据类型。
    LW_POINTCLOUD_FRAME     = 0B000001000,   ///< 点云数据类型，z-轴数据对应深度值。
    LW_RGB_FRAME            = 0B000010000,   ///< RGB数据类型。
    LW_RGB_TO_DEPTH_FRAME   = 0B000100000,   ///< RGB数据映射到深度后的RGB数据类型，其分辨率对齐到深度数据（与深度数据的分辨率一致）。
    LW_DEPTH_TO_RGB_FRAME   = 0B001000000,   ///< 深度数据映射到RGB后的深度数据类型，其分辨率对齐到RGB数据（与RGB数据的分辨率一致）。
    LW_IR_TO_RGB_FRAME      = 0B100000000,   ///< IR数据映射到RGB后的IR数据类型，其分辨率对齐到RGB数据（与RGB数据的分辨率一致）。
    LW_D2R_POINTCLOUD_FRAME = 0B010000000,   ///< 深度数据映射到RGB后的点云数据类型，z-轴数据对应深度值，其点数对齐到RGB像素数（与RGB数据的像素数一致）。

} LWFrameType;

/// @brief 数据帧的像素格式，用以正确的解析数据帧。
typedef enum : uint32_t
{
    LW_PIXEL_FORMAT_UCHAR       = 0x00,	///< 每像素为无符号字符型（unsigned char）。
    LW_PIXEL_FORMAT_USHORT      = 0x01,	///< 每像素为无符号短整型（unsigned short）。
    LW_PIXEL_FORMAT_RGB888      = 0x02,	///< 每像素为三通道无符号字符型(详见：“LWRGB888Pixel”结构体)。
    LW_PIXEL_FORMAT_VECTOR3F    = 0x03,	///< 每像素为三通道浮点型(详见：“LWVector3f”结构体)。

} LWPixelFormat;

/// @brief 用于设置从设备接收的数据类型。
typedef enum : uint32_t
{
    LW_IR_RTY                   = 0x04, ///< 设备只发送“IR”数据。
    LW_DEPTH_AMPLITUDE_RGB_RTY  = 0x05, ///< 设备只发送“深度+幅度+RGB”数据。由于点云数据是根据深度数据进行转换的，因此亦可获取点云数据。
    LW_RGB_RTY                  = 0x06, ///< 设备只发送“RGB”数据。
    LW_DEPTH_IR_RTY             = 0x07, ///< 设备只发送“深度+IR”数据。由于点云数据是根据深度数据进行转换的，因此亦可获取点云数据。
    LW_POINTCLOUD_IR_RTY        = 0x08, ///< 设备只发送“点云+IR”数据。
    LW_DEPTH_RGB_RTY            = 0x09, ///< 设备只发送“深度+RGB”数据。由于点云数据是根据深度数据进行转换的，因此亦可获取点云数据。

    LW_DEPTH_IR_RGB_RTY         = 0x0C, ///< 设备只发送“深度+IR+RGB”数据。由于点云数据是根据深度数据进行转换的，因此亦可获取点云数据。

    LW_POINTCLOUD_DEPTH_IR_RTY  = 0x0B  ///< 设备只发送“点云+深度+IR”数据。特定版本---中日龙安防项目。

} LWDataRecvType;

/// @brief 传感器类型。
typedef enum : uint32_t
{
    LW_TOF_SENSOR = 0x01,	///< TOF传感器。
    LW_RGB_SENSOR = 0x02,	///< RGB传感器。

} LWSensorType;

/// @brief 曝光模式。
typedef enum : uint32_t
{
    LW_EXPOSURE_AUTO    = 0x00, ///< 自动曝光，由设备根据外部环境自行调整曝光时间。
    LW_EXPOSURE_MANUAL  = 0x01, ///< 手动曝光，以设定的曝光时间为准，不再进行自动调整。

} LWExposureMode;

/// @brief TOF传感器的HDR模式枚举。
typedef enum : uint32_t
{
    LW_DFN_NOT_HDR  = 0x00, ///< 适用于远距离且没有高动态范围的应用场景，采用双频非HDR模式（双频2积分模式）。需设置1个曝光时间（例如：1000），否则会使用默认的曝光时间。其帧率最高可达28帧。
    LW_SFN_HDR      = 0x01, ///< 适用于近距离且具有高动态范围的应用场景，采用单频普通HDR模式（双频3积分模式）。需设置3个曝光时间依次对应高、中、低3个挡位（例如：1000，150，20），否则会使用默认的曝光时间。其帧率最高可达18帧。
    LW_DFN_HDR      = 0x02, ///< 适用于远距离且具有高动态范围的应用场景，采用双频普通HDR模式（双频6积分模式）。需设置3个曝光时间依次对应高、中、低3个挡位（例如：1000，150，20），否则会使用默认的曝光时间。其帧率最高可达9帧。
    LW_SFN_HP_HDR   = 0x03, ///< 适用于近距离且具有高动态范围的静态应用场景，采用单频高精度HDR模式（双频 6积分模式），可提升距离探测精度。需设置3个曝光时间依次对应高、中、低3个挡位（例如：1000，150，20），否则会使用默认的曝光时间。其帧率最高可达9帧。
    LW_DFN_HP_HDR   = 0x04, ///< 适用于远距离且具有高动态范围的静态应用场景，采用双频高精度HDR模式（双频12积分模式），可提升距离探测精度。需设置3个曝光时间依次对应高、中、低3个挡位（例如：1000，150，20），否则会使用默认的曝光时间。其帧率最高可达5帧。
    LW_SFN_NOT_HDR  = 0xF1, ///< 适用于近距离且没有高动态范围的应用场景，采用单频非HDR模式（单积分模式）。其帧率最高可达56帧。

} LWHDRMode;

/// @brief TOF传感器工作的频率模式。
typedef enum : uint32_t
{
    LW_FREQUENCY_DUAL   = 0x00, ///< 双频模式，用于远距离测距。
    LW_FREQUENCY_SINGLE = 0x01,	///< 单频模式，用于近距离测距。

} LWFrequencyMode;

/// @brief 设备的触发模式（工作模式），须在开启数据流之前进行设置。
typedef enum : uint32_t
{
    LW_TRIGGER_ACTIVE       = 0x00,	///< 主动模式（连续触发）。当开启数据流时，设备会按照指定帧率发送数据。
    LW_TRIGGER_SOFT         = 0x01,	///< 软触发模式。当开启数据流时，每调用一次“LWSoftTrigger”函数设备便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。
    LW_TRIGGER_HARD         = 0x02,	///< 硬触发模式。当开启数据流时，设备每检测到一次外部信号（电平信号）便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。
    LW_TRIGGER_HARD_FILTER  = 0x03,	///< 带滤波参数的硬触发模式（信号的持续时间和间隔）。根据设定的信号滤波参数，设备每检测到一次外部信号（电平信号）便会发送一帧数据。建议将帧率设置为最大帧率，至少不得低于触发的频率。

} LWTriggerMode;

/// @brief RGB传感器可发送的数据格式。
/// @note 千兆网的实际传输不可能稳定地达到125MB/s（理论值），因为受限于网络协议开销、TCP协议本身的机制、系统与硬件性能、网络环境与干扰等因素。实际传输一般在110MB/s左右。
typedef enum : uint32_t
{
    LW_MJPEG        = 0x00, ///< JPG格式，由于是有损压缩，它所占用的网络带宽很小，对设备帧率没影响。
    LW_YUV422_YUYV  = 0x01, ///< YUV格式，由于是无损压缩，它虽能完好的还原出原图像，但占用的网络带宽很大（相比JPG格式），其最大传输帧率在28左右（基于1000兆网），且对设备的帧率影响很大。
    LW_YVU420_NV12  = 0x02, ///< YUV格式，由于是无损压缩，它虽能完好的还原出原图像，但占用的网络带宽很大（相比JPG格式），其最大传输帧率在38左右（基于1000兆网），且对设备的帧率影响很大。
    LW_BGR_888      = 0x03, ///< RGB格式，由于是无损压缩，但占用的网络带宽最大（相比JPG格式），其最大传输帧率在22左右（基于1000兆网），对设备的帧率影响最大。
    LW_BGR_565      = 0x04, ///< RGB格式，由于是有损压缩，占用的网络带宽相对较少（相比BGR888格式），其最大传输帧率在33左右（基于1000兆网），对设备的帧率影响相对较小。

} LWRgbTransferFormat;

/// @brief 通用滤波参数结构。
typedef struct
{
    bool	enable;		///< 使能开关。
    int32_t	threshold;	///< 滤波阈值。
    int32_t	k1;	        ///< 扩展位，仅对部分滤波有效。
    int32_t	k2;	        ///< 扩展位，仅对部分滤波有效。

} LWFilterParam;

/// @brief 网络配置信息。
typedef struct
{
    char	type;			///< IPv4地址的类型, 0x00表示为动态IP地址, 0x01表示为静态IP地址。
    char    ip[32];			///< IPv4地址。
    char    netmask[32];	///< 子网掩码。
    char    gateway[32];	///< 网关（保留字段，无须关注）。
    char    mac[32];		///< MAC地址（保留字段，无须关注）。
    char    reserved[96];	///< 保留字段。

} LWNetworkInfo;

/// @brief 设备搜索信息。
typedef struct
{
    LWDeviceHandle  handle;         ///< 设备句柄描述符。
    char            sn[32];         ///< 设备SN。
    char            type[32];       ///< 设备类型。
    char            ip[32];         ///< 设备IP。
    char            local_ip[32];   ///< 本机IP。

} LWDeviceInfo;

/// @brief 版本号信息。
typedef struct
{
    int32_t major;      ///< 主版本号。
    int32_t minor;      ///< 次版本号。
    int32_t patch;      ///< 修订版本号。
    int32_t reserved;   ///< 编译版本号。

} LWVersionInfo;

/// @brief 温度结构信息。
typedef struct
{
    float	laser1;	///< 激光器1的温度。
    float	laser2;	///< 激光器2的温度。
    float	chip;	///< 芯片的温度。

} LWTemperature;

/// @brief 时间戳结构信息。
typedef struct
{
    int64_t tv_sec;		///< 秒数。
    int64_t tv_usec;	///< 微秒数。

} LWTimeStamp;

/// @brief 相机内参结构信息。
typedef struct
{
    float fx;	///< x轴向上的焦距长度，它是相机焦距f在x方向上的像素表示，以像素为单位。
    float fy;	///< y轴向上的焦距长度，它是相机焦距f在y方向上的像素表示，以像素为单位。
    float cx;	///< 相机光轴（也称为主点或光心点）在图像坐标系中x轴上的偏移量，以像素为单位。
    float cy;	///< 相机光轴（也称为主点或光心点）在图像坐标系中y轴上的偏移量，以像素为单位。
    float k1;	///< 径向畸变系数1，主要的径向畸变系数。
    float k2;	///< 径向畸变系数2，更高阶的畸变系数，用于更精确地描述畸变情况。
    float k3;	///< 径向畸变系数3，更高阶的畸变系数，用于更精确地描述畸变情况。
    float p1;	///< 切向畸变系数1，用于描述由于镜头装配不完全对称导致的图像畸变。切向畸变表现为图像边缘的偏移。
    float p2;	///< 切向畸变系数2，用于描述由于镜头装配不完全对称导致的图像畸变。切向畸变表现为图像边缘的偏移。

} LWSensorIntrinsicParam;

/// @brief 相机外参结构信息。
typedef struct
{
    float rotation[3][3];	///< 旋转矩阵。
    float translation[3];	///< 平移矩阵。

} LWSensorExtrinsicParam;

/// @brief IMU的旋转矩阵
typedef struct 
{
    float rotation[3][3];	///< 旋转矩阵。

} LWIMUExtrinsicParam;

/// @brief IMU位姿结构信息
typedef struct
{
    float xAxisACC;	    ///< x-轴向上的加速度。
    float yAxisACC;	    ///< y-轴向上的加速度。
    float zAxisACC;	    ///< z-轴向上的加速度。
    float xAxisGyro;    ///< （陀螺仪）x-轴向上的角速度。
    float yAxisGyro;    ///< （陀螺仪）y-轴向上的角速度。
    float zAxisGyro;    ///< （陀螺仪）z-轴向上的角速度。
    float yawAngle;     ///< 偏航角解算值。
    float rollAngle;    ///< 翻滚角解算值。
    float pitchAngle;   ///< 俯仰角解算值。

} LWIMUData;

/// @brief 识别出的托盘区域信息。
typedef struct
{
    int32_t x;      ///< 识别的托盘左上角x值
    int32_t y;      ///< 识别的托盘左上角y值
    int32_t w;      ///< 识别的托盘宽度
    int32_t h;      ///< 识别的托盘高度
    float   conf;   ///< 识别的托盘置信度

} LWPalletAreaData;

/// @brief 识别出的托盘位姿信息。
typedef struct
{
    float               x;      ///< 托盘中心点的x值
    float               y;      ///< 托盘中心点的y值
    float               z;      ///< 托盘中心点的z值
    float               rx;     ///< yoz面绕x轴偏转角(yaw)
    float               ry;     ///< xoz面绕y轴偏转角(pitch)
    float               rz;     ///< xoy面绕z轴偏转角(roll)
    float               cx;     ///< IR图像上托盘中心像素的x坐标
    float               cy;     ///< IR图像上托盘中心像素的y坐标
    LWPalletAreaData    box;    ///< 托盘区域

} LWPalletPoseData;

/// @brief 点云的点信息，单位：毫米。
typedef struct
{
    float x;    ///< 点在三维空间坐标系里的x坐标值。
    float y;    ///< 点在三维空间坐标系里的y坐标值。
    float z;    ///< 点在三维空间坐标系里的z坐标值（相对于镜头平面的垂直距离即深度值）。

} LWVector3f;

/// @brief RGB图像的像素信息
typedef struct
{
    uint8_t r;	///< 红色分量值。
    uint8_t g;	///< 绿色分量值。
    uint8_t b;	///< 蓝色分量值。

} LWRGB888Pixel;

/// @brief 为了保持"LWFrameData"结构体一致性的情况下能够进行相应扩展，于是新增该“变体”结构体，后续版本的新增数据结构（与帧数据息息相关）均在此结构体里进行添加。
/// 注：仅添加与帧数据相关联的数据结构。
typedef struct
{
    struct IMUStruct
    {
        uint32_t		    calMode;        ///< 用于点云旋转。仅限内部使用。
        LWIMUExtrinsicParam eParam;         ///< 用于点云旋转。仅限内部使用。
        LWIMUData           imuData;        ///< IMU位姿数据。

    } IMUModule;

    struct PalletStruct
    {
        bool                identifyEnable; ///< 识别使能开关。
        int32_t             identifyNumber; ///< 识别出的托盘数量。

        enum ErrorCode : int32_t
        {
            LW_NO_ERROR             =  1,   ///< 识别成功。
            LW_NOT_FOUND_ARG_SET    = -1,   ///< 没有找到“托盘算法参数集合名”。
            LW_NOT_FOUND_MODEL      = -2,   ///< 没有找到“托盘算法模型”。
            LW_NOT_FOUND_ARG_FILE   = -3,   ///< 没有找到“托盘算法参数集合名”对应的Json文件。
            LW_BUSY                 = -4,   ///< 算法正在运行中,请稍后再试。
            LW_TIMEOUT              = -5,   ///< 超时。
            LW_UNINITIALIZED        = -6,   ///< 未完成初始化。
            LW_ARG_MISMATCH         = -7,   ///< 参数不匹配。
            LW_ARG_REWORK           = -8,   ///< 参数被修改,当前识别结果不输出。
            LW_UNRECOGNIZED         = -9,   ///< 模型未识别到托盘。

        } errorCode;    ///< 错误码。

        LWPalletPoseData    poseData[20];   ///< 识别出的托盘位姿数据。

    } PalletModule;

#ifdef LW_ZHONGRILONG_PROJ
    /// @brief 中日龙项目
    struct ZRLSafetyStruct
    {
        bool        enable;                 ///< 安全防护使能开关。
        uint8_t     error;                  ///< 仅供内部人员排查问题，用户可忽略此项。
        uint8_t     areaAttribute[9];       ///< 区域属性(0：正常，1：报警，2：预警)。
        uint16_t    alarmAreaNumber[9];     ///< 警报区域编号。
        uint32_t    pots[8];                ///< 区域点数。
        float       pyramid[15];            ///< 视锥。
        float       detectionArea[80];      ///< 3D检测区域。

    } SecurityDetection;
#endif //LW_ZHONGRILONG_PROJ

} LWVariant;

/// @brief 数据帧结构信息。
typedef struct
{
    uint16_t		width;			///< 帧数据的宽度（列数），由于数据是基于像素的，因此也可以看作是水平分辨率。
    uint16_t		height;			///< 帧数据的高度（行数），由于数据是基于像素的，因此也可以看作是垂直分辨率。
    uint32_t		frameIndex;		///< 帧数据的编号，它是从1开始递增的，每次新开启数据流时都会对其进行重置。
    uint32_t		bufferSize;		///< 帧数据域的大小（字节数），即“pFrameData”指针指向的内存区域大小。
    uint32_t		elemSize;		///< 帧数据里一个像素所占用的字节数，例如：深度数据是用“unsigned short”表示的，因此该值是2，对于点云数据则是12。
    uint32_t		total;		    ///< 帧数据总的像素数，即数据宽度与高度的乘积（width * height）。
    LWFrameType		frameType;		///< 帧数据的类型，详见“LWFrameType”所枚举的类型。
    LWPixelFormat	pixelFormat;	///< 帧数据的像素类型，详见“LWPixelFormat”所枚举的类型。
    LWTemperature   temperature;    ///< 设备采集该帧数据时，设备的温度信息（仅TOF数据附带温度信息，如果是RGB数据则请忽略）。
    LWTimeStamp		timestamp;		///< 帧数据的时间戳。
    char*           pFrameData;		///< 指向帧数据域的指针。注：在使用时请先转换为对应的数据类型，比如使用点云数据时则需进行类型转换---> auto ptr = (LWVector3f*)frameData.pFrameData。

    LWVariant*      pVariant;       ///< 指向“LWVariant”数据结构的指针。

} LWFrameData;


#endif //LW_DM_C_TYPE_H

//
// Created by ljh on 2024/8/15.
//
#include "LWDMApi.h"
#include "UpdateToolTask.h"

#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <map>
#include <unordered_set>
#include <Eigen/Dense>
#include <openssl/evp.h>
#include <unsupported/Eigen/AutoDiff>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#if defined(_WIN32)
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define MSG_NOSIGNAL (0)
#define PATH_SEPARATOR '\\'
#elif defined(__linux__)
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <sys/stat.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <fcntl.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define PATH_SEPARATOR '/'
#endif


///< 循环队列最大节点数
#define LOOP_QUEUE_SIZE    4
///< 控制端口的端口号
#define COMMAND_PORT 50660
///< 数据端口的端口号
#define DATA_PORT 50661
///< 控制端口的数据接收缓冲区大小
#define COMMAND_MAX_SIZE 1024
///< 命令控制协议版本
#define PROTOCOL_VERSION 0x03
///< 数据映射线程数
#define THREAD_POOL_SIZE 4
///< RGBD投影冗余区域数据过滤阈值[mm]
#define RGBD_PROJ_REDUNDANCY 25
///< RGB传感器的垂直方向像素数。
#define RGB_MAX_PIX_ROWS 1200
///< RGB传感器的水平方向像素数。
#define RGB_MAX_PIX_COLS 1600
///< RGB传感器总像素数。
#define RGB_MAX_PIX_NUMBER  RGB_MAX_PIX_ROWS*RGB_MAX_PIX_COLS
///< 三维点云z坐标的最大值。
#define PCD_MAX_VALUE 60000
///< TOF传感器的垂直方向像素数。
#define TOF_MAX_PIX_ROWS 480
///< TOF传感器的水平方向像素数。
#define TOF_MAX_PIX_COLS 640
///< TOF传感器总像素数。
#define TOF_MAX_PIX_NUMBER  TOF_MAX_PIX_ROWS*TOF_MAX_PIX_COLS
///< Π的赋值。
#define PI 3.14159265359
// 日志文件的最大尺寸（字节）
#define LOG_FILE_MAX_SAVE_SIZE 100 * 1024 * 1024
///< 指令集
#define C_Start							0x0000
#define C_Stop							0x0001
#define C_SoftTrigger					0x0002
#define C_Discovery						0x0003
#define C_SaveConfig					0x0004
#define C_DeleteConfig					0x0005
#define C_RecoveryDefaultConfigure		0x0006
#define C_DropTest						0x00fc
#define C_StartEntrucking				0x00fd
#define C_StopEntrucking				0x00fe
#define C_DeviceReboot					0x00ff
#define C_SetDataAddress				0x0100
#define C_SetTriggerMode				0x0101
#define C_SetHDR						0x0102
#define C_SetFrameRate					0x0103
#define C_SetIntegralTime				0x0104
#define C_SetTimeFilter					0x0105
#define C_SetFlyPixeFilter				0x0106
#define C_SetConfidenceFilter			0x0107
#define C_SetKalmanFilter				0x0108
#define C_SetSpatialFilter				0x0109
#define C_SetNetworkModel				0x010a
#define C_HeartbeatAddress				0x010b
#define C_SetRecvDataType				0x010c
#define C_SetRecvDataRows				0x010d
#define C_SetTimeMedianFilter			0x010e
#define C_SetIRGMMGain					0x0110
#define C_SetExposureValue				0x0111
#define C_SetMeanFilter					0x0112
#define C_SetFrequencyModel				0x0113
#define C_SetOutputDO					0x0114
#define C_SetSingleIntegral				0x011f
#define C_SetBinning					0x0120
#define C_SetTempCompensate				0x0121
#define C_SetCameraIntrinsicArg			0x0122
#define C_SetSN							0x0123
#define C_SetDRNU						0x0124
#define C_SetLaserWorkFrequency			0x0125
#define C_SetDistortion					0x0126
#define C_SetTempCoefficient			0x0127
#define C_SetDelayHardTriggerTime		0x0128
#define C_SetRgbCameraExtrinsicArg		0x0129
#define C_SetRgbCameraIntrinsicArg		0x012a
#define C_SetRgbCameraExposureMode		0x012b
#define C_SetRgbCameraExposureTime		0x012c
#define C_SetRgbCameraBrightness		0x012d
#define C_SetRgbCameraContrastRatio		0x012e
#define C_SetRgbCameraResolutionRatio	0x012f
#define C_SetRgbCameraDataFormat		0x0130
#define C_SetRgbCameraGain				0x0132
#define C_SetRgbCameraGamma				0x0133
#define C_SetDepthCompensate			0x0134
#define C_SetSystemTime					0x013e
#define C_SetCameraNumber				0x013f
#define C_SetLaserEnableStatus			0x0140
#define C_SetIMUExtrinsicParam			0x0151
#define C_SetTRSDsimilarMax				0x0160
#define C_SetTRSDpstMax					0x0161
#define C_SetCutHeight					0x0162
#define C_SetPalletIdentifyEnable		0x0163
#define C_SetPalletIdentifyType			0x0164
#define C_SetSecurityCalibrationParams	0x0171
#define C_SecurityCancelCalibration		0x0172
#define C_SetSecurityAxialAdjustment	0x0173
#define C_ResetSecurityConfigure		0x0174
#define C_SetSecurityZHSAEnable			0x0175
#define C_GetDeviceStatus				0x0200
#define C_GetVersion					0x0301
#define C_GetTimeInfo					0x0302
#define C_GetFrameRate					0x0303
#define C_GetTriggerMode				0x0304
#define C_GetHDR						0x0305
#define C_GetIntegralTime				0x0306
#define C_GetTimeFilter					0x0307
#define C_GetFlyPixeFilter				0x0308
#define C_GetConfidenceFilter			0x0309
#define C_GetKalmanFilter				0x030a
#define C_GetSpatialFilter				0x030b
#define C_GetNetworkModel				0x030c
#define C_GetIntegralModel				0x030d
#define C_GetTimeMedianFilter			0x030e
#define C_GetDeviceType					0x030f
#define C_GetIRGMMGain					0x0310
#define C_GetTimeStamp					0x0311
#define C_GetFrequencyModel				0x0313
#define C_GetOutputDO					0x0314
#define C_GetBinning					0x0320
#define C_GetTempCompensate				0x0321
#define C_GetTofCameraIntrinsicArg		0x0322
#define C_GetRgbCameraExtrinsicArg		0x0329
#define C_GetRgbCameraIntrinsicArg		0x032a
#define C_GetRgbCameraExposureMode		0x032b
#define C_GetRgbCameraExposureTime		0x032c
#define C_GetRgbCameraBrightness		0x032d
#define C_GetRgbCameraContrastRatio		0x032e

#define C_SetRgbSensorSaturation		0xffff
#define C_GetRgbSensorSaturation		0xffff
#define C_SetRgbSensorWhiteBalance		0xffff
#define C_GetRgbSensorWhiteBalance		0xffff
#define C_SetIMUFrequency				0xffff
#define C_GetIMUFrequency				0xffff

#define C_GetDelayHardTriggerTime		0x032f
#define C_GetRecvDataRows				0x0325
#define C_GetChipTemperature			0x0326
#define C_GetSN							0x0327
#define C_GetImageInfo					0x0328
#define C_GetRgbCameraResolutionRatio	0x0330
#define C_GetRgbCameraDataFormat		0x0331
#define C_GetRgbCameraGain				0x0332
#define C_GetRgbCameraGamma				0x0333
#define C_GetDepthCompensate			0x0334
#define C_GetCameraNumber				0x033f
#define C_GetLaserEnableStatus			0x0340
#define C_GetIMUData					0x0350
#define C_GetIMUExtrinsicParam			0x0351
#define C_GetTRSDsimilarMax				0x0360
#define C_GetTRSDpstMax					0x0361
#define C_GetCutHeight					0x0362
#define C_GetPalletIdentifyEnable		0x0363
#define C_GetPalletIdentifyType			0x0364
#define C_GetPalletIdentifySupport		0x0365
#define C_GetPalletConfigureFile		0x0366
#define C_HasSecurityAbility			0x0370
#define C_GetSecurityEnable				0x0371
#define C_GetSecurityLog				0x0372
#define C_GetSecurityConfigFile			0x0373
#define C_GetSecurityCalibrationEnable	0x0374
#define C_SecurityPerspectiveFile		0x0375
#define C_GetSecurityPerspectiveMatrixFile		0x0376
#define C_GetSecurityCalibrationMatrixParams	0x0377
#define C_GetSecurityZHSAEnable			0x0378
#define C_GetSecurityCalibrationFile	0x0379
#define C_FileArgTransfer				0x0500
#define C_FileDataTransfer				0x0501
#define C_FileAckTransfer				0x0502
#define C_OperateCommand				0x0f00
// 日志写入宏定义。
#define LOG_ERROR_OUT(...)	ljhNS::gLogger.writeLog(" ERROR ", __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__);
#define LOG_INFO_OUT(...)	ljhNS::gLogger.writeLog(" INFO ", __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__);


/// @brief 该命名空间里的所有定义均由算法组提供。
namespace algNS
{
	// 创建RGB相机的畸变校准表（由算法组提供）。
	static void createRGBCameraCalibrationTable(const LWSensorIntrinsicParam& rgb_intrinsic, float d2rScale, cv::Size size_, cv::Mat& rgbCalibMap1, cv::Mat& rgbCalibMap2, cv::Mat& rMatrixR2D, cv::Mat& tMatrixR2D, cv::Mat& rMatrixD2R, cv::Mat& tMatrixD2R)
	{
		// 内参矩阵初始化-用于rgb图像畸变校正
		cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
		camera_matrix.at<float>(0, 0) = rgb_intrinsic.fx;
		camera_matrix.at<float>(0, 2) = rgb_intrinsic.cx;
		camera_matrix.at<float>(1, 1) = rgb_intrinsic.fy;
		camera_matrix.at<float>(1, 2) = rgb_intrinsic.cy;
		camera_matrix.at<float>(2, 2) = 1.0f;

		//径向畸变参数-用于rgb图像畸变校正
		cv::Mat dist_matrix = cv::Mat(1, 5, CV_32F, cv::Scalar::all(0));
		dist_matrix.at<float>(0, 0) = rgb_intrinsic.k1;
		dist_matrix.at<float>(0, 1) = rgb_intrinsic.k2;
		dist_matrix.at<float>(0, 2) = rgb_intrinsic.p1;
		dist_matrix.at<float>(0, 3) = rgb_intrinsic.p2;
		dist_matrix.at<float>(0, 4) = rgb_intrinsic.k3;

		////重构新内参
		//cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_matrix, size_, 0, size_, nullptr);

		// 得到重构后保留FOV的映射表 每个像素下标用32位浮点型盛放
		cv::initUndistortRectifyMap(camera_matrix, dist_matrix, cv::Mat(), camera_matrix, size_, CV_32F, rgbCalibMap1, rgbCalibMap2);

		// 外参矩阵初始化及赋值—旋转矩阵&平移矩阵
		rMatrixR2D = camera_matrix * rMatrixR2D;
		tMatrixR2D = camera_matrix * tMatrixR2D;

		//
		camera_matrix.at<float>(0, 0) /= d2rScale;
		camera_matrix.at<float>(0, 2) /= d2rScale;
		camera_matrix.at<float>(1, 1) /= d2rScale;
		camera_matrix.at<float>(1, 2) /= d2rScale;

		rMatrixD2R = camera_matrix * rMatrixD2R;
		tMatrixD2R = camera_matrix * tMatrixD2R;
	}

	// 与Eigen矩阵运算相关的函数。
	static Eigen::Matrix3d getRX_left(float radian)
	{
		Eigen::Matrix3d RX_left;
		RX_left << 1, 0, 0,
			0, cos(radian), sin(radian),
			0, -sin(radian), cos(radian);
		return 	RX_left;
	};
	static Eigen::Matrix3d getRY_left(float radian)
	{
		Eigen::Matrix3d RY_left;
		RY_left << cos(radian), 0, -sin(radian),
			0, 1, 0,
			sin(radian), 0, cos(radian);
		return 	RY_left;
	};
	static Eigen::Matrix3d getRZ_left(float radian)
	{
		Eigen::Matrix3d RZ_left;
		RZ_left << cos(radian), sin(radian), 0,
			-sin(radian), cos(radian), 0,
			0, 0, 1;
		return 	RZ_left;
	};
	static Eigen::Matrix3d getRX_right(float radian)
	{
		Eigen::Matrix3d RX_right;
		RX_right << 1, 0, 0,
			0, cos(radian), -sin(radian),
			0, sin(radian), cos(radian);
		return 	RX_right;
	};
	static Eigen::Matrix3d getRY_right(float radian)
	{
		Eigen::Matrix3d RY_right;
		RY_right << cos(radian), 0, sin(radian),
			0, 1, 0,
			-sin(radian), 0, cos(radian);
		return 	RY_right;
	};
	static Eigen::Matrix3d getRZ_right(float radian)
	{
		Eigen::Matrix3d RZ_right;
		RZ_right << cos(radian), -sin(radian), 0,
			sin(radian), cos(radian), 0,
			0, 0, 1;
		return 	RZ_right;
	};

}


namespace ljhNS
{
/// ******************************************* 内部工具函数 *******************************************

	// 获取文件的MD5值。
	static LWReturnCode getFileMD5(const std::string& fileName, unsigned char* md5, unsigned int& md5_len)
	{
		// 打开文件
		std::ifstream file(fileName, std::ios::binary | std::ios::in);
		if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;
		// 创建上下文
		EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
		EVP_DigestInit_ex(mdctx, EVP_md5(), NULL);
		char buffer[10240];
		while (file.read(buffer, 10240) || file.good())
		{
			EVP_DigestUpdate(mdctx, buffer, file.gcount());
		}
		file.close();
		// 计算并获取结果
		EVP_DigestFinal_ex(mdctx, md5, &md5_len);
		// 清理并释放资源
		EVP_MD_CTX_free(mdctx);

		return LW_RETURN_OK;
	}

	// 从含有路径的文件名里截取文件名。
	static std::string getFileNameFromStr(const char* str)
	{
		int begin = 0;
		int len = 0;
		for (int i = 0, n = strlen(str); i < n; ++i)
		{
			if ((str[i] == '\\') || (str[i] == '/'))
			{
				begin = i + 1;
				len = 0;
			}
			else {
				++len;
			}
		}

		return std::string(str + begin, len);
	}

	// 将传入的字节流转换为十六进制字符串。
	static std::string getHexStringFromBytes(const void* data, uint32_t length, char ch = '\0')
	{
		if (data == nullptr) return "";
		auto bytes = (const uint8_t*)data;
		std::string buff = "0x";
		int32_t high;
		int32_t low;

		for (uint32_t i = 0; i < length; i++)
		{
			if (i != 0 && ch != '\0') buff += ch;
			high = bytes[i] / 16;
			low = bytes[i] % 16;
			buff += char((high < 10) ? ('0' + high) : ('a' + high - 10));
			buff += char((low < 10) ? ('0' + low) : ('a' + low - 10));
		}

		return buff;
	}

	// 当特定的套接字函数指示发生了错误时，应立即调用此函数以检索失败的函数调用的扩展错误代码描述信息。
	static std::string getNetworkLastError()
	{
	#if defined(_WIN32)
		auto error_code = WSAGetLastError();
		if (error_code == 0) return "";
		char error[256];
		FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ARGUMENT_ARRAY, NULL, error_code, NULL, error, 255, NULL);
		return error;
	#else
		if (errno == 0) return "";
		return strerror(errno);
	#endif
	}

	// 设置网络数据接收超时。
	static bool setNetworkTimeout(SOCKET& sck, uint32_t t)
	{
	#if defined(_WIN32)
		return setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, (const char*)(&t), sizeof(t)) == 0;
	#else
		timeval tv{ t / 1000, t % 1000 * 1000 };
		return setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0;
	#endif
	}

	// 关闭套接字。
	static bool closeSocket(SOCKET& sck)
	{
		if (sck == INVALID_SOCKET) return true;

	#if defined(_WIN32)
		if (closesocket(sck) == SOCKET_ERROR) return false;
	#else
		if (shutdown(sck, SHUT_RDWR) == SOCKET_ERROR || close(sck) == SOCKET_ERROR) return false;
	#endif

		sck = INVALID_SOCKET;
		return true;
	}

	// 建立TCP连接。
	static bool connectToServer(SOCKET& sck, sockaddr_in& addr, uint32_t timeout, bool alive = true)
	{
		timeval tv{ timeout / 1000U, timeout % 1000U * 1000U };
		fd_set writefds;
		FD_ZERO(&writefds);
		FD_SET(sck, &writefds);

	#if defined(_WIN32)
		// 设为非阻塞模式
		u_long iMode = 1;
		if (ioctlsocket(sck, FIONBIO, &iMode) == SOCKET_ERROR) return false;
		// TCP连接
		connect(sck, (sockaddr*)&addr, sizeof(addr));
		// 等待连接直至超时
		auto ret = select(0, NULL, &writefds, NULL, &tv);
		if (ret == 0 || ret == SOCKET_ERROR) return false;
		// 设回阻塞模式
		iMode = 0;
		if (ioctlsocket(sck, FIONBIO, &iMode) == SOCKET_ERROR) return false;
		// 心跳检测
		if (alive)
		{
			// 启用自动心跳检测机制
			BOOL    keep_alive = 1;
			setsockopt(sck, SOL_SOCKET, SO_KEEPALIVE, (const char*)&keep_alive, sizeof(keep_alive));
			// 首次检测的时间间隔
			DWORD   keep_idle = 1;
			setsockopt(sck, IPPROTO_TCP, TCP_KEEPIDLE, (const char*)&keep_idle, sizeof(keep_idle));
			// 每次检测的时间间隔
			DWORD   keep_interval = 1;
			setsockopt(sck, IPPROTO_TCP, TCP_KEEPINTVL, (const char*)&keep_interval, sizeof(keep_interval));
			// 检测失败的最大次数
			DWORD   keep_count = 2;
			setsockopt(sck, IPPROTO_TCP, TCP_KEEPCNT, (const char*)&keep_count, sizeof(keep_count));
		}

	#else
		// 设为非阻塞模式
		auto flags = fcntl(sck, F_GETFL, 0);
		if (fcntl(sck, F_SETFL, flags | O_NONBLOCK) < 0) return false;
		// TCP连接
		if (connect(sck, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
		{
			if (errno != EINPROGRESS) return false;
			// 等待连接直至超时
			auto rc = select(sck + 1, NULL, &writefds, NULL, &tv);
			if (rc <= 0)
			{
				// 超时或错误
				if (rc == 0) errno = ETIMEDOUT;
				return false;
			}
			// 检查套接字错误
			int error;
			socklen_t len = sizeof(error);
			if (getsockopt(sck, SOL_SOCKET, SO_ERROR, &error, &len) == SOCKET_ERROR) return false;
			if (error != 0)
			{
				// 连接失败
				errno = error;
				return false;
			}
		}
		// 恢复阻塞模式（可选）
		if (fcntl(sck, F_SETFL, flags) < 0) return false;
		// 心跳检测
		if (alive)
		{
			// 启用心跳检测机制
			int keep_alive = 1;
			setsockopt(sck, SOL_SOCKET, SO_KEEPALIVE, (void*)&keep_alive, sizeof(keep_alive));
			// 首次检测的时间间隔
			int keep_idle = 1;
			setsockopt(sck, SOL_TCP, TCP_KEEPIDLE, (void*)&keep_idle, sizeof(keep_idle));
			// 每次检测的时间间隔
			int keep_interval = 1;
			setsockopt(sck, SOL_TCP, TCP_KEEPINTVL, (void*)&keep_interval, sizeof(keep_interval));
			// 检测失败的最大次数
			int keep_count = 2;
			setsockopt(sck, SOL_TCP, TCP_KEEPCNT, (void*)&keep_count, sizeof(keep_count));
		}

	#endif

		return true;
	}

	// 获取本机所有IP地址并建立对应的具有广播功能的套结字。
	std::map<uint32_t, SOCKET> gSocketMap;
	static bool getLocalAddrInfo(std::vector<sockaddr_in>& sockaddr_list)
	{
		int bOpt = true;

	#if defined(_WIN32)
		char  hostname[255];
		if (gethostname(hostname, sizeof(hostname)) == SOCKET_ERROR) return false;

		struct addrinfo hints;
		struct addrinfo* result = NULL;
		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_INET;
		if (getaddrinfo(hostname, NULL, &hints, &result) != 0) return false;
		for (addrinfo* ptr = result; ptr != NULL; ptr = ptr->ai_next)
		{
			auto local_addr = *((sockaddr_in*)(ptr->ai_addr));
			local_addr.sin_port = 0;
			sockaddr_list.push_back(local_addr);
			if (gSocketMap.find(local_addr.sin_addr.s_addr) == gSocketMap.end())
			{
				auto _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
				if ((bind(_socket, (sockaddr*)(&local_addr), sizeof(local_addr)) != SOCKET_ERROR)
					&& (setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR))
				{
					gSocketMap[local_addr.sin_addr.s_addr] = _socket;
				}
			}
		}
		if (result) freeaddrinfo(result);
	#else
		auto loop_ip = htonl(INADDR_LOOPBACK);
		struct ifaddrs* ifhead = nullptr;
		if (getifaddrs(&ifhead) == SOCKET_ERROR) return false;
		for (ifaddrs* ifpoint = ifhead; ifpoint != nullptr; ifpoint = ifpoint->ifa_next)
		{
			if (ifpoint->ifa_addr == nullptr) continue;
			//#if defined(__aarch64__)
			//        if (ifpoint->ifa_name[0] != 'e') continue;
			//#endif
			if (ifpoint->ifa_addr->sa_family == AF_INET)
			{
				auto local_addr = *(sockaddr_in*)ifpoint->ifa_addr;
				if (local_addr.sin_addr.s_addr != loop_ip)
				{
					local_addr.sin_port = 0;
					sockaddr_list.push_back(local_addr);
					if (gSocketMap.find(local_addr.sin_addr.s_addr) == gSocketMap.end())
					{
						auto _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
						if ((bind(_socket, (sockaddr*)(&local_addr), sizeof(local_addr)) != SOCKET_ERROR)
							&& (setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR))
						{
							gSocketMap[local_addr.sin_addr.s_addr] = _socket;
						}
					}
				}
			}
		}
		if (ifhead) freeifaddrs(ifhead);

	#endif

		return true;
	}

	// 网络异常检测回调函数。
	void(*networkAbnormalCallback)(LWDeviceHandle handle, const char* error, void* pUserData) = nullptr;
	void* pUserData1 = nullptr;

	// 帧数据刷新回调函数。
	void(*frameReadyCallback)(LWDeviceHandle handle, void* pUserData) = nullptr;
	void* pUserData2 = nullptr;


	/// ******************************************* 内部数据结构类型 *******************************************

	#if defined(_WIN32)
	/// @brief 通过进程启动对 Winsock DLL 的使用。
	class WinsockDllAutoManager
	{
	public:
		WinsockDllAutoManager()
		{
			WSADATA wsaData;
			auto ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
			if (ret != 0)
			{
				char error[256];
				FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ARGUMENT_ARRAY, NULL, ret, NULL, error, 255, NULL);
				throw std::runtime_error(error);
			}
		}

		~WinsockDllAutoManager()
		{
			WSACleanup();
		}

	private:

	} gWinsockDllAutoManager;
	#endif


	/// @brief 用于浮点数的大小端转换
	union FloatUint32 {
		float f;
		uint32_t u;
	};


	/// @brief 日志类，以本地网络通信的方式进行日志异步写入操作。
	class Logger
	{
	public:
		Logger(Logger&& obj) = delete;
		Logger(const Logger& obj) = delete;
		Logger& operator=(Logger&& obj) = delete;
		Logger& operator=(const Logger& obj) = delete;

		Logger()
		{
			// 打开日志文件
			mFileHandle.open("logger.txt", std::ios::app);
			if (mFileHandle.fail()) throw std::runtime_error("Failed to open log file!");
			// 获取日志文件大小
			struct stat file_info {};
			stat("logger.txt", &file_info);
			bufLen = file_info.st_size;
			// 初始化套接字
			sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			recvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			// 设置接收地址为本机回环地址
			recvAddr.sin_family = AF_INET;
			recvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
			recvAddr.sin_port = 0;
			// 绑定地址
			socklen_t addrlen = sizeof(sockaddr_in);
			if ((bind(recvSocket, (sockaddr*)&recvAddr, sizeof(recvAddr)) != SOCKET_ERROR)
				&& (getsockname(recvSocket, (sockaddr*)&recvAddr, &addrlen) != SOCKET_ERROR))
			{
				std::thread{ &Logger::writeFile, this }.detach();
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
			else
			{
				// 获取日期和时间
				std::stringstream ss;
				auto t0 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				ss << std::put_time(localtime(&t0), "%Y-%m-%d %H:%M:%S");

				mFileHandle << ss.str() << " ERROR " << getNetworkLastError();
			}
		}

		~Logger()
		{

			closeSocket(sendSocket);
			closeSocket(recvSocket);

			std::this_thread::sleep_for(std::chrono::milliseconds(15));
			mFileHandle.close();
		}

		// 写日志
		void writeLog(
			const char* loglevel,	// Log级别
			const char* fileName,	// 函数所在文件名
			const char* function,	// 函数名
			int lineNumber,			// 行号
			const char* format,		// 格式化
			...)					// 变量
		{
			logBuffer.clear();

			// 获取日期和时间
			std::stringstream ss;
			auto t0 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			ss << std::put_time(localtime(&t0), "%Y-%m-%d %H:%M:%S");
			logBuffer += ss.str();

			//	日志级别
			logBuffer += loglevel;

			// 日志辅助信息
			logBuffer += std::string(strrchr(fileName, PATH_SEPARATOR) + 1) + " " + function + " " + std::to_string(lineNumber) + " ";

			// 日志正文
			if (format[0] != '\0')
			{
				va_list ap;
				va_start(ap, format);
				vsnprintf(logInfo, 1024, format, ap);
				va_end(ap);

				logBuffer += std::string(logInfo);
			}

			// 写入日志内容
			auto ret = sendto(sendSocket, logBuffer.c_str(), logBuffer.size(), MSG_NOSIGNAL, (sockaddr*)&recvAddr, sizeof(recvAddr));
			if (ret == SOCKET_ERROR) mFileHandle << ss.str() << " ERROR " << getNetworkLastError();
		}

	private:
		void writeFile()
		{
			char buffer[1024];
			decltype(recvfrom(recvSocket, nullptr, 0, 0, nullptr, nullptr)) ret = 0;

			while (true)
			{
				ret = recvfrom(recvSocket, buffer, 1024, 0, nullptr, nullptr);
				if (ret < 1) break;

				mFileHandle << std::string(buffer, ret) << std::endl;
				mFileHandle.flush();

				bufLen += ret;
				if (bufLen > LOG_FILE_MAX_SAVE_SIZE)
				{
					mFileHandle.close();
					if (rename("logger.txt", "logger_old.txt") == -1) throw std::runtime_error("Failed to rename log file!");

					mFileHandle.open("logger.txt", std::ios::ate);
					if (mFileHandle.fail()) throw std::runtime_error("Failed to open log file!");

					bufLen = 0;
				}
			}

			mFileHandle.close();
		}

		// 
		sockaddr_in	recvAddr;
		SOCKET		sendSocket = INVALID_SOCKET;
		SOCKET		recvSocket = INVALID_SOCKET;
		// 日志正文信息
		char logInfo[1024]{};
		// 存储log的buffer
		std::string logBuffer;
		// 文件句柄
		std::ofstream mFileHandle;
		// 日志当前尺寸
		size_t bufLen = 0;

	} gLogger;


	/// @brief 临时授权，用于检测函数执行的完整性。
	class ProvisionalAuthority
	{
	public:
		explicit ProvisionalAuthority(std::atomic<bool>& val) :flag(val)
		{
			flag.store(true);
		}

		~ProvisionalAuthority()
		{
			if (isRestore) flag.store(false);
		}

		void authorize()
		{
			isRestore = false;
		}

	private:
		bool isRestore = true;
		std::atomic<bool>& flag;
	};


	/// @brief 自动内存管理。
	class AutoMemoryManager
	{
	public:
		explicit AutoMemoryManager(int len) : bufLen(len)
		{
			buffer = new char[len];
		}

		~AutoMemoryManager()
		{
			delete[] buffer;
		}

		void memset_v(int val)
		{
			memset(buffer, val, bufLen);
		}

		int size() const
		{
			return bufLen;
		}

		char* data()
		{
			return buffer;
		}

		unsigned char* u_data()
		{
			return (unsigned char*)buffer;
		}

	private:
		int   bufLen = 0;
		char* buffer = nullptr;

	};


	/// @brief 线程计数器。
	class ThreadCounter
	{
	public:
		explicit ThreadCounter(std::atomic<int>& _count) : count(_count)
		{
			count.fetch_add(1);
		}

		~ThreadCounter()
		{
			count.fetch_sub(1);
		}

	private:
		std::atomic<int>& count;

	};


	/// @brief 数据节点。
	class DataNode
	{
	public:
		explicit DataNode(int len) : size(len)
		{
			buffer = new char[len];
			variant = new LWVariant;
		}

		~DataNode()
		{
			delete[] buffer;
			delete variant;
		}

		unsigned char* data()
		{
			return (unsigned char*)buffer;
		}

		DataNode(DataNode&& obj) = delete;
		DataNode(const DataNode& obj) = delete;
		DataNode& operator=(const DataNode& obj) = delete;


	public:
		bool				update = false;
		uint32_t			mark = 0;
		uint32_t			serial = 0;
		uint32_t			size = 0;
		LWRgbTransferFormat	rgbTFormat;
		LWTemperature		temperature{};
		LWTimeStamp			time{};
		LWVariant*			variant = nullptr;

	private:
		char* buffer = nullptr;

	};


	/// @brief 非线程安全循环队列。注：由于读、写线程的执行机制，该队列不会出现由同步而引发的问题。
	class LoopQueueNCP
	{
	public:
		LoopQueueNCP(int _len, int _size) : size(_size)
		{
			for (int i = 0; i < size; i++) data[i] = new DataNode(_len);
		}

		~LoopQueueNCP()
		{
			for (int i = 0; i < size; i++) delete data[i];
		}

		/// @brief 重置队列数据。
		void clear()
		{
			writeIndex = 0;
			writeCounter = 0;

			for (int i = 0; i < size; i++)
			{
				data[i]->update = false;
				data[i]->serial = 0;
			}
		}

		/// @brief 入队（实际上是与指定节点进行替换）。
		/// @param[in,out] obj 替换值
		void enqueue(DataNode*& obj)
		{
			writeIndex = writeCounter++ % size;
			obj->update = true;
			std::swap(data[writeIndex], obj);
		}

		/// @brief 获取最新节点。
		/// @return 最新节点
		DataNode* latestNode() const
		{
			data[writeIndex]->mark = writeIndex;
			return data[writeIndex];
		}

		/// @brief 出队（实际上是与指定节点进行替换）。
		/// @param[in] index 节点索引
		/// @param obj 替换对象
		void dequeue(uint32_t index, DataNode*& obj)
		{
			obj->update = false;
			std::swap(data[index], obj);
		}

		/// @brief 根据序列号查找对应的节点。
		/// @param num 序列号
		/// @return 节点指针
		DataNode* find(uint32_t num) const
		{
			for (int32_t i = 0, k = int32_t(writeIndex) - 1; i < size; ++i, --k)
			{
				if (k < 0) k = size - 1;
				if (data[k]->serial == num)
				{
					data[k]->mark = k;
					return data[k]->update ? data[k] : nullptr;
				}
			}

			return nullptr;
		}

	private:
		int32_t		size = 0;
		uint32_t	writeIndex = 0;
		uint32_t	writeCounter = 0;
		DataNode* data[LOOP_QUEUE_SIZE]{};
	};


	/// @brief 发送命令结构体。
	class CommandSFrame
	{
	public:
		CommandSFrame(uint16_t command = 0xff, int32_t len = COMMAND_MAX_SIZE) {
			bufferLen = len;
			buffer = new uint8_t[len]{
				0xA5, 0x5A, 0xAA, 0x55,     //0  帧头
				PROTOCOL_VERSION,           //4  协议版本
				0x00,                       //5  命令类别
				0x00, 0x00,                 //6  命令码
				0x00, 0x01,                 //8  总分包数
				0x00, 0x01,                 //10 当前包号
				0x00, 0x00,                 //12 数据域长度

				0x5A, 0xA5, 0x55, 0xAA      //14 帧尾
			};
			command = htons(command);
			memcpy(buffer + 6, &command, 2);
		}

		CommandSFrame(char* buf, int32_t len) = delete;
		CommandSFrame& operator=(const CommandSFrame& obj) = delete;

		~CommandSFrame() {
			delete[] buffer;
		}

		/// @brief 获取命令的版本信息。
		/// @return 
		uint8_t getVersion() {
			return buffer[4];
		}

		/// @brief 获取命令的类型。
		/// @return 
		uint8_t getCommandType() {
			return buffer[5];
		}

		/// @brief 获取命令码。
		/// @return 
		uint16_t getCommand() {
			return ntohs(*(uint16_t*)(buffer + 6));
		}

		/// @brief 设置命令的版本协议。
		/// @param[in] version 协议版本
		void setVersion(uint8_t version) {
			buffer[4] = version;
		}

		/// @brief 设置命令类别。
		/// @param[in] type 类别
		void setCommandType(uint8_t type) {
			buffer[5] = type;
		}

		/// @brief 设置命令码。
		/// @param[in] commandcode 命令码
		void setCommand(uint16_t commandcode) {
			commandcode = htons(commandcode);
			memcpy(buffer + 6, &commandcode, 2);
		}

		/// @brief 设置命令的总分包数。
		/// @param[in] number 总包数
		void setTotalSerialNumber(uint16_t number) {
			number = htons(number);
			memcpy(buffer + 8, &number, 2);
		}

		/// @brief 设置当前的命令分包号。
		/// @param[in] _number 分包号
		void setCurrentSerialNumber(uint16_t _number) {
			_number = htons(_number);
			memcpy(buffer + 10, &_number, 2);
		}

		/// @brief 设置命令的数据域。
		/// @param[in] buf 数据域地址
		/// @param[in] len 数据的长度
		void setArgField(const void* buf, uint16_t len) {
			if (buf == nullptr)
			{
				memcpy(buffer + 12, &len, 2);
				memcpy(buffer + 14, footer, 4);

				return;
			}

			if ((bufferLen - 18) < len)
			{
				bufferLen = len + 18;
				auto new_buffer = new uint8_t[bufferLen];
				memcpy(new_buffer, buffer, 12);
				delete[] buffer;
				buffer = new_buffer;
			}

			auto len_ = htons(len);
			memcpy(buffer + 12, &len_, 2);
			memcpy(buffer + 14, buf, len);
			memcpy(buffer + 14 + len, footer, 4);

		}

		/// @brief 获取指定数据包的数据域长度
		/// @param num 包编号（从0编号开始）
		/// @return 数据域长度
		uint16_t getArgFieldLength()
		{
			return ntohs(*(uint16_t*)(buffer + 12));
		}

		/// @brief 获取命令的数据域。
		/// @return 
		char* getArgField() {
			return (char*)(buffer + 14);
		}

		/// @brief 判定命令是否是指定命令。
		/// @param commandcode 命令码
		/// @return 
		bool isCommand(uint16_t commandcode) {
			return getCommand() == commandcode
				&& memcmp(buffer, header, 4) == 0
				&& memcmp(buffer + 14 + ntohs(*(uint16_t*)(buffer + 12)), footer, 4) == 0;
		}

		/// @brief 获取命令的总长度。
		/// @return 命令长度
		int32_t size() {
			return ntohs(*(uint16_t*)(buffer + 12)) + 18;
		}

		/// @brief 获取命令地址。
		/// @return 命令地址
		char* data() {
			return (char*)buffer;
		}

		/// @brief 获取命令缓存区的大小。
		/// @return 
		int32_t  maxBuffer() const {
			return bufferLen;
		}


	protected:
		uint8_t		header[4] = { 0xA5, 0x5A, 0xAA, 0x55 }; ///< 帧协议头
		uint8_t		footer[4] = { 0x5A, 0xA5, 0x55, 0xAA }; ///< 帧协议尾

		uint8_t*	buffer = nullptr;
		int32_t		bufferLen = 0;  ///< 缓冲区大小

	};


	/// @brief 接收命令结构体。
	class CommandRFrame
	{
	public:
		CommandRFrame()
		{
			bufferLen = COMMAND_MAX_SIZE;
			buffer = new char[COMMAND_MAX_SIZE];
		}

		~CommandRFrame()
		{
			delete[] buffer;
		}

		/// @brief 获取命令码。
		/// @return 
		uint16_t getCommand() {
			return ntohs(*(uint16_t*)(buffer + 6));
		}

		/// @brief 获取命令类型
		/// @return 
		uint8_t getCommandType() {
			return buffer[5];
		}

		/// @brief 获取命令的总分包数。
		/// @return 
		uint16_t getTotalSerialNumber() {
			return ntohs(*(uint16_t*)(buffer + 8));
		}

		/// @brief 获取当前命令的分包号。
		/// @return 
		uint16_t getCurrentSerialNumber() {
			return ntohs(*(uint16_t*)(buffer + 10));
		}

		/// @brief 获取指定数据包的数据域首地址
		/// @param num 包编号（从0编号开始）
		/// @return 数据域地址
		char* getArgField(int num = 0)
		{
			auto ptr = buffer;

			for (int i = 0; i < num; i++)
			{
				auto len = ntohs(*(uint16_t*)(ptr + 12));
				ptr += (len + 18);
			}

			return  ptr + 14;
		}

		/// @brief 获取指定数据包的数据域长度
		/// @param num 包编号（从0编号开始）
		/// @return 数据域长度
		uint16_t getArgFieldLength(int num = 0)
		{
			auto ptr = buffer;

			for (int i = 0; i < num; i++)
			{
				auto len = ntohs(*(uint16_t*)(ptr + 12));
				ptr += (len + 18);
			}

			return ntohs(*(uint16_t*)(ptr + 12));
		}

		/// @brief 获取命令的大小
		/// @return 
		int32_t size()
		{
			auto ptr = buffer;
			auto num = getTotalSerialNumber();
			auto sum = 0;

			for (int i = 0; i < num; i++)
			{
				sum += (ntohs(*(uint16_t*)(ptr + 12)) + 18);
				ptr = buffer + sum;
			}
			return sum;
		}

		/// @brief 根据指定命令码判定是否命令结构。
		/// @param commandcode 命令码。
		/// @return 
		bool isCommand(uint16_t commandcode)
		{
			if (getTotalSerialNumber() < 1) return false;

			auto ptr = buffer;
			int fielen = 0;
			int sumlen = 0;
			for (int i = 0, k = getTotalSerialNumber(); i < k; ++i)
			{
				fielen = ntohs(*(uint16_t*)(ptr + 12));
				sumlen += (fielen + 18);
				if ((dataLen < sumlen)
					|| (ntohs(*(uint16_t*)(ptr + 6)) != commandcode)
					|| (memcmp(ptr, header, 4) != 0)
					|| (memcmp(ptr + 14 + fielen, footer, 4) != 0)
					)
				{
					return false;
				}

				ptr += (fielen + 18);
			}

			return true;
		}

		/// @brief 根据指定命令码判定是否命令头。
		/// @param commandcode 命令码。
		/// @return 
		bool isCommandHeader(uint16_t commandcode) {
			return (memcmp(buffer, header, 4) == 0)
				&& getCommand() == commandcode;
		}

		/// @brief 向命令的数据域写入数据。
		/// @param data 
		/// @param len 
		void push(void* data, int32_t len)
		{
			if (dataLen > 1024 * 1024 * 50) return;

			auto mLen = dataLen + len;
			if (mLen > bufferLen)
			{
				bufferLen = mLen * 2;
				auto newbuffer = new char[bufferLen];
				memcpy(newbuffer, buffer, dataLen);
				delete[] buffer;
				buffer = newbuffer;
			}
			memcpy(buffer + dataLen, data, len);
			dataLen += len;
		}

		/// @brief 清空命令所有内容。
		void clear()
		{
			dataLen = 0;

			if (bufferLen > COMMAND_MAX_SIZE)
			{
				delete[] buffer;
				buffer = new char[COMMAND_MAX_SIZE];
				bufferLen = COMMAND_MAX_SIZE;
			}

		}

		/// @brief 获取命令的首地址
		/// @return 
		char* data()
		{
			return	buffer;
		}

		/// @brief 获取存储命令的缓存区大小。
		/// @return 
		int32_t  maxBuffer() const
		{
			return bufferLen;
		}

	private:
		uint8_t	header[4] = { 0xA5, 0x5A, 0xAA, 0x55 }; ///< 帧协议头
		uint8_t	footer[4] = { 0x5A, 0xA5, 0x55, 0xAA }; ///< 帧协议尾
		int32_t	bufferLen = 0;  ///< 命令最大的内存占用数
		int32_t	dataLen = 0;  ///< 命令数据的内存占用数
		char* buffer = nullptr;

	};


	/// @brief 设备管理类
	class DeviceHandle
	{
	public:
		explicit DeviceHandle(LWDeviceHandle _handle);
		DeviceHandle(DeviceHandle&& obj) = delete;
		DeviceHandle(const DeviceHandle& obj) = delete;
		DeviceHandle& operator=(DeviceHandle&& obj) = delete;
		DeviceHandle& operator=(const DeviceHandle& obj) = delete;
		~DeviceHandle();

		LWReturnCode OpenDevice();
		LWReturnCode CloseDevice();
		LWReturnCode ReconnectDevice(uint32_t t);
		LWReturnCode RebootDevice();
		LWReturnCode SaveConfigureInfo();
		LWReturnCode RemoveConfigureInfo();
		LWReturnCode RestoreFactoryConfigureInfo();
		LWReturnCode StartStream();
		LWReturnCode StopStream();
		LWReturnCode HasRgbModule(bool& value);
		LWReturnCode SoftTrigger();
		LWReturnCode SetDataReceiveType(LWDataRecvType type);
		LWReturnCode SetTriggerMode(LWTriggerMode mode);
		LWReturnCode GetTriggerMode(LWTriggerMode& mode);
		LWReturnCode SetExposureMode(LWSensorType sensorType, LWExposureMode mode);
		LWReturnCode GetExposureMode(LWSensorType sensorType, LWExposureMode& mode);
		LWReturnCode SetHDRMode(LWHDRMode mode);
		LWReturnCode GetHDRMode(LWHDRMode& mode);
		LWReturnCode SetTransformRgbToDepthEnable(bool enable);
		LWReturnCode SetTransformDepthToRgbEnable(bool enable);
		LWReturnCode SetFrameRate(int32_t value);
		LWReturnCode GetFrameRate(int32_t& value);
		LWReturnCode SetExposureTime(LWSensorType sensorType, const int32_t* exposureTimeArray, int32_t arraySize);
		LWReturnCode GetExposureTime(LWSensorType sensorType, int32_t* exposureTimeArray, int32_t arraySize, int32_t& filledCount);
		LWReturnCode GetResolution(LWSensorType type, int32_t& width, int32_t& height);
		LWReturnCode GetIntrinsicParam(LWSensorType sensorType, LWSensorIntrinsicParam& intrinsicParam);
		LWReturnCode GetExtrinsicParam(LWSensorType sensorType, LWSensorExtrinsicParam& extrinsicParam);
		LWReturnCode SetTimeFilterParams(LWFilterParam param);
		LWReturnCode GetTimeFilterParams(LWFilterParam& param);
		LWReturnCode SetFlyingPixelsFilterParams(LWFilterParam param);
		LWReturnCode GetFlyingPixelsFilterParams(LWFilterParam& param);
		LWReturnCode SetConfidenceFilterParams(LWFilterParam param);
		LWReturnCode GetConfidenceFilterParams(LWFilterParam& param);
		LWReturnCode SetSpatialFilterParams(LWFilterParam param);
		LWReturnCode GetSpatialFilterParams(LWFilterParam& param);
		LWReturnCode SetTimeMedianFilterParams(LWFilterParam param);
		LWReturnCode GetTimeMedianFilterParams(LWFilterParam& param);
		LWReturnCode SetIRGMMGain(int32_t value);
		LWReturnCode GetIRGMMGain(int32_t& gain);
		LWReturnCode SetRgbSensorGain(int32_t value);
		LWReturnCode GetRgbSensorGain(int32_t& value);
		LWReturnCode SetRgbSensorGamma(int32_t value);
		LWReturnCode GetRgbSensorGamma(int32_t& value);
		LWReturnCode SetRgbSensorBrightness(int32_t value);
		LWReturnCode GetRgbSensorBrightness(int32_t& value);
		LWReturnCode SetRgbSensorContrastRatio(int32_t value);
		LWReturnCode GetRgbSensorContrastRatio(int32_t& value);
		LWReturnCode SetRgbSensorSaturation(int32_t value);
		LWReturnCode GetRgbSensorSaturation(int32_t& value);
		LWReturnCode SetRgbSensorWhiteBalance(int32_t value);
		LWReturnCode GetRgbSensorWhiteBalance(int32_t& value);
		LWReturnCode SetNetworkInfo(LWNetworkInfo info);
		LWReturnCode GetNetworkInfo(LWNetworkInfo& info);
		LWReturnCode SetDeviceNumber(int32_t value);
		LWReturnCode GetDeviceNumber(int32_t& value);
		LWReturnCode SetDepthCompensateValue(int32_t value);
		LWReturnCode GetDepthCompensateValue(int32_t& value);
		LWReturnCode SetHardTriggerFilterParams(int32_t t1, int32_t t2);
		LWReturnCode GetHardTriggerFilterParams(int32_t& t1, int32_t& t2);
		LWReturnCode GetIMUExtrinsicParam(LWIMUExtrinsicParam& para);
		LWReturnCode GetIMUData(LWIMUData& imu_data);
		LWReturnCode SetIMUFrequency(int32_t value);
		LWReturnCode GetDeviceSN(char* _SN_, int32_t bufferLen);
		LWReturnCode GetDeviceType(char* type, int32_t bufferLen);
		LWReturnCode GetTimeStamp(LWTimeStamp& t);
		LWReturnCode GetDeviceVersion(LWVersionInfo& fv, LWVersionInfo& dv);
		LWReturnCode GetFrameReady();
		LWReturnCode UpdateFirmware(const char* filename);
		LWReturnCode SetOutputDO(int32_t channel, int32_t value);
		LWReturnCode GetOutputDO(int32_t channel, int32_t& value);

		/*托盘模块*/
		LWReturnCode HasPalletIdentifyModule(bool& value);
		LWReturnCode UploadRKNNFile(const char* filename);
		LWReturnCode SetPalletConfigureFile(const char* filename);
		LWReturnCode SetPalletConfigureFileFromBuffer(const char* buf, int32_t bufLen);
		LWReturnCode GetPalletConfigureFile(const char* filename);
		LWReturnCode GetPalletConfigureFileToBuffer(char* buf, int32_t len);
		LWReturnCode SetPalletIdentifyType(const char* type);
		LWReturnCode GetPalletIdentifyType(char* type, int32_t len);
		LWReturnCode SetPalletIdentifyEnable(bool enable);
		LWReturnCode GetPalletIdentifyEnable(bool& enable);
		LWReturnCode SetTRSDsimilarMax(float val);
		LWReturnCode GetTRSDsimilarMax(float& val);
		LWReturnCode SetTRSDpstMax(float val);
		LWReturnCode GetTRSDpstMax(float& val);
		LWReturnCode SetCutHeight(int32_t val);
		LWReturnCode GetCutHeight(int32_t& val);
		LWReturnCode SendFile(const char* fullname, uint32_t type);
		LWReturnCode SendFile(const char* buffer, int len, uint32_t type);


	#ifdef LW_INTERNAL_API
		LWReturnCode SetDeviceSN(const char* _SN_, int size);
		LWReturnCode SendOperateCommand(const char* comstr, int size);
		LWReturnCode SetDRNU(bool enable);
		LWReturnCode SetBinningMode(LWBinningMode mode);
		LWReturnCode SetResolution(LWSensorType sensorType, int32_t width, int32_t height);
		LWReturnCode SetDistortionCalibration(LWSensorType sensorType, bool enable);
		LWReturnCode SetLaserWorkFrequency(const uint8_t* arr, int size);
		LWReturnCode SetAutoExposureDefaultValue(uint16_t val);
		LWReturnCode SetIntrinsicParam(LWSensorType sensorType, const LWSensorIntrinsicParam& para);
		LWReturnCode SetExtrinsicParam(LWSensorType sensorType, const LWSensorExtrinsicParam& para);
		LWReturnCode SetIMUExtrinsicParam(const LWIMUExtrinsicParam& para);
		LWReturnCode SetTemperatureCompensation(bool enable);
		LWReturnCode GetTemperatureCompensation(bool& enable);
		LWReturnCode SetTemperatureParams(const LWTemperatureParams& para);
		LWReturnCode SetLaserEnableStatus(uint32_t flag);
		LWReturnCode GetLaserEnableStatus(uint32_t& flag);
		LWReturnCode SetDataSyncEnable(bool enable);
	#endif //LW_INTERNAL_API


	#ifdef LW_ZHONGRILONG_PROJ
		LWReturnCode SynchronizeDeviceSystemTime();
		LWReturnCode HasSecurityAbility(bool& enable);
		LWReturnCode GetSecurityEnable(bool& enable);
		LWReturnCode SetSecurityCalibrationParams(const int32_t* array, int32_t size);
		LWReturnCode SecurityCancelCalibration();
		LWReturnCode GetSecurityCalibrationEnable(bool& enable);
		LWReturnCode SetSecurityAxialAdjustment(int flag, float value);
		LWReturnCode ResetSecurityConfigure();
		LWReturnCode GetSecurityCalibrationMatrixParams(float* mat, int32_t* size);
		LWReturnCode GetSecurityZHSAEnable(bool& enable);
		LWReturnCode SetSecurityZHSAEnable(bool enable);
		LWReturnCode GetSecurityConfigFile(int type, const char* filename);
		LWReturnCode SetSecurityConfigFile(int type, const char* filename);
		LWReturnCode GetSecurityConfigFileToBuffer(int type, char* buffer, int32_t bufLen);
		LWReturnCode SetSecurityConfigFileFromBuffer(int type, const char* buffer, int32_t bufLen);
	#endif //LW_ZHONGRILONG_PROJ


	public:
		bool	isPro = false;
		bool	depthEnable = true;
		bool	ampEnable = false;
		bool	irEnable = true;
		bool	potEnable = false;
		bool	rgbEnable = false;
		bool	isReady = false;

		bool	dataSyncEnable = true;
		bool	rgbDistortionEnable = true;

		std::atomic<bool>	isAlign{ false };
		std::atomic<bool>	isR2DEnable{ false };
		std::atomic<bool>	isD2REnable{ false };
		std::atomic<bool>	hasRgbModule{ false };

		int32_t			tofWidth = TOF_MAX_PIX_COLS;
		int32_t			tofHeight = TOF_MAX_PIX_ROWS;
		int32_t			tofPixels = TOF_MAX_PIX_NUMBER;
		int32_t			rgbWidth = RGB_MAX_PIX_COLS;
		int32_t			rgbHeight = RGB_MAX_PIX_ROWS;
		int32_t			rgbPixels = RGB_MAX_PIX_NUMBER;
		uint32_t		timeout = 3000;
		int32_t			irGMMGain = 255;
		int32_t		    r2dRunCount = 0;
		int32_t		    d2rRunCount = 0;
		uint32_t		r2dFlag = 0;
		uint32_t		d2rFlag = 0;
		uint32_t		r2dArgRenew = 0;
		uint32_t		d2rArgRenew = 0;
		uint32_t		cal_mode = 0x00;

		SOCKET			broadcastSocket = INVALID_SOCKET;

		sockaddr_in		remoteAddr{};
		sockaddr_in		localAddr{};
		sockaddr_in		recvAddr{};

		LWDataRecvType	dataRecvType = LW_DEPTH_IR_RTY;

		LWSensorIntrinsicParam	tofInArg{};
		LWSensorIntrinsicParam	rgbInArg{};
		LWSensorExtrinsicParam	rgbOutArg{};
		LWIMUExtrinsicParam		imuOutArg{ 1,0,0, 0,1,0, 0,0,1 };

		std::string     _SN_;
		std::string     describe;
		std::string     deviceType;

		DataNode* tofNode;
		DataNode* rgbNode;
		DataNode* tofCutNode;
		DataNode* rgbCutNode;

		AutoMemoryManager pDepth{ TOF_MAX_PIX_NUMBER * 2 };
		AutoMemoryManager pAmp{ TOF_MAX_PIX_NUMBER * 2 };
		AutoMemoryManager pGra{ TOF_MAX_PIX_NUMBER };
		AutoMemoryManager pPot{ TOF_MAX_PIX_NUMBER * 12 };
		AutoMemoryManager pTPot{ RGB_MAX_PIX_NUMBER * 12 };
		AutoMemoryManager pRgb{ RGB_MAX_PIX_NUMBER * 3 };
		AutoMemoryManager pR2D{ TOF_MAX_PIX_NUMBER * 3 };

		std::mutex				socketMutex;
		std::condition_variable	socketNotify;
		std::mutex				r2dMutex;
		std::condition_variable r2dNotify;
		std::mutex				d2rMutex;
		std::condition_variable d2rNotify;

		cv::Mat         d2rOutputImg;
		cv::Mat			d2rInputImg;
		cv::Mat         ir2rOutputImg;
		cv::Mat			ir2rInputImg;


	private:
		void commandRecvThread();
		void rawDataRecvThread();
		void tofDataHandleThread();
		void rgbDataHandleThread();
		void dataSyncThread();
		void rgbToDepthThread(int threadID, int srcStartRow, int srcEndRow);
		void depthToRgbThread(int threadID, int srcStartRow, int srcEndRow);

		LWReturnCode ExecuteCommand(CommandSFrame& command);


	private:
		std::atomic<bool>	aliveEnable{ false };
		std::atomic<bool>	openEnable{ false };
		std::atomic<bool>	connectEnable{ false };
		std::atomic<bool>	tofReadEnable{ false };
		std::atomic<bool>	rgbReadEnable{ false };
		std::atomic<bool>	readEnable{ false };
		std::atomic<bool>	readyEnable{ false };
		std::atomic<bool>	isCommandReply{ false };

		std::atomic<uint16_t>	exeCommand{ 65535 };
		std::atomic<int32_t>	threadCounter{ 0 };

		CommandRFrame		commandRecvFrame;

		float               d2rScale = 2.5f;

		LWDeviceHandle      handle = 0;
		LWTriggerMode       triggerMode = LW_TRIGGER_ACTIVE;
		LWTimeStamp		    rgbTimeStamp{};
		LWTimeStamp		    tofTimeStamp{};

		DataNode* tofRecvNode;
		DataNode* rgbRecvNode;
		DataNode* tofSwapNode;
		DataNode* rgbSwapNode;
		DataNode* tofHandleNode;
		DataNode* rgbSrcNode;
		DataNode* rgbDstNode;

		LoopQueueNCP	tofDataReadQueue{ TOF_MAX_PIX_NUMBER * 15, LOOP_QUEUE_SIZE };
		LoopQueueNCP	rgbDataReadQueue{ RGB_MAX_PIX_NUMBER * 3, LOOP_QUEUE_SIZE };

		SOCKET			commandSocket	= INVALID_SOCKET;
		SOCKET			dataSocket		= INVALID_SOCKET;

		cv::Mat			rMatrixR2D{ cv::Size(3, 3), CV_32FC1 };
		cv::Mat			tMatrixR2D{ cv::Size(1, 3), CV_32FC1 };
		cv::Mat			rMatrixD2R{ cv::Size(3, 3), CV_32FC1 };
		cv::Mat			tMatrixD2R{ cv::Size(1, 3), CV_32FC1 };
		cv::Mat			rgbCalibMap1;//用于RGB畸变的表
		cv::Mat			rgbCalibMap2;//用于RGB畸变的表

		std::mutex				nrMutex;//用于API调用

		std::mutex				tofWriteMutex;
		std::condition_variable tofWriteNotify;
		std::mutex				rgbWriteMutex;
		std::condition_variable rgbWriteNotify;
		std::mutex				dataReadMutex;
		std::condition_variable dataReadNotify;
		std::mutex				syncDataMutex;
		std::condition_variable syncDataNotify;

	};

	/// @brief 用于全局变量的自动化管理。
	class AutoInitGlobalResources
	{
	public:
		AutoInitGlobalResources()
		{
			initEnable.store(false);
		}

		~AutoInitGlobalResources()
		{

		}

	public:
		bool	d2rTF = false;
		bool	r2dTF = false;

		std::atomic<bool>	initEnable;
		std::string			errorInfo;

		std::map<std::string, DeviceHandle*>	strDeviceMap;
		std::map<LWDeviceHandle, DeviceHandle*> numDeviceMap;

	} gGlobal;

}


ljhNS::DeviceHandle::DeviceHandle(LWDeviceHandle _handle): handle(_handle)
{
	aliveEnable.store(true);

	tofNode			= new DataNode{ TOF_MAX_PIX_NUMBER * 15 };
	rgbNode			= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofRecvNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 15 };
	rgbRecvNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofSwapNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 15 };
	rgbSwapNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofHandleNode	= new DataNode{ TOF_MAX_PIX_NUMBER * 15 };
	rgbSrcNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	rgbDstNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };
	tofCutNode		= new DataNode{ TOF_MAX_PIX_NUMBER * 15 };
	rgbCutNode		= new DataNode{ RGB_MAX_PIX_NUMBER * 3 };

	std::thread{ &DeviceHandle::commandRecvThread, this }.detach();
	std::thread{ &DeviceHandle::rawDataRecvThread, this }.detach();
	std::thread{ &DeviceHandle::tofDataHandleThread, this }.detach();
	std::thread{ &DeviceHandle::rgbDataHandleThread, this }.detach();
	std::thread{ &DeviceHandle::dataSyncThread, this }.detach();
}

ljhNS::DeviceHandle::~DeviceHandle()
{
	aliveEnable.store(false);
	connectEnable.store(false);
	isR2DEnable.store(false);
	isD2REnable.store(false);
	closeSocket(commandSocket);
	closeSocket(dataSocket);
	std::this_thread::sleep_for(std::chrono::milliseconds(20));

	r2dNotify.notify_all();
	d2rNotify.notify_all();
	socketNotify.notify_all();
	tofWriteNotify.notify_all();
	rgbWriteNotify.notify_all();
	dataReadNotify.notify_all();
	syncDataNotify.notify_all();
	std::this_thread::sleep_for(std::chrono::milliseconds(30));

	auto t = std::chrono::steady_clock::now();
	while (threadCounter.load() > 0)
	{
		if (timeout < std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t).count())
		{
			LOG_ERROR_OUT("<%s>, Thread exit timeout.", _SN_.c_str());
			break;
		}
	}

	delete tofNode;
	delete rgbNode;
	delete tofRecvNode;
	delete rgbRecvNode;
	delete tofSwapNode;
	delete rgbSwapNode;
	delete tofHandleNode;
	delete rgbSrcNode;
	delete rgbDstNode;
	delete tofCutNode;
	delete rgbCutNode;
}

/// @brief 用于接收命令发送后设备端传来的数据
void ljhNS::DeviceHandle::commandRecvThread()
{
	ThreadCounter		_tc_{ threadCounter };
	AutoMemoryManager	buf{ 65535 };

	decltype(recv(0, nullptr, 0, 0)) recv_len = 0;

	while (aliveEnable.load())
	{
		recv_len = recv(commandSocket, buf.data(), 65535, 0);
		if (recv_len > 0)
		{
			commandRecvFrame.push(buf.data(), recv_len);

			if (commandRecvFrame.isCommandHeader(exeCommand.load()))
			{
				if (commandRecvFrame.isCommand(exeCommand.load()))
				{
					isCommandReply.store(true);
					socketNotify.notify_all();
				}
			}
			else
			{
				commandRecvFrame.clear();
			}

			continue;
		}
		if (!aliveEnable.load()) break;

		std::unique_lock<std::mutex>	lock{ socketMutex };
		if (connectEnable.load())
		{
			connectEnable.store(false);
			auto einfo = getNetworkLastError();

			if (networkAbnormalCallback != nullptr)
				networkAbnormalCallback(handle, einfo.c_str(), pUserData1);

			LOG_ERROR_OUT("<%s>, The network connection has been disconnected.---%s", _SN_.c_str(), einfo.c_str());
		}
		socketNotify.wait(lock, [this] {	return  connectEnable.load() || !aliveEnable.load(); });
	}
}

void ljhNS::DeviceHandle::rawDataRecvThread()
{
	ThreadCounter _tc_{ threadCounter };

	int					k = 0;
	uint32_t			magic = 2774182485U;
	uint32_t			serial{};
	LWTimeStamp			time{};
	AutoMemoryManager	recvBuf{ 15000000 };

	auto*	recv_buf	= recvBuf.data();
	auto*	u_recv_buf	= recvBuf.u_data();

    decltype(recv(0, nullptr, 0, 0)) recv_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) read_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) frame_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) data_len	= 0;
	decltype(recv(0, nullptr, 0, 0)) head_len	= 56;

	bool	determine	= true;
	while (aliveEnable.load())
	{
		data_len	= 0;
		determine	= true;
		
		while (true)
		{
			recv_len = recv(dataSocket, recv_buf + data_len, 65536, 0);

			if (recv_len > 0)
			{
				if (determine)
				{
					// 魔术判定（即一帧的前4个字节：0x55 0xaa 0x5a 0xa5）
					if ((magic != *(uint32_t*)recv_buf))
					{
						if (data_len != 0) data_len = 0;
						continue;
					}
					// 获取帧数据基本信息
					time.tv_sec			= *(uint32_t*)(recv_buf + 6);
					time.tv_usec		= *(uint32_t*)(recv_buf + 12);
					serial				= *(uint32_t*)(recv_buf + 32);
					// 帧数据长度
					read_len = ntohl(*(uint32_t*)(recv_buf + 28));
					// 帧头长度
					head_len = (u_recv_buf[4] > 0x03) ? ntohs(*(uint16_t*)(u_recv_buf + 53)) : 56;
					// 帧长度(帧头+数据长度+帧尾)
					frame_len = read_len + head_len + 4;
					
					// 判定数据解析是否正常（正常情况下一帧的数据量大小不可能超过此数值），用于规避数据混乱造成的内存读写错误。
					if (frame_len > 6000000)
					{
						if (data_len != 0) data_len = 0;
						continue;
					}
					// 判断数据接收类型
					if (dataRecvType != u_recv_buf[52])
					{
						dataRecvType	= LWDataRecvType(u_recv_buf[52]);

						rgbEnable		= (u_recv_buf[52] == 0x06 || u_recv_buf[52] == 0x09 || u_recv_buf[52] == 0x05 || u_recv_buf[52] == 0x0C);
						depthEnable		= (u_recv_buf[52] == 0x07 || u_recv_buf[52] == 0x09 || u_recv_buf[52] == 0x05 || u_recv_buf[52] == 0x0B || u_recv_buf[52] == 0x0C);
						irEnable		= (u_recv_buf[52] == 0x04 || u_recv_buf[52] == 0x07 || u_recv_buf[52] == 0x08 || u_recv_buf[52] == 0x05 || u_recv_buf[52] == 0x0B || u_recv_buf[52] == 0x0C);
						ampEnable		= (u_recv_buf[52] == 0x05);
						potEnable		= (u_recv_buf[52] == 0x08 || u_recv_buf[52] == 0x0B);

						isAlign.store(u_recv_buf[52] == 0x05 || u_recv_buf[52] == 0x09 || u_recv_buf[52] == 0x0C);
					}

					determine = false;
				}
				
				data_len += recv_len;
				if (data_len < frame_len) continue;

				// 数据类型处理
				if (u_recv_buf[5] == 0x06)
				{
					// RGB数据
					memcpy(rgbRecvNode->data(), recv_buf + head_len, read_len);
					rgbRecvNode->serial = serial;
					rgbRecvNode->time = time;
					rgbRecvNode->size = read_len;
					rgbRecvNode->rgbTFormat = LWRgbTransferFormat(u_recv_buf[25]);

					// 数据同步
					rgbWriteMutex.lock();
					std::swap(rgbRecvNode, rgbSwapNode);
					rgbReadEnable.store(true);
					// *由于只有一个线程竞争，所以可先解锁再通知，将延时最小化。
					rgbWriteMutex.unlock();
					rgbWriteNotify.notify_one();
				}
				else
				{
					// TOF数据
					memcpy(tofRecvNode->data(), recv_buf + head_len, read_len);
					// IMU数据
					memcpy(&tofRecvNode->variant->IMUModule.imuData, recv_buf + 56, 36);
					// 拓展数据
					if (u_recv_buf[4] > 0x04 && head_len > 92)
					{
						//托盘数据
						tofRecvNode->variant->PalletModule.identifyNumber = (u_recv_buf[92] < 20) ? u_recv_buf[92] : 20;
						tofRecvNode->variant->PalletModule.errorCode = LWVariant::PalletStruct::ErrorCode(recv_buf[93]);
						tofRecvNode->variant->PalletModule.identifyEnable = recv_buf[94];
						if (u_recv_buf[92] > 0)
						{
							memcpy(&tofRecvNode->variant->PalletModule.poseData, recv_buf + 108, tofRecvNode->variant->PalletModule.identifyNumber * 52);
						}
						
					#ifdef LW_ZHONGRILONG_PROJ
						// 数据偏移统计
						k = (u_recv_buf[92] < 2) ? 160 : (52 * u_recv_buf[92] + 108);
						if (head_len > k)
						{
							// 中日龙---安防数据
							// 使能开关
							tofRecvNode->variant->SecurityDetection.enable = u_recv_buf[k];
							++k;

							// 错误码
							tofRecvNode->variant->SecurityDetection.error = u_recv_buf[k];

							// 错误码长度+保留字段长度
							k += 29;

							// 3D检测区域
							memcpy(&tofRecvNode->variant->SecurityDetection.detectionArea, recv_buf + k, 320);
							k += 320;

							// 报警区域
							memcpy(&tofRecvNode->variant->SecurityDetection.alarmAreaNumber, recv_buf + k, 18);
							k += 18;

							// 区域有效点
							memcpy(&tofRecvNode->variant->SecurityDetection.pots, recv_buf + k, 32);
							k += 32;

							// 视锥信息
							memcpy(&tofRecvNode->variant->SecurityDetection.pyramid, recv_buf + k, 60);
							k += 60;

							// 区域属性
							memcpy(&tofRecvNode->variant->SecurityDetection.areaAttribute, recv_buf + k, 9);
						}
					#endif //LW_ZHONGRILONG_PROJ
						
					}
					else
					{
						tofRecvNode->variant->PalletModule.identifyEnable = false;
					}

					tofRecvNode->temperature.laser1 = float(u_recv_buf[21] << 8 | u_recv_buf[22]) / 10.0f;
					tofRecvNode->temperature.laser2 = float(u_recv_buf[23] << 8 | u_recv_buf[24]) / 10.0f;
					tofRecvNode->temperature.chip = u_recv_buf[25];
					tofRecvNode->serial = serial;
					tofRecvNode->time = time;
					tofRecvNode->size = read_len;

					tofWriteMutex.lock();
					std::swap(tofRecvNode, tofSwapNode);
					tofReadEnable.store(true);
					// 由于只有一个线程竞争，所以可先解锁再通知，将延时最小化。
					tofWriteMutex.unlock();
					tofWriteNotify.notify_one();
				}

				data_len -= frame_len;
				if (data_len > 0)
				{
					memcpy(recv_buf, recv_buf + frame_len, data_len);
				}
				determine = true;

				continue;
			}
			
			break;
		}

		if (!aliveEnable.load()) break;

		std::unique_lock<std::mutex>	lock{ socketMutex };
		if (connectEnable.load())
		{
			connectEnable.store(false);
			auto einfo = getNetworkLastError();

			if (networkAbnormalCallback != nullptr)
				networkAbnormalCallback(handle, einfo.c_str(), pUserData1);

			LOG_ERROR_OUT("<%s>, The network connection has been disconnected.---%s", _SN_.c_str(), einfo.c_str());
		}

		tofReadEnable.store(false);
		rgbReadEnable.store(false);
		tofTimeStamp.tv_sec = 0;
		tofTimeStamp.tv_usec = 0;
		rgbTimeStamp.tv_sec = 0;
		rgbTimeStamp.tv_usec = 0;
		socketNotify.wait(lock, [this]{	return  connectEnable.load() || !aliveEnable.load(); });

	}
}

void ljhNS::DeviceHandle::tofDataHandleThread()
{
	ThreadCounter _tc_{ threadCounter };

	while (true)
	{
		{// 此块作用域的作用是为了以最快的时间释放锁，以最大化降低写线程的等待时延。
			std::unique_lock<std::mutex> lock{ tofWriteMutex };
			tofWriteNotify.wait(lock, [this] { return tofReadEnable.load() || !aliveEnable.load(); });
            if (!aliveEnable.load())  break;

			tofReadEnable.store(false);
			std::swap(tofHandleNode, tofSwapNode);
		}

		//
		dataReadMutex.lock();
		readEnable.store(true);
		tofDataReadQueue.enqueue(tofHandleNode);
		dataReadMutex.unlock();
		dataReadNotify.notify_one();
	}

}

void ljhNS::DeviceHandle::rgbDataHandleThread()
{
	ThreadCounter _tc_{ threadCounter };

	while (true)
	{
		{// 此块作用域的作用是为了以最快的时间释放锁，以最大化降低写线程的等待时延。
			std::unique_lock<std::mutex> lock{ rgbWriteMutex };
			rgbWriteNotify.wait(lock, [this] {  return rgbReadEnable.load() || !aliveEnable.load(); });
			if (!aliveEnable.load()) break;

			rgbReadEnable.store(false);
			std::swap(rgbSrcNode, rgbSwapNode);
		}

		// YUV转RGB格式
		cv::Mat img(rgbHeight, rgbWidth, CV_8UC3, rgbDstNode->data());
		if (rgbSrcNode->rgbTFormat == LWRgbTransferFormat::LW_YVU420_NV12)
		{
			// 格式转换
			cv::cvtColor(
				cv::Mat(int32_t(rgbHeight * 1.5), rgbWidth, CV_8UC1, rgbSrcNode->data()),
				img,
				cv::COLOR_YUV2RGB_NV12
			);
			// 畸变校准
			if (rgbDistortionEnable) cv::remap(img, img, rgbCalibMap1, rgbCalibMap2, cv::INTER_LINEAR);
		}
		else if (rgbSrcNode->rgbTFormat == LWRgbTransferFormat::LW_BGR_565)
		{
			// 格式转换
			cv::cvtColor(cv::Mat(1200, 1600, CV_8UC2, rgbSrcNode->data()), img, cv::COLOR_BGR5652BGR);
			// 畸变校准
			if (rgbDistortionEnable) cv::remap(img, img, rgbCalibMap1, rgbCalibMap2, cv::INTER_NEAREST);
		}
		else if(rgbSrcNode->rgbTFormat == LWRgbTransferFormat::LW_BGR_888)
		{
			// 畸变校准
			if (rgbDistortionEnable)
			{
				cv::remap(
					cv::Mat(rgbHeight, rgbWidth, CV_8UC3, rgbSrcNode->data()),
					img, 
					rgbCalibMap1, 
					rgbCalibMap2, 
					cv::INTER_NEAREST
				);
			}
			else
			{
				memcpy(rgbDstNode->data(), rgbSrcNode->data(), rgbSrcNode->size);
			}
		}

		// 
		rgbDstNode->serial = rgbSrcNode->serial;
		rgbDstNode->time = rgbSrcNode->time;
		rgbDstNode->size = rgbWidth * rgbHeight * 3;

		dataReadMutex.lock();
		readEnable.store(true);
		rgbDataReadQueue.enqueue(rgbDstNode);
		dataReadMutex.unlock();
		dataReadNotify.notify_one();
		
	}
}

void ljhNS::DeviceHandle::dataSyncThread()
{
	ThreadCounter _tc_{ threadCounter };

	while (true)
	{
		std::unique_lock<std::mutex> tof_lock{ dataReadMutex };
		dataReadNotify.wait(tof_lock, [this] { return readEnable.load() || !aliveEnable.load(); });
		if (!aliveEnable.load()) break;

		readEnable.store(false);

		// 获取TOF和RGB最新数据
		auto tof_node = tofDataReadQueue.latestNode();
		auto rgb_node = rgbDataReadQueue.latestNode();

		// 对齐判定
		if (dataSyncEnable && isAlign.load())
		{
			// 帧数据的序列号必须相等
			if (tof_node->serial != rgb_node->serial)
			{
				if (tof_node->serial < rgb_node->serial)
				{
					rgb_node = rgbDataReadQueue.find(tof_node->serial);
					if (rgb_node == nullptr) continue;
				}
				else
				{
					tof_node = tofDataReadQueue.find(rgb_node->serial);
					if (tof_node == nullptr) continue;
				}
			}
			
			// 时间戳必须比前一帧的大，以防止帧回跳
			if (tofTimeStamp.tv_sec > tof_node->time.tv_sec) continue;
			if (tofTimeStamp.tv_sec < tof_node->time.tv_sec || tofTimeStamp.tv_usec < tof_node->time.tv_usec)
			{
				tofTimeStamp = tof_node->time;

				syncDataMutex.lock();
				tofDataReadQueue.dequeue(tof_node->mark, tofNode);
				rgbDataReadQueue.dequeue(rgb_node->mark, rgbNode);
				readyEnable.store(true);
				syncDataMutex.unlock();
				syncDataNotify.notify_one();

				if (frameReadyCallback != nullptr) frameReadyCallback(handle, pUserData2);

				continue;
			}
		}
		else
		{
			syncDataMutex.lock();
			if (tof_node->update)
			{
				// 时间戳必须比前一帧的大，防止帧回跳
                if ((tof_node->time.tv_sec > tofTimeStamp.tv_sec) 
					|| ((tof_node->time.tv_sec == tofTimeStamp.tv_sec) && (tof_node->time.tv_usec > tofTimeStamp.tv_usec)))
                {
					tofTimeStamp = tof_node->time;
					tofDataReadQueue.dequeue(tof_node->mark, tofNode);
                }
			}
			else {
				tofNode->update = false;
			}
			
			if(rgb_node->update) 
			{
				// 时间戳必须比前一帧的大，防止帧回跳
                if ((rgb_node->time.tv_sec > rgbTimeStamp.tv_sec)
                    || ((rgb_node->time.tv_sec == rgbTimeStamp.tv_sec) && (rgb_node->time.tv_usec > rgbTimeStamp.tv_usec)))
                {
					rgbTimeStamp = rgb_node->time;
					rgbDataReadQueue.dequeue(rgb_node->mark, rgbNode);
                }
			}
			else {
				rgbNode->update = false;
			}

			readyEnable.store(true);
			syncDataMutex.unlock();
			syncDataNotify.notify_one();

			if (frameReadyCallback != nullptr) frameReadyCallback(handle, pUserData2);

			continue;
		}
	}
}

/// @brief RGB数据到深度的映射。注：由算法组给出，由算法组维护该代码。
/// @param threadID 
/// @param srcStartRow 
/// @param srcEndRow 
void ljhNS::DeviceHandle::rgbToDepthThread(int threadID, int srcStartRow, int srcEndRow)
{
	ThreadCounter _tc_{ threadCounter };

	int voxelSize = 4;
	int extend_size = 8;
	int startRow = 0;
	int endRow = 0;
	int shift_ptr = threadID != 0 ? extend_size * tofWidth : 0;

	if (threadID == 0)
	{
		endRow = srcEndRow + extend_size;
	}
	else if (threadID == THREAD_POOL_SIZE - 1)
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow;
	}
	else
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow + extend_size;
	}

	int xmin = 0;
	int ymin = 0;
	int xmax = rgbWidth;
	int ymax = rgbHeight;
	int n = (endRow - startRow) * tofWidth;    //tof分段对应像素点数量

	cv::Mat T_rpt = cv::repeat(tMatrixR2D, 1, n);
	cv::Mat tempXYZ(3, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat xPixel(1, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat yPixel(1, n, CV_32FC1, cv::Scalar::all(0));
	auto* ptrX = (float*)(xPixel.data);
	auto* ptrY = (float*)(yPixel.data);

	uint32_t idFlag = 1U << threadID;   // 线程标识
	while (isR2DEnable.load())
	{
		{// 线程运行条件判定
			std::unique_lock<std::mutex> lock{ r2dMutex };
			r2dNotify.wait(lock, [&idFlag, this] { return (r2dFlag & idFlag) > 0 || !isR2DEnable.load(); });
			if (!isR2DEnable.load()) break;
			if (r2dArgRenew & idFlag)
			{
				xmax = rgbWidth;
				ymax = rgbHeight;
				T_rpt = cv::repeat(tMatrixR2D, 1, n);
				r2dArgRenew &= ~idFlag;
			}
		}

		// 先计算tof相机点云
		auto tXYZ1 = (float*)tempXYZ.row(0).data;
		auto tXYZ2 = (float*)tempXYZ.row(1).data;
		auto tXYZ3 = (float*)tempXYZ.row(2).data;
		auto tRow = float(startRow) - tofInArg.cy;
		auto tCol = -tofInArg.cx;
		auto ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		for (int i = startRow; i < endRow; ++i)
		{
			tCol = -tofInArg.cx;
			for (int j = 0; j < tofWidth; ++j, tCol += 1.0f, ++ptrDepth, ++tXYZ1, ++tXYZ2, ++tXYZ3)
			{
				if (*ptrDepth < PCD_MAX_VALUE)
				{
					*tXYZ1 = tCol / tofInArg.fx * float(*ptrDepth);
					*tXYZ2 = tRow / tofInArg.fy * float(*ptrDepth);
					*tXYZ3 = *ptrDepth;
				}
			}
			tRow += 1.0f;
		}

		// 再转换至RGB坐标系
		tempXYZ = rMatrixR2D * tempXYZ + T_rpt;
		cv::divide(tempXYZ.row(0), tempXYZ.row(2), xPixel);
		cv::divide(tempXYZ.row(1), tempXYZ.row(2), yPixel);

		// 20240727-修改后，把均值计算放至后续判定条件里
		// 像素占据情况统计
		cv::Mat occupMap = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		cv::Mat occupMapZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_32F, cv::Scalar::all(0));
		ptrX = (float*)(xPixel.data);
		ptrY = (float*)(yPixel.data);
		auto ptrZ = (float*)(tempXYZ.row(2).data);
		for (int i = 0; i < xPixel.cols; i++, ++ptrX, ++ptrY, ++ptrZ)
		{
			int idxX = std::floor((*ptrX - float(xmin)) / float(voxelSize));
			int idxY = std::floor((*ptrY - float(ymin)) / float(voxelSize));
			if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows && *ptrZ>0 && *ptrZ < PCD_MAX_VALUE)
				//if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
			{
				occupMap.at<uint16_t>(idxY, idxX) = occupMap.at<uint16_t>(idxY, idxX) + 1;
				occupMapZ.at<float>(idxY, idxX) += (*ptrZ);
			}
		}

		//创建结果储存
		ptrX = (float*)(xPixel.data) + shift_ptr;
		ptrY = (float*)(yPixel.data) + shift_ptr;
		ptrZ = (float*)(tempXYZ.row(2).data) + shift_ptr;
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * srcStartRow;
		auto ptrR2D = (LWRGB888Pixel*)pR2D.data() + tofWidth * srcStartRow;
		auto ptrRGB = (LWRGB888Pixel*)rgbCutNode->data();
		for (int row = srcStartRow; row < srcEndRow; ++row)
		{
			for (int col = 0; col < tofWidth; ++col, ++ptrDepth, ++ptrX, ++ptrY, ++ptrZ, ++ptrR2D)
			{
				if (*ptrDepth < PCD_MAX_VALUE && *ptrDepth>0)
				{
					auto x_ = int(*ptrX);
					auto y_ = int(*ptrY);

					int idxX = std::floor(float(x_ - xmin) / float(voxelSize));
					int idxY = std::floor(float(y_ - ymin) / float(voxelSize));
					// 20240727-根据数据精度设置低于均值+σ以下的数据才是RGB坐标系下的目标物，否则就是其投影区冗余数据
					if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
					{
						float tmp_average_Z = occupMapZ.at<float>(idxY, idxX) / float(occupMap.at<uint16_t>(idxY, idxX));
						if (x_ > -1 && y_ > -1 && x_ < rgbWidth && y_ < rgbHeight && ((*ptrZ) <= (tmp_average_Z + RGBD_PROJ_REDUNDANCY)))
						{
							int index = y_ * rgbWidth + x_;
							*ptrR2D = ptrRGB[index];

							continue;
						}
					}
				}
			}
		}

		r2dMutex.lock();
		r2dFlag &= ~idFlag;
		if (--r2dRunCount == 0)
		{// 所有线程跑完一帧 
			r2dNotify.notify_all();
		}
		r2dMutex.unlock();
	}
}

/// @brief 深度数据到RGB的映射。注：由算法组给出，由算法组维护该代码。
/// @param threadID 
/// @param srcStartRow 
/// @param srcEndRow 
void ljhNS::DeviceHandle::depthToRgbThread(int threadID, int srcStartRow, int srcEndRow)
{
	ThreadCounter _tc_{ threadCounter };

	int voxelSize = 2;
	int extend_size = 8;
	int startRow = 0;
	int endRow = 0;
	int shift_ptr = threadID != 0 ? extend_size * tofWidth : 0;
	
	uint16_t* ptrDepth;
	uint16_t* ptrDst;
	uint8_t* ptrIR;
	uint8_t* ptrIRDst;

	if (threadID == 0)
	{
		endRow = srcEndRow + extend_size;
	}
	else if (threadID == THREAD_POOL_SIZE - 1)
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow;
	}
	else
	{
		startRow = srcStartRow - extend_size;
		endRow = srcEndRow + extend_size;
	}

	int xmin = 0;
	int ymin = 0;
	int xmax = int(float(rgbWidth) / d2rScale);
	int ymax = int(float(rgbHeight) / d2rScale);
	int n = (endRow - startRow) * tofWidth;    //tof分段对应像素点数量

	cv::Mat T_rpt = cv::repeat(tMatrixD2R, 1, n);
	cv::Mat tempXYZ(3, n, CV_32FC1, cv::Scalar::all(0));
	cv::Mat xPixel(1, n, CV_32SC1, cv::Scalar::all(0));
	cv::Mat yPixel(1, n, CV_32SC1, cv::Scalar::all(0));
	auto* ptrX = (float*)(xPixel.data);
	auto* ptrY = (float*)(yPixel.data);

	uint32_t idFlag = 1U << threadID;   // 线程标识
	while (isD2REnable.load())
	{
		{// 线程运行条件判定
			std::unique_lock<std::mutex> lock{ d2rMutex };
			d2rNotify.wait(lock, [&idFlag, this] { return (d2rFlag & idFlag) > 0 || !isD2REnable.load(); });
			if (!isD2REnable.load()) break;
			if (d2rArgRenew & idFlag)
			{
				xmax = int(float(rgbWidth) / d2rScale);
				ymax = int(float(rgbHeight) / d2rScale);
				T_rpt = cv::repeat(tMatrixD2R, 1, n);
				d2rArgRenew &= ~idFlag;
			}
		}

		// 对 tempXYZ 变量赋初值
		auto tXYZ1 = (float*)tempXYZ.row(0).data;
		auto tXYZ2 = (float*)tempXYZ.row(1).data;
		auto tXYZ3 = (float*)tempXYZ.row(2).data;
		auto tRow = float(startRow) - tofInArg.cy;
		auto tCol = -tofInArg.cx;
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		for (int i = startRow; i < endRow; ++i)
		{
			tCol = -tofInArg.cx;
			for (int j = 0; j < tofWidth; ++j, ++ptrDepth, ++tXYZ1, ++tXYZ2, ++tXYZ3)
			{
				if (*ptrDepth < PCD_MAX_VALUE && *ptrDepth>0)
				{
					*tXYZ1 = tCol / tofInArg.fx * float(*ptrDepth);
					*tXYZ2 = tRow / tofInArg.fy * float(*ptrDepth);
					*tXYZ3 = *ptrDepth;
				}
				tCol += 1.0f;
			}
			tRow += 1.0f;
		}

		// 对 tempXYZ 变量的值做相应变换
		tempXYZ = rMatrixD2R * tempXYZ + T_rpt;
		cv::divide(tempXYZ.row(0), tempXYZ.row(2), xPixel);
		cv::divide(tempXYZ.row(1), tempXYZ.row(2), yPixel);

		// 20240727-修改为累加后除以计数
		// 像素占据情况统计
		cv::Mat occupMap = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		cv::Mat occupMapZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_32F, cv::Scalar::all(0));
		cv::Mat occupMinZ = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_16U, cv::Scalar::all(0));
		cv::Mat occupMinIR = cv::Mat(std::ceil(float(ymax - ymin) / float(voxelSize)), std::ceil(float(xmax - xmin) / float(voxelSize)), CV_8U, cv::Scalar::all(0));
		ptrX = (float*)(xPixel.data);
		ptrY = (float*)(yPixel.data);
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * startRow;
		ptrIR = (uint8_t*)(tofCutNode->data() + tofPixels * 2) + tofWidth * startRow;
		auto ptrZ = (float*)(tempXYZ.row(2).data);
		for (int i = 0; i < xPixel.cols; i++, ++ptrX, ++ptrY, ++ptrZ, ++ptrDepth, ++ptrIR)
		{
			int idxX = std::floor((*ptrX - float(xmin)) / float(voxelSize));
			int idxY = std::floor((*ptrY - float(ymin)) / float(voxelSize));
			if (idxX >= 0 && idxY >= 0 && idxX < occupMinZ.cols && idxY < occupMinZ.rows && *ptrZ>0 && *ptrZ < PCD_MAX_VALUE)
			{
				if ((occupMap.at<uint16_t>(idxY, idxX) == 0) || (*ptrDepth < occupMinZ.at<uint16_t>(idxY, idxX)))
				{
					occupMinZ.at<uint16_t>(idxY, idxX) = *ptrDepth;
					occupMinIR.at<uint8_t>(idxY, idxX) = *ptrIR;
				}
				occupMap.at<uint16_t>(idxY, idxX) += 1;
				occupMapZ.at<float>(idxY, idxX) += *ptrZ;
			}
		}

		//创建结果储存
		ptrX = (float*)(xPixel.data) + shift_ptr;
		ptrY = (float*)(yPixel.data) + shift_ptr;
		ptrZ = (float*)(tempXYZ.row(2).data) + shift_ptr;
		ptrDepth = (uint16_t*)(tofCutNode->data()) + tofWidth * srcStartRow;
		ptrDst = (uint16_t*)d2rInputImg.data;
		ptrIR = (uint8_t*)(tofCutNode->data() + tofPixels * 2) + tofWidth * srcStartRow;
		ptrIRDst = (uint8_t*)ir2rInputImg.data;
		for (int row = srcStartRow; row < srcEndRow; ++row)
		{
			for (int col = 0; col < tofWidth; ++col, ++ptrDepth, ++ptrIR, ++ptrX, ++ptrY)
			{
				if (*ptrDepth < PCD_MAX_VALUE)
				{
					auto x_ = int(*ptrX);
					auto y_ = int(*ptrY);

					int idxX = std::floor(float(x_ - xmin) / float(voxelSize));
					int idxY = std::floor(float(y_ - ymin) / float(voxelSize));
					// 根据数据精度设置低于均值+σ以下的数据才是RGB坐标系下的目标物，否则就是其投影区冗余数据
					if (idxX >= 0 && idxY >= 0 && idxX < occupMap.cols && idxY < occupMap.rows)
					{
						auto tmp_average_Z = uint16_t(occupMapZ.at<float>(idxY, idxX) / float(occupMap.at<uint16_t>(idxY, idxX)));
						if (x_ > -1 && y_ > -1 && x_ < xmax && y_ < ymax)
						{
							int index = y_ * xmax + x_;
							ptrDst[index] = *ptrDepth;
							ptrIRDst[index] = *ptrIR;
							if (*ptrDepth > (tmp_average_Z + RGBD_PROJ_REDUNDANCY))
							{
								ptrDst[index] = occupMinZ.at<uint16_t>(idxY, idxX);
								ptrIRDst[index] = occupMinIR.at<uint8_t>(idxY, idxX);
							}
							continue;
						}
					}
				}
			}
		}

		d2rMutex.lock();
		d2rFlag &= ~idFlag;
		if (--d2rRunCount == 0)
		{// 所有线程跑完一帧 
			// fill empty area
			ptrDst = (uint16_t*)d2rInputImg.data;
			ptrIRDst = (uint8_t*)ir2rInputImg.data;
			for (int row = 1, index; row < tofHeight - 1; ++row)
			{
				for (int col = 1; col < tofWidth - 1; ++col)
				{
					index = row * tofWidth + col;
					if (*(ptrDst + index) == 0)
					{
						int cnt = 0;
						int tmpIndex = 0;
						int tmpVal = 0;
						int tmpIRVal = 0;
						for (int locR = row - 1; locR <= row + 1; locR++)
						{
							for (int locC = col - 1; locC <= col + 1; locC++)
							{
								tmpIndex = locR * tofWidth + locC;
								if (*(ptrDst + tmpIndex) > 0)
								{
									cnt++;
									tmpVal += ptrDst[tmpIndex];
									tmpIRVal += ptrIRDst[tmpIndex];
								}
							}
						}
						if (cnt > 6) // 当该像素邻域存在超20个点有效值时，将这些点的均值赋值给他
						{
							ptrDst[index] = tmpVal / cnt;
							ptrIRDst[index] = tmpIRVal / cnt;
						}
					}

				}
			}

			// 缩放图像大小
			cv::resize(d2rInputImg, d2rOutputImg, cv::Size(rgbWidth, rgbHeight), 0, 0, cv::INTER_NEAREST);
			cv::resize(ir2rInputImg, ir2rOutputImg, cv::Size(rgbWidth, rgbHeight), 0, 0, cv::INTER_NEAREST);

			d2rNotify.notify_all();
		}
		d2rMutex.unlock();
	}
}

LWReturnCode ljhNS::DeviceHandle::ExecuteCommand(CommandSFrame& command)
{
	std::unique_lock<std::mutex> lock{ socketMutex };

	isCommandReply.store(false);
	exeCommand.store(command.getCommand());
	commandRecvFrame.clear();
	if (send(commandSocket, command.data(), command.size(), MSG_NOSIGNAL) != SOCKET_ERROR)
	{
		if (socketNotify.wait_for(lock, std::chrono::milliseconds{ timeout }, [this] { return isCommandReply.load() || !connectEnable.load() || !aliveEnable.load(); }))
		{
			if (isCommandReply.load())
			{
				return  (commandRecvFrame.getCommandType() < 0x03) ? LW_RETURN_OK : LWReturnCode(commandRecvFrame.getCommandType());
			}
		}
		else
			return LW_RETURN_TIMEOUT;
	}

	gGlobal.errorInfo = getNetworkLastError();
	return LW_RETURN_NETWORK_ERROR;
}

LWReturnCode ljhNS::DeviceHandle::OpenDevice()
{
	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if(!connectEnable.load())
    {
		// 清理连接
		closeSocket(dataSocket);
		closeSocket(commandSocket);

		// 休眠，以规避设备占用问题
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// 探测设备占用情况
		SOCKET _socket = gSocketMap[localAddr.sin_addr.s_addr];
		setNetworkTimeout(_socket, timeout);
		CommandSFrame command{ C_Discovery };
		if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr*)&remoteAddr, sizeof(remoteAddr)) != SOCKET_ERROR)
			&& (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) != SOCKET_ERROR))
		{
			if (!command.isCommand(C_Discovery)) return LW_RETURN_COMMAND_ERROR;
			auto lik = ntohl(*((uint32_t*)(command.getArgField() + 4)));
			if (lik != 0)
			{
				char buffer[128];
				auto ipPtr = (unsigned char*)&lik;
				snprintf(buffer, 128, "The device is already occupied.(%u.%u.%u.%u)", ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
				gGlobal.errorInfo = std::string(buffer);
				LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
				return LW_RETURN_CUSTOM_ERROR;
			}
		}
		else
		{
			gGlobal.errorInfo = getNetworkLastError();
			LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_NETWORK_ERROR;
		}

		// 建立连接
        commandSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		int bOpt = true;
        if ((setsockopt(commandSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) == SOCKET_ERROR)
			|| (bind(commandSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR)
			|| !connectToServer(commandSocket, remoteAddr, timeout))
        {
			auto error_str = getNetworkLastError();
			gGlobal.errorInfo = describe.empty() ? (error_str.empty() ? "Control socket connection timeout!" : error_str) : describe;
			LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_CUSTOM_ERROR;
        }

        dataSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if ((setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) == SOCKET_ERROR)
			|| (bind(dataSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR)
			|| !connectToServer(dataSocket, recvAddr, timeout))
        {
            auto error_str = getNetworkLastError();
			gGlobal.errorInfo = error_str.empty() ? "Data socket connection timeout!" : error_str;
			LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
            return LW_RETURN_CUSTOM_ERROR;
        }

        connectEnable.store(true);
        socketNotify.notify_all();
    }
	ProvisionalAuthority val{ openEnable };

	// 资源初始化
    auto RC = GetResolution(LWSensorType::LW_RGB_SENSOR, rgbWidth, rgbHeight);
    if (hasRgbModule.load())
    {
		rgbPixels = rgbWidth * rgbHeight;

        RC = GetIntrinsicParam(LWSensorType::LW_RGB_SENSOR, rgbInArg);
        if (RC != LW_RETURN_OK) return RC;

        RC = GetExtrinsicParam(LWSensorType::LW_RGB_SENSOR, rgbOutArg);
        if (RC != LW_RETURN_OK) return RC;

        auto ptr1 = (float*)&rgbOutArg;
        auto ptr2 = (float*)rMatrixR2D.data;
        auto ptr3 = (float*)tMatrixR2D.data;
        auto ptr4 = (float*)rMatrixD2R.data;
        auto ptr5 = (float*)tMatrixD2R.data;
        for (int i = 0; i < 9; i++, ++ptr1)
        {
            *ptr2++ = *ptr1;
            *ptr4++ = *ptr1;
        }
        for (int i = 0; i < 3; i++, ++ptr1)
        {
            *ptr3++ = *ptr1;
            *ptr5++ = *ptr1;
        }

		d2rArgRenew = 0xFF;
		r2dArgRenew = 0xFF;
		algNS::createRGBCameraCalibrationTable(rgbInArg, d2rScale, cv::Size(rgbWidth, rgbHeight), rgbCalibMap1, rgbCalibMap2, rMatrixR2D, tMatrixR2D, rMatrixD2R, tMatrixD2R);
	}

	RC = GetResolution(LWSensorType::LW_TOF_SENSOR, tofWidth, tofHeight);
	if (RC != LW_RETURN_OK) return RC;
	tofPixels = tofWidth * tofHeight;
	d2rInputImg.create(cv::Size(tofWidth, tofHeight), CV_16UC1);
	ir2rInputImg.create(cv::Size(tofWidth, tofHeight), CV_8UC1);

    RC = GetTriggerMode(triggerMode);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetIntrinsicParam(LWSensorType::LW_TOF_SENSOR, tofInArg);
    if (RC != LW_RETURN_OK) return RC;

    RC = GetIRGMMGain(irGMMGain);
    if (RC != LW_RETURN_OK) return RC;

	LWIMUData temp;
	RC = GetIMUData(temp);
	if (RC != LW_RETURN_OK) return RC;

	RC = GetIMUExtrinsicParam(imuOutArg);
	if (RC != LW_RETURN_OK) return RC;

	val.authorize();

    return  LW_RETURN_OK;
}

LWReturnCode ljhNS::DeviceHandle::CloseDevice()
{
	LOG_INFO_OUT("<%s>", _SN_.c_str());

	openEnable.store(false);

	if (connectEnable.load())
	{
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return LW_RETURN_OK;
}

LWReturnCode ljhNS::DeviceHandle::ReconnectDevice(uint32_t t)
{
	LOG_INFO_OUT("<%s>, timeout: %u", _SN_.c_str(), t);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	if (connectEnable.load()) return LW_RETURN_OK;

	// 清理连接
	closeSocket(dataSocket);
	closeSocket(commandSocket);

	// 休眠，以规避设备占用问题
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// 检测是否被占用
	CommandSFrame command{ C_Discovery };
	SOCKET _socket = gSocketMap[localAddr.sin_addr.s_addr];
	setNetworkTimeout(_socket, t);
	if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr*)&remoteAddr, sizeof(remoteAddr)) != SOCKET_ERROR)
		&& (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) != SOCKET_ERROR))
	{
		if (!command.isCommand(C_Discovery)) return LW_RETURN_COMMAND_ERROR;
		auto lik = ntohl(*((uint32_t*)(command.getArgField() + 4)));
		if (lik != 0)
		{
			char buffer[128];
			auto ipPtr = (unsigned char*)&lik;
			snprintf(buffer, 128, "The device is already occupied.(%u.%u.%u.%u)", ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
			gGlobal.errorInfo = std::string(buffer);
			LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
			return LW_RETURN_CUSTOM_ERROR;
		}
	}
	else
	{
		gGlobal.errorInfo = getNetworkLastError();
		LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
		return LW_RETURN_NETWORK_ERROR;
	}

	// 建立新连接
	commandSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	dataSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	int bOpt = true;
	if ((setsockopt(commandSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR)
		&& (setsockopt(dataSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bOpt, sizeof(bOpt)) != SOCKET_ERROR)
		&& (bind(commandSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) != SOCKET_ERROR)
		&& (bind(dataSocket, (sockaddr*)(&localAddr), sizeof(localAddr)) != SOCKET_ERROR)
		&& connectToServer(commandSocket, remoteAddr, t)
		&& connectToServer(dataSocket, recvAddr, t))
	{
		connectEnable.store(true);
		socketNotify.notify_all();

		// 重新获取IMU数据起始位姿信息
		LWIMUData temp;
		GetIMUData(temp);

		return LW_RETURN_OK;
	}

	auto error_str = getNetworkLastError();
	gGlobal.errorInfo = error_str.empty() ? "Device reconnection timeout!" : error_str;
	LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());

	return LW_RETURN_CUSTOM_ERROR;
}

LWReturnCode ljhNS::DeviceHandle::RebootDevice()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_DeviceReboot };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode ljhNS::DeviceHandle::SaveConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SaveConfig };

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::RemoveConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_DeleteConfig };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode ljhNS::DeviceHandle::RestoreFactoryConfigureInfo()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_RecoveryDefaultConfigure };

	auto ret = ExecuteCommand(command);
	if (ret == LW_RETURN_OK)
	{
		openEnable.store(false);
		connectEnable.store(false);

		closeSocket(commandSocket);
		closeSocket(dataSocket);
	}

	return ret;
}

LWReturnCode ljhNS::DeviceHandle::StartStream()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	
	CommandSFrame command{ C_Start };
	
	readyEnable.store(false);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::StopStream()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_Stop };

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::HasRgbModule(bool& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetImageInfo };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = bool(commandRecvFrame.getArgField()[0]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SoftTrigger()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	//LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (triggerMode != LW_TRIGGER_SOFT)
	{
		gGlobal.errorInfo = "The sensor is currently not in soft trigger mode.";
		LOG_ERROR_OUT("<%s>, %s", _SN_.c_str(), gGlobal.errorInfo.c_str());
		return LW_RETURN_CUSTOM_ERROR;
	}

	CommandSFrame command{ C_SoftTrigger };

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetDataReceiveType(LWDataRecvType type)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Type: %u", _SN_.c_str(), type);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetRecvDataType };
	uint8_t val = type;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetTriggerMode(LWTriggerMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", _SN_.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetTriggerMode };

	uint8_t val = mode;
	command.setArgField(&val, 1);
	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		triggerMode = mode;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetTriggerMode(LWTriggerMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetTriggerMode };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = LWTriggerMode(commandRecvFrame.getArgField()[0]);
		triggerMode = mode;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetExposureMode(LWSensorType sensorType, LWExposureMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u, Mode: %u", _SN_.c_str(), sensorType,  mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_SetIntegralTime);

		uint16_t data = (mode == LW_EXPOSURE_AUTO) ? 0 : htons(1000);
		command.setArgField(&data, 2);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_SetRgbCameraExposureMode);

		uint8_t val = (mode == LW_EXPOSURE_AUTO) ? 0 : 1;
		command.setArgField(&val, 1);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetExposureMode(LWSensorType sensorType, LWExposureMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;
	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_GetIntegralModel);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_GetRgbCameraExposureMode);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = commandRecvFrame.getArgField()[0] == 0x00 ? LW_EXPOSURE_AUTO : LW_EXPOSURE_MANUAL;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetHDRMode(LWHDRMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", _SN_.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetHDR };

	unsigned char val = mode;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetHDRMode(LWHDRMode& mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetHDR };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		mode = LWHDRMode((uint8_t)commandRecvFrame.getArgField()[0]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTransformRgbToDepthEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (enable)
	{
		if (!isR2DEnable.load())
		{
			isR2DEnable.store(true);

			int start_row = 0;
			int end_cow = 0;
			int step = tofHeight / THREAD_POOL_SIZE;
			for (int i = 0; i < THREAD_POOL_SIZE; i++)
			{
				start_row = i * step;
				end_cow = (i < THREAD_POOL_SIZE - 1) ? (start_row + step) : tofHeight;
				std::thread{ &DeviceHandle::rgbToDepthThread, this, i, start_row, end_cow }.detach();
			}
		}
	}
	else
	{
		if (isR2DEnable.load())
		{
			isR2DEnable.store(false);
			r2dNotify.notify_all();
		}
	}

	return LW_RETURN_OK;
}

LWReturnCode ljhNS::DeviceHandle::SetTransformDepthToRgbEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (enable)
	{
		if (!isD2REnable.load())
		{
			isD2REnable.store(true);

			int start_row = 0;
			int end_cow = 0;
			int step = tofHeight / THREAD_POOL_SIZE;
			for (int i = 0; i < THREAD_POOL_SIZE; i++)
			{
				start_row = i * step;
				end_cow = (i < THREAD_POOL_SIZE - 1) ? (start_row + step) : tofHeight;
				std::thread{ &DeviceHandle::depthToRgbThread, this, i, start_row, end_cow }.detach();
			}
		}
	}
	else
	{
		if (isD2REnable.load())
		{
			isD2REnable.store(false);
			d2rNotify.notify_all();
		}
	}

	return LW_RETURN_OK;
}

LWReturnCode ljhNS::DeviceHandle::SetFrameRate(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 56 || value < 1)
	{
		return LW_RETURN_ARG_OUT_OF_RANGE;
	}

	CommandSFrame command{ C_SetFrameRate };
	auto val = uint8_t(value);
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetFrameRate(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetFrameRate };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = uint8_t(commandRecvFrame.getArgField()[0]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetExposureTime(LWSensorType sensorType, const int32_t* exposureTimeArray, int32_t arraySize)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	for(int i =0; i < arraySize; ++i) str += (std::to_string(exposureTimeArray[i]) + " ");
	LOG_INFO_OUT("<%s>, SensorType: %u, Value: %s", _SN_.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	if (arraySize > 4 || arraySize < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_UNOPENED;
	}

	uint16_t data[10];
	CommandSFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		for (int i = 0; i < arraySize; i++)
		{
			if (exposureTimeArray[i] > 4000 || exposureTimeArray[i] < 1) return LW_RETURN_ARG_OUT_OF_RANGE;
			data[i] = htons(uint16_t(exposureTimeArray[i]));
		}
		
		command.setCommand(C_SetIntegralTime);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		for (int i = 0; i < arraySize; i++)
		{
			if (exposureTimeArray[i] > 1404 || exposureTimeArray[i] < 1) return LW_RETURN_ARG_OUT_OF_RANGE;
			data[i] = htons(uint16_t(exposureTimeArray[i]));
		}

		command.setCommand(C_SetRgbCameraExposureTime);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}
	
	command.setArgField(data, arraySize * 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetExposureTime(LWSensorType sensorType, int32_t* exposureTimeArray, int32_t arraySize, int32_t& filledCount)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", _SN_.c_str(), sensorType);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (arraySize < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	CommandSFrame command;
	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_GetRgbCameraExposureTime);
	}
	else if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_GetIntegralTime);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		int len = commandRecvFrame.getArgFieldLength() / 2;
		if (arraySize < len) return LW_RETURN_OUT_OF_MEMORY;

		auto data = (uint16_t*)(commandRecvFrame.getArgField());
		for (int i = 0; i < len; i++)
		{
			exposureTimeArray[i] = ntohs(data[i]);
		}

		filledCount = len;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetResolution(LWSensorType type, int32_t &width, int32_t &height)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", _SN_.c_str(), type);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	
    if (type == LWSensorType::LW_RGB_SENSOR)
    {
        CommandSFrame command{ C_GetImageInfo };

        auto RC = ExecuteCommand(command);
        if (RC == LW_RETURN_OK)
        {
            auto data = commandRecvFrame.getArgField();

            hasRgbModule.store(bool(data[0]));

            if (!hasRgbModule.load()) return LW_RETURN_TYPE_NOT_EXIST;

            width = ntohs(*((uint16_t*)(data + 1)));
            height = ntohs(*((uint16_t*)(data + 3)));
        }

        return RC;
    }
    else if (type == LWSensorType::LW_TOF_SENSOR)
    {
		CommandSFrame command{ C_GetBinning };

		auto RC = ExecuteCommand(command);
		if (RC == LW_RETURN_OK)
		{
			auto data = commandRecvFrame.getArgField();

			if (data[0] == 0x00)
			{
				width = 640;
				height = 480;
			}
			else if (data[0] == 0x01)
			{
				width = 320;
				height = 240;
			}
			else if (data[0] == 0x02)
			{
				width = 160;
				height = 120;
			}
			else
			{
				gGlobal.errorInfo = "获取的\"Binning\"模式不在解析范围!";
				return LW_RETURN_CUSTOM_ERROR;
			}
		}

		return RC;
    }

    return LW_RETURN_NOT_SUPPORTED;
}

LWReturnCode ljhNS::DeviceHandle::GetIntrinsicParam(LWSensorType sensorType, LWSensorIntrinsicParam& intrinsicParam)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u", _SN_.c_str(), sensorType);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command;

    if (sensorType == LWSensorType::LW_TOF_SENSOR)
    {
        command.setCommand(C_GetTofCameraIntrinsicArg);
    }
    else if (sensorType == LWSensorType::LW_RGB_SENSOR)
    {
        command.setCommand(C_GetRgbCameraIntrinsicArg);
    }
    else
    {
        return LW_RETURN_TYPE_NOT_EXIST;
    }

	intrinsicParam.k1 = 0;
	intrinsicParam.k2 = 0;
	intrinsicParam.k3 = 0;
	intrinsicParam.p1 = 0;
	intrinsicParam.p2 = 0;
    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr1 = (double*)commandRecvFrame.getArgField();
		int len = commandRecvFrame.getArgFieldLength() / sizeof(double);
        auto ptr2 = (float*)&intrinsicParam;
        for (int i = 0; i < len; i++)
        {
            *ptr2++ = float(*ptr1++);
        }
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetExtrinsicParam(LWSensorType sensorType, LWSensorExtrinsicParam &extrinsicParam)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetRgbCameraExtrinsicArg };

    //if (sensorType == LWSensorType::LW_RGB_SENSOR)
    //{
    //    command.setCommand(C_GetRgbCameraExtrinsicArg);
    //}
    //else
    //{
    //    return LW_RETURN_TYPE_NOT_EXIST;
    //}

	auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr1 = (double*)command.getArgField();
        auto ptr2 = (float*)&extrinsicParam;
        for (int i = 0; i < 12; i++)
        {
            *ptr2++ = float(*ptr1++);
        }
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTimeFilterParams(LWFilterParam param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d, %d]", _SN_.c_str(), param.enable, param.threshold, param.k1);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 2 
		|| param.threshold > 28
		|| param.k1 < 1
		|| param.k1 > 10000
		) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint32_t var = htonl(param.k1);
	uint8_t arg[6] = { param.enable, (uint8_t)param.threshold };
	CommandSFrame command{ C_SetTimeFilter };
	memcpy(arg + 2, &var, 4);
	command.setArgField(&arg, 6);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetTimeFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetTimeFilter };

    auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = commandRecvFrame.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = ntohl(*((int32_t*)(ptr + 2)));
		param.k2 = 0;
	}

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetFlyingPixelsFilterParams(LWFilterParam param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", _SN_.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 1 || param.threshold > 64) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandSFrame command{ C_SetFlyPixeFilter };
	command.setArgField(&arg, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetFlyingPixelsFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetFlyPixeFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = commandRecvFrame.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetConfidenceFilterParams(LWFilterParam param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", _SN_.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (param.threshold < 1 || param.threshold > 150) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandSFrame command{ C_SetConfidenceFilter };
	command.setArgField(&arg, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetConfidenceFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetConfidenceFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = commandRecvFrame.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetSpatialFilterParams(LWFilterParam param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", _SN_.c_str(), param.enable, param.threshold);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (std::unordered_set<int>{3, 5, 7}.count(param.threshold) == 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint8_t arg[2] = { param.enable, (uint8_t)param.threshold };
	CommandSFrame command{ C_SetSpatialFilter };
	command.setArgField(&arg, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetSpatialFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetSpatialFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = commandRecvFrame.getArgField();
		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = 0;
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTimeMedianFilterParams(LWFilterParam param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d, %d]", _SN_.c_str(), param.enable, param.threshold, param.k1);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (std::unordered_set<int>{3, 5, 7, 9}.count(param.threshold) == 0 
		|| param.k1 < 1
		|| param.k1 > 10000
		) return LW_RETURN_ARG_OUT_OF_RANGE;

	uint32_t var = htonl(param.k1);
	uint8_t arg[6] = { param.enable, (uint8_t)param.threshold };
	memcpy(arg + 2, &var, 4);
	CommandSFrame command{ C_SetTimeMedianFilter };
	command.setArgField(&arg, 6);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetTimeMedianFilterParams(LWFilterParam& param)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetTimeMedianFilter };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        auto ptr = commandRecvFrame.getArgField();

		param.enable = bool(ptr[0]);
		param.threshold = uint8_t(ptr[1]);
		param.k1 = ntohl(*((int32_t*)(ptr + 2)));
		param.k2 = 0;
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetIRGMMGain(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value < 0 || value > 255) return LW_RETURN_ARG_OUT_OF_RANGE;

	irGMMGain = value;

	uint8_t val = value;
	CommandSFrame command{ C_SetIRGMMGain };
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetIRGMMGain(int32_t &gain)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

    CommandSFrame command{ C_GetIRGMMGain };

    auto RC = ExecuteCommand(command);
    if (RC == LW_RETURN_OK)
    {
        gain = uint8_t(commandRecvFrame.getArgField()[0]);
    }

    return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorGain(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 988 || value < 16) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbCameraGain };

	uint16_t var = htons(value);
	command.setArgField(&var, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorGain(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetRgbCameraGain };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohs(*((unsigned short*)commandRecvFrame.getArgField()));
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorGamma(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	if (value > 300 || value < 64) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbCameraGamma };

	uint16_t var = htons(value);
	command.setArgField(&var, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorGamma(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	CommandSFrame command{ C_GetRgbCameraGamma };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohs(*((unsigned short*)commandRecvFrame.getArgField()));
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorBrightness(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	
	return LW_RETURN_NOT_SUPPORTED;

	if (value > 64 || value < -64) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbCameraBrightness };

	uint32_t var = htonl(value);
	command.setArgField(&var, 4);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorBrightness(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	CommandSFrame command{ C_GetRgbCameraBrightness };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohl(*((int32_t*)commandRecvFrame.getArgField()));
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorContrastRatio(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	if (value > 95 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbCameraContrastRatio };

	uint8_t var = value;
	command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorContrastRatio(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	CommandSFrame command{ C_GetRgbCameraContrastRatio };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = (uint8_t)commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorSaturation(int32_t value)
{
	return LWReturnCode(); std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	//if (value > 95 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbSensorSaturation };

	//uint8_t var = value;
	//command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorSaturation(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	CommandSFrame command{ C_GetRgbSensorSaturation };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		//value = (uint8_t)commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetRgbSensorWhiteBalance(int32_t value)
{
	return LWReturnCode(); std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	//if (value > 95 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetRgbSensorSaturation };

	//uint8_t var = value;
	//command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetRgbSensorWhiteBalance(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	CommandSFrame command{ C_GetRgbSensorWhiteBalance };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		//value = (uint8_t)commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetNetworkInfo(LWNetworkInfo info)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Type: %d, IP: %s, Netmask: %s", _SN_.c_str(), info.type, info.ip, info.netmask);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetNetworkModel };

	uint8_t data[9]{};
	data[0] = info.type;
	if (info.type == 0x01)
	{
		if (inet_pton(AF_INET, info.ip, data + 1) != 1 || inet_pton(AF_INET, info.netmask, data + 5) != 1)
		{
			gGlobal.errorInfo = getNetworkLastError();
			return LW_RETURN_NETWORK_ERROR;
		}
	}
	command.setArgField(data, 9);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetNetworkInfo(LWNetworkInfo& info)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetNetworkModel };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		char arr[16];
		auto data = commandRecvFrame.getArgField();

		info.type = data[0];

		auto ret = inet_ntop(AF_INET, data + 1, arr, 16);
		if (ret == nullptr) return LW_RETURN_NETWORK_ERROR;
		memcpy(info.ip, arr, 16);

		ret = inet_ntop(AF_INET, data + 5, arr, 16);
		if (ret == nullptr) return LW_RETURN_NETWORK_ERROR;
		memcpy(info.netmask, arr, 16);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetDeviceNumber(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (value > 255 || value < 1) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetCameraNumber };

	uint8_t var = value;
	command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetDeviceNumber(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetCameraNumber };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = uint8_t(commandRecvFrame.getArgField()[0]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetDepthCompensateValue(int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	//if (value > 255 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetDepthCompensate };

	uint16_t var = value;
	var = htons(var);
	command.setArgField(&var, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetDepthCompensateValue(int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetDepthCompensate };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = ntohs(*((unsigned short*)commandRecvFrame.getArgField()));
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetHardTriggerFilterParams(int32_t t1, int32_t t2)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: [%d, %d]", _SN_.c_str(), t1, t2);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (t1 < 1000 || t2 < 50 || t1 > 65535 || t2 > 65535) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetDelayHardTriggerTime };

	uint16_t data[2] = { htons((uint16_t)t1),  htons((uint16_t)t2) };
	command.setArgField(data, 4);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetHardTriggerFilterParams(int32_t& t1, int32_t& t2)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetDelayHardTriggerTime };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = (unsigned short*)commandRecvFrame.getArgField();
		t1 = ntohs(ptr[0]);
		t2 = ntohs(ptr[1]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetIMUExtrinsicParam(LWIMUExtrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetIMUExtrinsicParam };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto* ptr1 = (double*)commandRecvFrame.getArgField();
		auto* ptr2 = (float*)&para;
		for (int i = 0; i < 9; i++)
		{
			*ptr2++ = float(*ptr1++);
		}
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetIMUData(LWIMUData& imu_data)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetIMUData };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = commandRecvFrame.getArgField();
		if (ptr[0] == 0x00) return LW_RETURN_NOT_SUPPORTED;

		cal_mode = (uint8_t)ptr[1];
		memcpy(&imu_data, ptr + 2, 36);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetIMUFrequency(int32_t value)
{
	return LWReturnCode(); std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %d", _SN_.c_str(), value);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return LW_RETURN_NOT_SUPPORTED;

	//if (value > 95 || value < 0) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command{ C_SetIMUFrequency };

	//uint8_t var = value;
	//command.setArgField(&var, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetDeviceSN(char* sn, int32_t bufferLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetSN };

	auto RC = ExecuteCommand(command);
	if (RC != LW_RETURN_OK) return RC;
	auto len = commandRecvFrame.getArgFieldLength();
	if (len > bufferLen) return LW_RETURN_OUT_OF_MEMORY;

	auto _sn = std::string(commandRecvFrame.getArgField(), len);

	if (_SN_ != _sn)
	{
		gGlobal.strDeviceMap[_sn] = gGlobal.strDeviceMap[_SN_];
		gGlobal.numDeviceMap[handle] = gGlobal.strDeviceMap[_SN_];
		gGlobal.strDeviceMap.erase(_SN_);
		_SN_ = _sn;
	}

	memcpy(sn, _sn.c_str(), len);
	if (len != bufferLen) sn[len] = '\0';
	return LW_RETURN_OK;
	
}

LWReturnCode ljhNS::DeviceHandle::GetDeviceType(char* type, int32_t bufferLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetDeviceType };

	auto RC = ExecuteCommand(command);
	if (RC != LW_RETURN_OK) return RC;
	auto len = commandRecvFrame.getArgFieldLength();
	if (len > bufferLen) return LW_RETURN_OUT_OF_MEMORY;

	memcpy(type, commandRecvFrame.getArgField(), len);
	if (len != bufferLen) type[len] = '\0';
	return LW_RETURN_OK;
}

LWReturnCode ljhNS::DeviceHandle::GetTimeStamp(LWTimeStamp& t)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetTimeStamp };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto ptr = commandRecvFrame.getArgField();
		t.tv_sec = *((uint32_t*)ptr);
		t.tv_usec = *((uint32_t*)(ptr + 4));
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetDeviceVersion(LWVersionInfo& fv, LWVersionInfo& dv)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetVersion };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto data = (uint8_t*)commandRecvFrame.getArgField();

		fv.major 	= data[0];
		fv.minor 	= data[1];
		fv.patch 	= data[2];
		fv.reserved	= data[3];

		dv.major 	= data[4];
		dv.minor 	= data[5];
		dv.patch 	= data[6];
		dv.reserved	= data[7];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetFrameReady()
{
	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	std::unique_lock<std::mutex> lock{syncDataMutex};
	if (syncDataNotify.wait_for(lock, std::chrono::milliseconds{ timeout }, [this] { return readyEnable.load() || !aliveEnable.load(); }))
	{
		if (!aliveEnable.load()) return LW_RETURN_UNINITIALIZED;
		if (rgbNode->update) std::swap(rgbNode, rgbCutNode);
		if (tofNode->update) std::swap(tofNode, tofCutNode);
		isReady = readyEnable.load();
		readyEnable.store(false);

		gGlobal.d2rTF = true;
		gGlobal.r2dTF = true;

		return LW_RETURN_OK;
	}

	return LW_RETURN_TIMEOUT;
}

LWReturnCode ljhNS::DeviceHandle::UpdateFirmware(const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	std::string password = "dm-zgyfjch";
	if(!deviceType.empty() && (deviceType.find("D345") != std::string::npos))
	{
		password = "dm-se-zgyfjch";
	}

	UpdateToolTask task(inet_ntoa(remoteAddr.sin_addr), 22, "root", password, filename, 3000);
	UpdateResultType result = task.processor();
	if (result == RESULT_OK)
	{
		return LW_RETURN_OK;
	}
	else
	{
		gGlobal.errorInfo = result.desc;
		return LW_RETURN_FIRMWARE_UPDATE_FAIL;
	}
}

LWReturnCode ljhNS::DeviceHandle::SetOutputDO(int32_t channel, int32_t value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetOutputDO };

	char val[2] = { (char)channel, (char)value };
	command.setArgField(val, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetOutputDO(int32_t channel, int32_t& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetOutputDO };
	char val = channel;
	command.setArgField(&val, 1);

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		int len = commandRecvFrame.getArgFieldLength();
		auto src_data = commandRecvFrame.getArgField();
		for (int i = 0; i < len; i += 2)
		{
			if (int(src_data[i]) == channel)
			{
				value = src_data[i+1];
				return LW_RETURN_OK;
			}
		}
		
		return LW_RETURN_INDEX_NOT_EXIST;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::HasPalletIdentifyModule(bool& value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetPalletIdentifySupport };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		value = (bool)commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::UploadRKNNFile(const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return SendFile(filename, 0xf0);
}

LWReturnCode ljhNS::DeviceHandle::SetPalletConfigureFile(const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return SendFile(filename, 0x05);
}

LWReturnCode ljhNS::DeviceHandle::SetPalletConfigureFileFromBuffer(const char* buf, int32_t bufLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s, is nullptr? %d>", _SN_.c_str(), buf == nullptr);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return SendFile(buf, bufLen, 0x05);
}

LWReturnCode ljhNS::DeviceHandle::GetPalletConfigureFile(const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetPalletConfigureFile };
	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		FILE* fp = fopen(filename, "wb");
		if (fp != nullptr)
		{
			int packnum = commandRecvFrame.getTotalSerialNumber();
			if (packnum < 2) {
				fwrite(commandRecvFrame.getArgField(), commandRecvFrame.getArgFieldLength(), 1, fp);
			}
			else {
				for (int i = 0; i < packnum; i++) {
					fwrite(commandRecvFrame.getArgField(i), commandRecvFrame.getArgFieldLength(i), 1, fp);
				}
			}

			fclose(fp);
		}
		else
		{
			return LW_RETURN_FILE_OPEN_ERROR;
		}
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetPalletConfigureFileToBuffer(char* buf, int32_t len)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetPalletConfigureFile };
	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		int packnum = commandRecvFrame.getTotalSerialNumber();

		if ((len + packnum * 18) < commandRecvFrame.size()) return LW_RETURN_OUT_OF_MEMORY;

		if (packnum < 2)
		{
			uint32_t arglen = commandRecvFrame.getArgFieldLength();
			memcpy(buf, commandRecvFrame.getArgField(), arglen);
			buf[arglen] = '\0';
		}
		else {

			uint32_t arglen = 0;
			uint32_t sum = 0;
			for (int i = 0; i < packnum; i++) {
				arglen = commandRecvFrame.getArgFieldLength(i);
				memcpy(buf + sum, commandRecvFrame.getArgField(i), arglen);
				sum += arglen;
			}
			buf[sum] = '\0';
		}
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetPalletIdentifyType(const char* type)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetPalletIdentifyType };

	command.setArgField(type, strlen(type));

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetPalletIdentifyType(char* type, int32_t len)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetPalletIdentifyType };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		if (len < commandRecvFrame.getArgFieldLength()) return LW_RETURN_OUT_OF_MEMORY;
		
		memset(type, 0, len);
		memcpy(type, commandRecvFrame.getArgField(), commandRecvFrame.getArgFieldLength());
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetPalletIdentifyEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetPalletIdentifyEnable };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetPalletIdentifyEnable(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetPalletIdentifyEnable };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = (bool)commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTRSDsimilarMax(float val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetTRSDsimilarMax };

	command.setArgField(&val, 4);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetTRSDsimilarMax(float& val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetTRSDsimilarMax };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		FloatUint32 uinf;
		uinf.u = ntohl(*((uint32_t*)commandRecvFrame.getArgField()));
		val = uinf.f;
	}
	
	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTRSDpstMax(float val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetTRSDpstMax };

	command.setArgField(&val, 4);
	
	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetTRSDpstMax(float& val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetTRSDpstMax };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		FloatUint32 uinf;
		uinf.u = ntohl(*((uint32_t*)commandRecvFrame.getArgField()));
		val = uinf.f;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetCutHeight(int32_t val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetCutHeight };

	command.setArgField(&val, 4);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetCutHeight(int32_t& val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetCutHeight };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		val = *((int32_t*)commandRecvFrame.getArgField());
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SendFile(const char* fullname, uint32_t type)
{
	LOG_INFO_OUT("<%s>, FileType: %d, Filename: %s", _SN_.c_str(), type, fullname);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	// 判定文件是否存在
	//if(access(fullname, 0) != 0) return LW_RETURN_FILE_NOT_EXIST;

	// 获取文件状态信息
	struct stat statbuf {};
	stat(fullname, &statbuf);
	
	// 数据初始化
	LWReturnCode		returncode = LW_RETURN_NETWORK_ERROR;
	uint32_t			retransmit = 3;//重传次数
	uint32_t 			file_size = statbuf.st_size;
	AutoMemoryManager 	dat{65535};
	auto 				buf = dat.u_data();
	CommandSFrame 		command{ C_FileArgTransfer };

	command.setVersion(0x01);
	buf[0] = type; // 文件类型
	uint32_t val32 = htonl(file_size);
	memcpy(buf + 1, &val32, 4);				// 文件长度
	getFileMD5(fullname, buf + 5, val32); // 文件DM5信息
	if (type != 0xf0)
	{
		command.setArgField(buf, 21);
	}
	else
	{
		auto fileName = getFileNameFromStr(fullname);
		memcpy(buf + 21, fileName.data(), fileName.size()); // 文件名
		command.setArgField(buf, 21 + fileName.size());
	}

	// 打开文件
	FILE* fp = fopen(fullname, "rb");
	if (fp == nullptr) return LW_RETURN_FILE_OPEN_ERROR;

	// 建立网络通讯
	sockaddr_in send_addr = remoteAddr;
	send_addr.sin_port = htons(64110);
	SOCKET _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ((bind(_socket, (sockaddr *)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR) || !setNetworkTimeout(_socket, timeout))
	{
		goto LABE1;
	}

	// 发送数据
	for (uint32_t i = 0; i < retransmit; i++)
	{
		// 传递文件信息
		if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR) 
			|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
		{
			continue;
		}

		if (command.isCommand(C_FileArgTransfer))
		{
			uint16_t count = 1;
			uint32_t residue_size = file_size;
			uint32_t block_size = file_size < 50000u ? file_size : 50000u;

			command.setCommand(C_FileDataTransfer);
			command.setTotalSerialNumber((file_size % block_size > 0) ? (file_size / block_size + 1) : (file_size / block_size));

			while (fread(buf, block_size, 1, fp) != 0)
			{
				command.setCurrentSerialNumber(count++);
				command.setArgField(buf, block_size);

				for (uint32_t ii1 = 1;; ii1++)
				{
					if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR)
						|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
					{
						if (ii1 > retransmit)
							goto LABE1;
						continue;
					}

					if (command.isCommand(C_FileDataTransfer))
						break;

					if (ii1 > retransmit)
					{
						returncode = LW_RETURN_COMMAND_ERROR;
						goto LABE2;
					}
				}

				residue_size -= block_size;

				if (residue_size < 1)
					break;

				if (residue_size < block_size)
					block_size = residue_size;
			}

			// 校验文件信息
			command.setCommand(C_FileAckTransfer);
			command.setCurrentSerialNumber(1);
			command.setTotalSerialNumber(1);
			command.setArgField(nullptr, 0);
			for (uint8_t ii2 = 1;; ii2++)
			{
				if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR)
					|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
				{
					if (ii2 > retransmit)
						goto LABE1;
					continue;
				}

				if (command.isCommand(C_FileAckTransfer))
				{
					returncode = command.getCommandType() == 0x02 ? LW_RETURN_OK : LWReturnCode(command.getCommandType());
					goto LABE2;
				}

				if (ii2 > retransmit)
				{
					returncode = LW_RETURN_COMMAND_ERROR;
					goto LABE2;
				}
			}
		}

	}

LABE1:
	gGlobal.errorInfo = getNetworkLastError();

LABE2:
	fclose(fp);
	closeSocket(_socket);
	
	return returncode;

}

LWReturnCode ljhNS::DeviceHandle::SendFile(const char* buffer, int len, uint32_t type)
{
	LOG_INFO_OUT("<%s>---Memory data", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	// 计算文件的MD5
	unsigned int md_len;
	unsigned char md5[EVP_MAX_MD_SIZE];
	EVP_MD_CTX* mdContext = EVP_MD_CTX_new();
	EVP_DigestInit_ex(mdContext, EVP_md5(), NULL);
	EVP_DigestUpdate(mdContext, buffer, len);
	EVP_DigestFinal_ex(mdContext, md5, &md_len);
	EVP_MD_CTX_free(mdContext);

	//FILE* fp = fopen("C:\\Users\\12267\\Desktop\\DATA\\MD5T.txt", "w");
	//fwrite(buffer, len, 1, fp);
	//fclose(fp);

	//std::cout << len << std::endl;
	//std::cout << ljhNS::getHexStringFromBytes(md5, md_len, ' ') << std::endl;

	// 数据初始化
	LWReturnCode		returncode = LW_RETURN_NETWORK_ERROR;
	uint32_t			retransmit = 3;//重传次数
	AutoMemoryManager 	dat{ 65535 };
	auto 				buf = dat.data();
	CommandSFrame 		command{ C_FileArgTransfer };

	command.setVersion(0x01);
	buf[0] = type;						//文件类型
	uint32_t val32 = htonl(len);
	memcpy(buf + 1, &val32, 4);			//文件长度
	memcpy(buf + 5, md5, 16);			//文件DM5信息
	command.setArgField(buf, 21);

	// 建立网络通讯
	sockaddr_in send_addr = remoteAddr;
	send_addr.sin_port = htons(64110);
	SOCKET _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if ((bind(_socket, (sockaddr *)(&localAddr), sizeof(localAddr)) == SOCKET_ERROR) || !setNetworkTimeout(_socket, timeout))
	{
		goto LABE1;
	}

	for (uint8_t i = 0; i < retransmit; i++)
	{
		// 传递文件信息
		if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR)
			|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
		{
			continue;
		}

		// 传递文件内容，50000字节一个数据包
		if (command.isCommand(C_FileArgTransfer))
		{
			uint16_t 	count = 1;
			uint32_t 	index = 0;
			uint32_t 	residue_size = len;
			uint32_t 	block_size = len < 50000u ? len : 50000u;

			command.setCommand(C_FileDataTransfer);
			command.setTotalSerialNumber((len % block_size > 0) ? (len / block_size + 1) : (len / block_size));

			while (residue_size > 0 )
			{
				command.setCurrentSerialNumber(count++);
				command.setArgField(buffer + index, block_size);

				for (uint32_t ii1 = 1;; ii1++)
				{
					if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR) 
						|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
					{
						if (ii1 > retransmit)
							goto LABE1;
						continue;
					}

					if (command.isCommand(C_FileDataTransfer))
						break;

					if (ii1 > retransmit)
					{
						returncode = LW_RETURN_COMMAND_ERROR;
						goto LABE2;
					}
				}

				index += block_size;
				residue_size -= block_size;
				if (residue_size < block_size)
					block_size = residue_size;
			}

			// 校验文件信息
			command.setCommand(C_FileAckTransfer);
			command.setCurrentSerialNumber(1);
			command.setTotalSerialNumber(1);
			command.setArgField(nullptr, 0);
			for (uint8_t ii2 = 1;; ii2++)
			{
				if ((sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr *)&send_addr, sizeof(send_addr)) == SOCKET_ERROR)
					|| (recvfrom(_socket, command.data(), command.maxBuffer(), 0, nullptr, nullptr) == SOCKET_ERROR))
				{
					if (ii2 > retransmit)
						goto LABE1;
					continue;
				}

				if (command.isCommand(C_FileAckTransfer))
				{
					returncode = command.getCommandType() == 0x02 ? LW_RETURN_OK : LWReturnCode(command.getCommandType());
					goto LABE2;
				}

				if (ii2 > retransmit)
				{
					returncode = LW_RETURN_COMMAND_ERROR;
					goto LABE2;
				}
			}
		}
	}


LABE1:
	gGlobal.errorInfo = getNetworkLastError();

LABE2:
	closeSocket(_socket);

	return returncode;

}


/*************************** 内部接口 ***************************/
#ifdef  LW_INTERNAL_API

LWReturnCode ljhNS::DeviceHandle::SetDeviceSN(const char* sn, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, _SN_: %s", _SN_.c_str(), sn);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetSN };
	command.setArgField(sn, size);

	auto _sn = std::string(sn, size);
	auto ret = ExecuteCommand(command);
	if ((ret == LW_RETURN_OK) && (_SN_ != _sn))
	{
		gGlobal.strDeviceMap[_sn] = gGlobal.strDeviceMap[_SN_];
		gGlobal.numDeviceMap[handle] = gGlobal.strDeviceMap[_SN_];
		gGlobal.strDeviceMap.erase(_SN_);
		_SN_ = _sn;
	}
	return ret;
}

LWReturnCode ljhNS::DeviceHandle::SendOperateCommand(const char* comstr, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %s", _SN_.c_str(), comstr);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_OperateCommand };

	command.setArgField(comstr, size + 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetDRNU(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetDRNU };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetBinningMode(LWBinningMode mode)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Mode: %u", _SN_.c_str(), mode);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetBinning };

	uint8_t val = mode;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetResolution(LWSensorType sensorType, int32_t width, int32_t height)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, SensorType: %u, Width: %d, Height: %d", _SN_.c_str(), sensorType, width, height);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	return LW_RETURN_NOT_SUPPORTED;

	uint8_t var = 0xff;
	if (width == 1600 && height == 1200) var = 0x00;
	if (width == 800 && height == 600) var = 0x01;
	if (width == 640 && height == 480) var = 0x02;
	if (var == 0xff) return LW_RETURN_ARG_OUT_OF_RANGE;

	CommandSFrame command;

	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command .setCommand(C_SetRgbCameraResolutionRatio);
	}
	else
	{
		return LW_RETURN_NOT_SUPPORTED;
	}

	command.setArgField(&var, 1);
	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetDistortionCalibration(LWSensorType sensorType, bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		CommandSFrame command{ C_SetDistortion };

		command.setArgField(&enable, 1);

		return ExecuteCommand(command);

	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		rgbDistortionEnable = enable;

		return LW_RETURN_OK;
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}
}

LWReturnCode ljhNS::DeviceHandle::SetLaserWorkFrequency(const uint8_t* arr, int size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	for (int i = 0; i < size; ++i) str += (std::to_string(arr[i]) + " ");
	LOG_INFO_OUT("<%s>, Value: %s", _SN_.c_str(), str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	if (size < 1)
	{
		gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	CommandSFrame command{ C_SetLaserWorkFrequency };

	command.setArgField(arr, size);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetAutoExposureDefaultValue(uint16_t val)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Value: %u", _SN_.c_str(), val);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetExposureValue };

	command.setArgField(&val, 2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetIntrinsicParam(LWSensorType sensorType, const LWSensorIntrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (float*)&para;
	for (int i = 0; i < 7; ++i) str += (std::to_string(ptr[i]) + " ");
	LOG_INFO_OUT("<%s>, SensorType: %d, Value: %s", _SN_.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;

	if (sensorType == LWSensorType::LW_TOF_SENSOR)
	{
		command.setCommand(C_SetCameraIntrinsicArg);
	}
	else if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_SetRgbCameraIntrinsicArg);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	double val[7];
	for (int i = 0; i < 7; i++) val[i] = ptr[i];
	command.setArgField(val, 56);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetExtrinsicParam(LWSensorType sensorType, const LWSensorExtrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (float*)&para;
	for (int i = 0; i < 12; ++i) str += (std::to_string(ptr[i]) + " ");
	LOG_INFO_OUT("<%s>, SensorType: %d, Value: %s", _SN_.c_str(), sensorType, str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;

	if (sensorType == LWSensorType::LW_RGB_SENSOR)
	{
		command.setCommand(C_SetRgbCameraExtrinsicArg);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}

	double val[12];
	for (int i = 0; i < 12; i++) val[i] = ptr[i];
	command.setArgField(val, 96);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetIMUExtrinsicParam(const LWIMUExtrinsicParam& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (float*)&para;
	for (int i = 0; i < 9; ++i) str += (std::to_string(ptr[i]) + " ");
	LOG_INFO_OUT("<%s>, Value: %s", _SN_.c_str(), str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetIMUExtrinsicParam };

	double val[9];
	for (int i = 0; i < 9; i++) val[i] = ptr[i];
	command.setArgField(val, 72);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetTemperatureCompensation(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetTempCompensate };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetTemperatureCompensation(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetTempCompensate };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetTemperatureParams(const LWTemperatureParams& para)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	std::string str;
	auto* ptr = (double*)&para;
	for (int i = 0; i < 4; ++i) str += (std::to_string(ptr[i]) + " ");
	LOG_INFO_OUT("<%s>, Value: %s", _SN_.c_str(), str.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetTempCoefficient };

	command.setArgField(&para, sizeof(para));

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SetLaserEnableStatus(uint32_t flag)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Status: %u", _SN_.c_str(), flag);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetLaserEnableStatus };

	uint8_t val = flag;
	command.setArgField(&val, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetLaserEnableStatus(uint32_t& flag)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetLaserEnableStatus };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		flag = uint8_t(command.getArgField()[0]);
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetDataSyncEnable(bool enable)
{
	dataSyncEnable = enable;

	return LW_RETURN_OK;
}

#endif //LW_INTERNAL_API


#ifdef LW_ZHONGRILONG_PROJ

LWReturnCode ljhNS::DeviceHandle::SynchronizeDeviceSystemTime()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetSystemTime };

	// 获取当前时间点
	auto now = std::chrono::system_clock::now();
	// 转换为自纪元以来的毫秒数
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
	// 
	command.setArgField(&seconds, 8);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::HasSecurityAbility(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_HasSecurityAbility };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityEnable(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetSecurityEnable };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetSecurityCalibrationParams(const int32_t* _array, int32_t size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	//if (size != 4) return LW_RETURN_DATA_SIZE_ERROR;

	CommandSFrame command{ C_SetSecurityCalibrationParams };

	uint16_t data[16];
	for (int i = 0; i < size; i++)
	{
		data[i] = _array[i];
		data[i] = htons(data[i]);
	}
	command.setArgField(data, size*2);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::SecurityCancelCalibration()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SecurityCancelCalibration };

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityCalibrationEnable(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetSecurityCalibrationEnable };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetSecurityAxialAdjustment(int flag, float value)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;
	
	CommandSFrame command{ C_SetSecurityAxialAdjustment };

	char data[5];
	data[0] = flag;
	memcpy(&data[1], &value, 4);
	command.setArgField(data, 5);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::ResetSecurityConfigure()
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_ResetSecurityConfigure };

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityCalibrationMatrixParams(float* mat, int32_t* size)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetSecurityCalibrationMatrixParams };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		auto len = commandRecvFrame.getArgFieldLength();
		if ((*size) * 4 > len) return LW_RETURN_OUT_OF_MEMORY;
		memcpy(mat, commandRecvFrame.getArgField(), len);
		*size = len;
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityZHSAEnable(bool& enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_GetSecurityZHSAEnable };

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		enable = commandRecvFrame.getArgField()[0];
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetSecurityZHSAEnable(bool enable)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>, Enable: %d", _SN_.c_str(), enable);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command{ C_SetSecurityZHSAEnable };

	command.setArgField(&enable, 1);

	return ExecuteCommand(command);
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityConfigFile(int type, const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;
	if (type == LWFileType::LW_SECURITY_LOG)
	{
		command.setCommand(C_GetSecurityLog);
	}
	else if (type == LWFileType::LW_SECURITY_ARG)
	{
		command.setCommand(C_GetSecurityConfigFile);
	}
	else if (type == LWFileType::LW_SECURITY_PR_ARG)
	{
		command.setCommand(C_SecurityPerspectiveFile);
	}
	else if (type == LWFileType::LW_SECURITY_PR_MAT_ARG)
	{
		command.setCommand(C_GetSecurityPerspectiveMatrixFile);
	}
	else if (type == LWFileType::LW_SECURITY_CALIB)
	{
		command.setCommand(C_GetSecurityCalibrationFile);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}
	
	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		FILE* fp = fopen(filename, "wb");
		if (fp != nullptr)
		{
			int packnum = commandRecvFrame.getTotalSerialNumber();
			if (packnum < 2) {
				fwrite(commandRecvFrame.getArgField(), commandRecvFrame.getArgFieldLength(), 1, fp);
			}
			else {

				for (int i = 0; i < packnum; i++) {
					fwrite(commandRecvFrame.getArgField(i), commandRecvFrame.getArgFieldLength(i), 1, fp);
				}
			}

			fclose(fp);
		}
		else
		{
			return LW_RETURN_FILE_OPEN_ERROR;
		}
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::GetSecurityConfigFileToBuffer(int type, char* buffer, int32_t bufLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	CommandSFrame command;

	if (type == LWFileType::LW_SECURITY_LOG)
	{
		command.setCommand(C_GetSecurityLog);
	}
	else if (type == LWFileType::LW_SECURITY_ARG)
	{
		command.setCommand(C_GetSecurityConfigFile);
	}
	else if (type == LWFileType::LW_SECURITY_PR_ARG)
	{
		command.setCommand(C_SecurityPerspectiveFile);
	}
	else if (type == LWFileType::LW_SECURITY_PR_MAT_ARG)
	{
		command.setCommand(C_GetSecurityPerspectiveMatrixFile);
	}
	else if (type == LWFileType::LW_SECURITY_CALIB)
	{
		command.setCommand(C_GetSecurityCalibrationFile);
	}
	else
	{
		return LW_RETURN_TYPE_NOT_EXIST;
	}
	

	auto RC = ExecuteCommand(command);
	if (RC == LW_RETURN_OK)
	{
		int packnum = commandRecvFrame.getTotalSerialNumber();

		if ((bufLen + packnum * 18) < commandRecvFrame.size()) return LW_RETURN_OUT_OF_MEMORY;

		if (packnum < 2)
		{
			uint32_t arglen = commandRecvFrame.getArgFieldLength();
			memcpy(buffer, commandRecvFrame.getArgField(), arglen);
			buffer[arglen] = '\0';
		}
		else {

			uint32_t arglen = 0;
			uint32_t sum = 0;
			for (int i = 0; i < packnum; i++) {
				arglen = commandRecvFrame.getArgFieldLength(i);
				memcpy(buffer + sum, commandRecvFrame.getArgField(i), arglen);
				sum += arglen;
			}
			buffer[sum] = '\0';
		}
	}

	return RC;
}

LWReturnCode ljhNS::DeviceHandle::SetSecurityConfigFile(int type, const char* filename)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s>", _SN_.c_str());

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return SendFile(filename, type);
}

LWReturnCode ljhNS::DeviceHandle::SetSecurityConfigFileFromBuffer(int type, const char* buffer, int32_t bufLen)
{
	std::lock_guard<std::mutex> guard{ nrMutex };

	LOG_INFO_OUT("<%s, is nullptr? %d>", _SN_.c_str(), buffer == nullptr);

	if (!openEnable.load()) return LW_RETURN_UNOPENED;

	return SendFile(buffer, bufLen, type);
}

#endif //LW_ZHONGRILONG_PROJ

/// ******************************************* API接口定义 *******************************************

std::mutex notReentryMutex;//不可重入锁（为了实时监控，命令的收发不在同一线程）

LWReturnCode LWInitializeResources()
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	ljhNS::gGlobal.initEnable.store(true);

    return LW_RETURN_OK;
}

LWReturnCode LWCleanupResources()
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	ljhNS::gGlobal.initEnable.store(false);

    for (auto & iter : ljhNS::gGlobal.strDeviceMap)
    {
        delete iter.second;
    }
	ljhNS::gGlobal.strDeviceMap.clear();
	ljhNS::gGlobal.numDeviceMap.clear();

    return LW_RETURN_OK;
}

LWReturnCode LWGetDeviceInfoList(LWDeviceInfo* deviceInfoList, int32_t listCount, int32_t* findCount)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	// 判定传入的数组大小是否合理
	if (listCount < 1)
	{
		ljhNS::gGlobal.errorInfo = "The size of the incoming array is abnormal.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	// 清空设备列表
	ljhNS::gGlobal.numDeviceMap.clear();
	*findCount = 0;

	// 获取本机所有地址信息，以备后续广播搜寻设备
	std::vector<sockaddr_in> sockaddr_list;
	if (!ljhNS::getLocalAddrInfo(sockaddr_list))
	{
		ljhNS::gGlobal.errorInfo = ljhNS::getNetworkLastError();
		LOG_ERROR_OUT("%s", ljhNS::gGlobal.errorInfo.c_str());
		return LW_RETURN_NETWORK_ERROR;
	}

	// 多线程广播搜索设备，一个本机地址对应一个线程
	std::mutex _mutex;
	std::vector<std::thread> thread_list;
	for (const auto& local_addr : sockaddr_list)
	{
		thread_list.emplace_back([&local_addr, &_mutex, &deviceInfoList, &listCount, &findCount]()->void {
			// 广播地址
			sockaddr_in to_addr{};
			to_addr.sin_family = AF_INET;
			to_addr.sin_port = htons(COMMAND_PORT);
			to_addr.sin_addr.s_addr = INADDR_BROADCAST;
			// 发送广播消息
			ljhNS::CommandSFrame command{ C_Discovery };
			command.setVersion(0x03);
			SOCKET &_socket = ljhNS::gSocketMap[local_addr.sin_addr.s_addr];
			if (sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr*)&to_addr, sizeof(to_addr)) == SOCKET_ERROR ||
				!ljhNS::setNetworkTimeout(_socket, 3000)
				)
			{
				ljhNS::gGlobal.errorInfo = ljhNS::getNetworkLastError();
				LOG_ERROR_OUT("%s", ljhNS::gGlobal.errorInfo.c_str());
				return;
			}
			// 接收应答信息
			sockaddr_in from_addr{};
			socklen_t addr_len = sizeof(from_addr);
			while (true)
			{
				if (recvfrom(_socket, command.data(), command.maxBuffer(), 0, (sockaddr*)&from_addr, &addr_len) == SOCKET_ERROR) break;
				
				// 过滤掉IP为：66.66.66.66 的地址
				if (from_addr.sin_addr.s_addr == 1111638594U) continue;

				// 判定应答包
				if (command.isCommand(C_Discovery))
				{
					if (command.getVersion() > 0x02)
					{
						auto reply = (uint8_t*)(command.getArgField());

						uint32_t devIP = from_addr.sin_addr.s_addr;
						uint32_t locIP = local_addr.sin_addr.s_addr;
						uint32_t devMrak = ntohl(*((uint32_t*)(reply + 13)));

						// 获取设备序列号
						std::string sn(command.getArgField() + 17, 13);
						if (sn.find("DM") == std::string::npos) continue;

						// 生成设备唯一标识ID
						LWDeviceHandle id = 0;
						auto idPtr = (char*)&id;
						memcpy(idPtr, &devIP, 4);
						memcpy(idPtr + 4, &locIP, 4);

						// 获取设备类型
						std::string devType;
						auto t_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
						ljhNS::setNetworkTimeout(t_socket, 1500);
						ljhNS::CommandSFrame t_command{ C_GetDeviceType };
						if (sendto(t_socket, t_command.data(), t_command.size(), MSG_NOSIGNAL, (sockaddr*)&from_addr, sizeof(from_addr)) != SOCKET_ERROR)
						{
							if (recvfrom(t_socket, t_command.data(), t_command.maxBuffer(), 0, (sockaddr*)&from_addr, &addr_len) != SOCKET_ERROR)
							{
								if (t_command.isCommand(C_GetDeviceType) && (t_command.getCommandType() < 0x03))
								{
									devType = std::string(t_command.getArgField(), t_command.getArgFieldLength());
								}
							}
						}
						ljhNS::closeSocket(t_socket);

						// 根据序列号创建设备对象并保存到全局列表中
						_mutex.lock();
						if (ljhNS::gGlobal.strDeviceMap.find(sn) == ljhNS::gGlobal.strDeviceMap.end())
						{
							ljhNS::gGlobal.strDeviceMap[sn] = new ljhNS::DeviceHandle(id);
						}
						ljhNS::gGlobal.numDeviceMap[id] = ljhNS::gGlobal.strDeviceMap[sn];
						
						// 填充设备列表信息
						if (*findCount < listCount)
						{
							deviceInfoList[*findCount].handle = id;
							memcpy(deviceInfoList[*findCount].sn, sn.c_str(), 13);
							deviceInfoList[*findCount].sn[13] = '\0';
							inet_ntop(AF_INET, &devIP, deviceInfoList[*findCount].ip, 32);
							inet_ntop(AF_INET, &locIP, deviceInfoList[*findCount].local_ip, 32);

							auto len = devType.size();
							if (len > 0)
							{
								memcpy(deviceInfoList[*findCount].type, devType.c_str(), len);
								deviceInfoList[*findCount].type[len] = '\0';
							}
						}
						++(*findCount);
						_mutex.unlock();

						// 填充设备信息
						auto handle = ljhNS::gGlobal.numDeviceMap[id];
						handle->_SN_ = sn;
						handle->deviceType = devType;
						handle->describe.clear();
						handle->localAddr = local_addr;
						handle->remoteAddr = from_addr;
						handle->recvAddr = from_addr;
						handle->recvAddr.sin_port = htons(DATA_PORT);

						// 粗略检测本机与设备的网段是否匹配
						if (reply[8] == 0x01)
						{
							if ((devIP & devMrak) != (locIP & devMrak))
							{
								char buffer[256];
								auto ipPtr1 = (unsigned char*)&locIP;
								auto ipPtr2 = (unsigned char*)&devIP;
								snprintf(buffer, 256, "The remote IP and the local IP are not in the same network segment and need to be adjusted to the same network segment. Local IP: %u.%u.%u.%u, Remote IP: %u.%u.%u.%u",
									ipPtr1[0], ipPtr1[1], ipPtr1[2], ipPtr1[3], ipPtr2[0], ipPtr2[1], ipPtr2[2], ipPtr2[3]);
								handle->describe = std::string(buffer);
							}
						}
						else
						{
							auto devip_c = (uint8_t*)(&devIP);
							if (((devip_c[0] == 0xA9) && (devip_c[1] == 0xFE)) && ((reply[3] != 0xA9) || (reply[2] != 0xFE)))
							{
								handle->describe = "The remote device has enabled DHCP protocol, but the computer has a static IP and needs to be adjusted to match it.";
							}
						}
					}
					else
					{
						LOG_ERROR_OUT("The SDK does not match the device firmware version.");
					}
				}
			}
			});
	}

	for (auto& var : thread_list)
	{
		var.join();
	}

	return LW_RETURN_OK;
}

LWReturnCode LWFindDevices(LWDeviceHandle *handleList, int32_t listCount, int32_t *filledCount)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	if (listCount < 1)
	{
		ljhNS::gGlobal.errorInfo = "The number of parameters exceeds the range.";
		return LW_RETURN_CUSTOM_ERROR;
	}

	ljhNS::gGlobal.numDeviceMap.clear();
    *filledCount = 0;

    std::vector<sockaddr_in> sockaddr_list;
	if (!ljhNS::getLocalAddrInfo(sockaddr_list))
	{
		ljhNS::gGlobal.errorInfo = ljhNS::getNetworkLastError();
		LOG_ERROR_OUT("%s", ljhNS::gGlobal.errorInfo.c_str());
		return LW_RETURN_NETWORK_ERROR;
	}

    std::mutex _mutex;
    std::vector<std::thread> thread_list;
    for (const auto& local_addr : sockaddr_list)
    {
        thread_list.emplace_back([&local_addr, &_mutex, &handleList, &listCount, &filledCount]()->void {
			// 广播地址
            sockaddr_in to_addr{};
            to_addr.sin_family = AF_INET;
            to_addr.sin_port = htons(COMMAND_PORT);
            to_addr.sin_addr.s_addr = INADDR_BROADCAST;
			// 发送广播消息
			ljhNS::CommandSFrame command{ C_Discovery };
			command.setVersion(0x03);
			SOCKET _socket = ljhNS::gSocketMap[local_addr.sin_addr.s_addr];
            if (sendto(_socket, command.data(), command.size(), MSG_NOSIGNAL, (sockaddr*)&to_addr, sizeof(to_addr)) == SOCKET_ERROR ||
                !ljhNS::setNetworkTimeout(_socket, 3000)
                    )
            {
				ljhNS::gGlobal.errorInfo = ljhNS::getNetworkLastError();
				LOG_ERROR_OUT("%s", ljhNS::gGlobal.errorInfo.c_str());
                return;
            }
			// 接收应答信息
            sockaddr_in from_addr{};
            socklen_t addr_len = sizeof(from_addr);
            while (true)
            {
                if (recvfrom(_socket, command.data(), command.maxBuffer(), 0, (sockaddr*)&from_addr, &addr_len) == SOCKET_ERROR) break;

				// 过滤掉IP为：66.66.66.66 的地址
                if (from_addr.sin_addr.s_addr == 1111638594U) continue;

				// 创建设备句柄对象
                if (command.isCommand(C_Discovery))
                {
                    if (command.getVersion() > 0x02)
                    {
						auto reply = (uint8_t*)(command.getArgField());

						uint32_t devIP = from_addr.sin_addr.s_addr;
						uint32_t locIP = local_addr.sin_addr.s_addr;
                        uint32_t devMrak = ntohl(*((uint32_t*)(reply + 13)));

						std::string sn(command.getArgField() + 17, 13);
						if (sn.find("DM") == std::string::npos) continue;

						LWDeviceHandle id = 0;
						auto idPtr = (char*)&id;
						memcpy(idPtr, &devIP, 4);
						memcpy(idPtr + 4, &locIP, 4);

						// 获取设备类型
						std::string devType;
						auto t_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
						ljhNS::setNetworkTimeout(t_socket, 1500);
						ljhNS::CommandSFrame t_command{ C_GetDeviceType };
						if (sendto(t_socket, t_command.data(), t_command.size(), MSG_NOSIGNAL, (sockaddr*)&from_addr, sizeof(from_addr)) != SOCKET_ERROR)
						{
							if (recvfrom(t_socket, t_command.data(), t_command.maxBuffer(), 0, (sockaddr*)&from_addr, &addr_len) != SOCKET_ERROR)
							{
								if (t_command.isCommand(C_GetDeviceType) && (t_command.getCommandType() < 0x03))
								{
									devType = std::string(t_command.getArgField(), t_command.getArgFieldLength());
								}
							}
						}
						ljhNS::closeSocket(t_socket);

						_mutex.lock();
						if (ljhNS::gGlobal.strDeviceMap.find(sn) == ljhNS::gGlobal.strDeviceMap.end())
						{
							ljhNS::gGlobal.strDeviceMap[sn] = new ljhNS::DeviceHandle(id);
						}
						ljhNS::gGlobal.numDeviceMap[id] = ljhNS::gGlobal.strDeviceMap[sn];
						if (*filledCount < listCount)
						{
							handleList[*filledCount] = id;
							++(*filledCount);
						}
						_mutex.unlock();
						
						auto handle = ljhNS::gGlobal.numDeviceMap[id];
						handle->_SN_ = sn;
						handle->deviceType = devType;
						handle->describe.clear();
						handle->localAddr = local_addr;
						handle->remoteAddr = from_addr;
						handle->recvAddr = from_addr;
						handle->recvAddr.sin_port = htons(DATA_PORT);
						
						if (reply[8] == 0x01)
						{
							if ((devIP & devMrak) != (locIP & devMrak))
							{
								char buffer[256];
								auto ipPtr1 = (unsigned char*)&locIP;
								auto ipPtr2 = (unsigned char*)&devIP;
								snprintf(buffer, 256, "The remote IP and the local IP are not in the same network segment and need to be adjusted to the same network segment. Local IP: %u.%u.%u.%u, Remote IP: %u.%u.%u.%u",
									ipPtr1[0], ipPtr1[1], ipPtr1[2], ipPtr1[3], ipPtr2[0], ipPtr2[1], ipPtr2[2], ipPtr2[3]);
								handle->describe = std::string(buffer);
							}
						}
						else
						{
							auto devip_c = (uint8_t*)(&devIP);
							if (((devip_c[0] == 0xA9) && (devip_c[1] == 0xFE)) && ((reply[3] != 0xA9) || (reply[2] != 0xFE)))
							{
								handle->describe = "The remote device has enabled DHCP protocol, but the computer has a static IP and needs to be adjusted to match it.";
							}
						}
                    }
                    else
					{
						LOG_ERROR_OUT("The SDK does not match the device firmware version.");
					}
                }
            }
		});
    }

    for (auto& var : thread_list)
    {
        var.join();
    }

    return LW_RETURN_OK;
}

LWReturnCode LWOpenDevice(LWDeviceHandle handle)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
    if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
        return LW_RETURN_HANDLE_MISMATCH;

    return ljhNS::gGlobal.numDeviceMap[handle]->OpenDevice();
}

LWReturnCode LWCloseDevice(LWDeviceHandle handle)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->CloseDevice();
}

LWReturnCode LWReconnectDevice(LWDeviceHandle handle, uint32_t t)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->ReconnectDevice(t);
}

LWReturnCode LWRebootDevice(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->RebootDevice();
}

LWReturnCode LWSaveConfigureInfo(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SaveConfigureInfo();
}

LWReturnCode LWRemoveConfigureInfo(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->RemoveConfigureInfo();
}

LWReturnCode LWRestoreFactoryConfigureInfo(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->RestoreFactoryConfigureInfo();
}

LWReturnCode LWStartStream(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->StartStream();
}

LWReturnCode LWStopStream(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->StopStream();
}

LWReturnCode LWHasRgbModule(LWDeviceHandle handle, bool* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->HasRgbModule(*value);
}

LWReturnCode LWSoftTrigger(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SoftTrigger();
}

LWReturnCode LWSetDataReceiveType(LWDeviceHandle handle, LWDataRecvType type)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDataReceiveType(type);
}

LWReturnCode LWSetTimeout(LWDeviceHandle handle, uint32_t t)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	ljhNS::gGlobal.numDeviceMap[handle]->timeout = t;

	return LW_RETURN_OK;
}

LWReturnCode LWSetTriggerMode(LWDeviceHandle handle, LWTriggerMode mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTriggerMode(mode);
}

LWReturnCode LWGetTriggerMode(LWDeviceHandle handle, LWTriggerMode* mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTriggerMode(*mode);
}

LWReturnCode LWSetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetExposureMode(sensorType, mode);
}

LWReturnCode LWGetExposureMode(LWDeviceHandle handle, LWSensorType sensorType, LWExposureMode* mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetExposureMode(sensorType, *mode);
}

LWReturnCode LWSetHDRMode(LWDeviceHandle handle, LWHDRMode mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetHDRMode(mode);
}

LWReturnCode LWGetHDRMode(LWDeviceHandle handle, LWHDRMode* mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetHDRMode(*mode);
}

LWReturnCode LWSetTransformDepthToRgbEnable(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

    return ljhNS::gGlobal.numDeviceMap[handle]->SetTransformDepthToRgbEnable(enable);
}

LWReturnCode LWSetTransformRgbToDepthEnable(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

    return ljhNS::gGlobal.numDeviceMap[handle]->SetTransformRgbToDepthEnable(enable);
}

LWReturnCode LWSetFrameRate(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetFrameRate(value);
}

LWReturnCode LWGetFrameRate(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetFrameRate(*value);
}

LWReturnCode LWSetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, const int32_t* exposureTimeArray, int32_t arraySize)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetExposureTime(sensorType, exposureTimeArray, arraySize);
}

LWReturnCode LWGetExposureTime(LWDeviceHandle handle, LWSensorType sensorType, int32_t* exposureTimeArray, int32_t arraySize, int32_t* filledCount)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetExposureTime(sensorType, exposureTimeArray, arraySize, *filledCount);
}

LWReturnCode LWSetTimeFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTimeFilterParams(param);
}

LWReturnCode LWGetTimeFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTimeFilterParams(*param);
}

LWReturnCode LWSetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTimeMedianFilterParams(param);
}

LWReturnCode LWGetTimeMedianFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTimeMedianFilterParams(*param);
}

LWReturnCode LWSetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSpatialFilterParams(param);
}

LWReturnCode LWGetSpatialFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSpatialFilterParams(*param);
}

LWReturnCode LWSetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetFlyingPixelsFilterParams(param);
}

LWReturnCode LWGetFlyingPixelsFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetFlyingPixelsFilterParams(*param);
}

LWReturnCode LWSetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetConfidenceFilterParams(param);
}

LWReturnCode LWGetConfidenceFilterParams(LWDeviceHandle handle, LWFilterParam* param)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetConfidenceFilterParams(*param);
}

LWReturnCode LWSetIRGMMGain(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetIRGMMGain(value);
}

LWReturnCode LWGetIRGMMGain(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetIRGMMGain(*value);
}

LWReturnCode LWSetRgbSensorGain(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorGain(value);
}

LWReturnCode LWGetRgbSensorGain(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorGain(*value);
}

LWReturnCode LWSetRgbSensorGamma(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorGamma(value);
}

LWReturnCode LWGetRgbSensorGamma(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorGamma(*value);
}

LWReturnCode LWSetRgbSensorBrightness(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorBrightness(value);
}

LWReturnCode LWGetRgbSensorBrightness(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorBrightness(*value);
}

LWReturnCode LWSetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorContrastRatio(value);
}

LWReturnCode LWGetRgbSensorContrastRatio(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorContrastRatio(*value);
}

LWReturnCode LWSetRgbSensorSaturation(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorSaturation(value);
}

LWReturnCode LWGetRgbSensorSaturation(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorSaturation(*value);
}

LWReturnCode LWSetRgbSensorWhiteBalance(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetRgbSensorWhiteBalance(value);
}

LWReturnCode LWGetRgbSensorWhiteBalance(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetRgbSensorWhiteBalance(*value);
}

LWReturnCode LWSetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo info)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetNetworkInfo(info);
}

LWReturnCode LWGetNetworkInfo(LWDeviceHandle handle, LWNetworkInfo* info)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetNetworkInfo(*info);
}

LWReturnCode LWSetDeviceNumber(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDeviceNumber(value);
}

LWReturnCode LWGetDeviceNumber(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetDeviceNumber(*value);
}

LWReturnCode LWSetDepthCompensateValue(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDepthCompensateValue(value);
}

LWReturnCode LWGetDepthCompensateValue(LWDeviceHandle handle, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetDepthCompensateValue(*value);
}

LWReturnCode LWSetHardTriggerFilterParams(LWDeviceHandle handle, int32_t t1, int32_t t2)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetHardTriggerFilterParams(t1, t2);
}

LWReturnCode LWGetHardTriggerFilterParams(LWDeviceHandle handle, int32_t* t1, int32_t* t2)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetHardTriggerFilterParams(*t1, *t2);
}

LWReturnCode LWGetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t* width, int32_t* height)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetResolution(sensorType, *width, *height);
}

LWReturnCode LWGetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam* intrinsicParam)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetIntrinsicParam(sensorType, *intrinsicParam);
}

LWReturnCode LWGetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam* extrinsicParam)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetExtrinsicParam(sensorType, *extrinsicParam);
}

LWReturnCode LWGetIMUExtrinsicParam(LWDeviceHandle handle, LWIMUExtrinsicParam* para)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetIMUExtrinsicParam(*para);
}

LWReturnCode LWGetIMUData(LWDeviceHandle handle, LWIMUData* imu_data)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetIMUData(*imu_data);
}

LWReturnCode LWSetIMUFrequency(LWDeviceHandle handle, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetIMUFrequency(value);
}

LWReturnCode LWGetDeviceSN(LWDeviceHandle handle, char* _SN_, int32_t bufferLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetDeviceSN(_SN_, bufferLen);
}

LWReturnCode LWGetDeviceType(LWDeviceHandle handle, char* type, int32_t bufferLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetDeviceType(type, bufferLen);
}

LWReturnCode LWGetTimeStamp(LWDeviceHandle handle, LWTimeStamp* t)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTimeStamp(*t);
}

LWReturnCode LWGetLibVersion(LWVersionInfo *version)
{
	LOG_INFO_OUT("");

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	version->major = MAJOR_VERSION;
	version->minor = MINOR_VERSION;
	version->patch = PATCH_VERSION;
	version->reserved = REVISION_VERSION;

	return LW_RETURN_OK;
}

LWReturnCode LWGetDeviceVersion(LWDeviceHandle handle, LWVersionInfo* fv, LWVersionInfo* dv)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetDeviceVersion(*fv, *dv);
}

LWReturnCode LWRegisterNetworkMonitoringCallback(void(*pCallback)(LWDeviceHandle handle, const char* error, void* pUserData), void* pUserData)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	ljhNS::pUserData1 = pUserData;
	ljhNS::networkAbnormalCallback = pCallback;

	return LW_RETURN_OK;
}

LWReturnCode LWRegisterFrameReadyCallback(void(*pCallback)(LWDeviceHandle handle, void* pUserData), void* pUserData)
{
	std::lock_guard<std::mutex> guard{ notReentryMutex };

	LOG_INFO_OUT("");

	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	ljhNS::pUserData2 = pUserData;
	ljhNS::frameReadyCallback = pCallback;

	return LW_RETURN_OK;
}

const char *LWGetReturnCodeDescriptor(LWReturnCode code)
{
	switch (code)
	{
	case LW_RETURN_OK:
		ljhNS::gGlobal.errorInfo = "Execution successful.";
		break;

	case LW_RETURN_TIMEOUT:
		ljhNS::gGlobal.errorInfo = "Execution timeout.";
		break;

	case LW_RETURN_NETWORK_ERROR:
		break;

	case LW_RETURN_DATA_TYPE_MISMATCH:
		ljhNS::gGlobal.errorInfo = "The data type that needs to be read does not match the data type being transmitted, so the data type cannot be read.";
		break;

	case LW_RETURN_DATA_NOT_UPDATED:
		ljhNS::gGlobal.errorInfo = "The LWGetFrameReady function was not called or the call failed.";
		break;

	case LW_RETURN_ARG_OUT_OF_RANGE:
		ljhNS::gGlobal.errorInfo = "Parameter settings are out of range.";
		break;

	case LW_RETURN_UNINITIALIZED:
		ljhNS::gGlobal.errorInfo = "The resource has not been initialized.";
		break;

	case LW_RETURN_UNOPENED:
		ljhNS::gGlobal.errorInfo = "The device is not turned on.";
		break;

	case LW_RETURN_COMMAND_UNDEFINED:
		ljhNS::gGlobal.errorInfo = "The command is undefined.The device does not yet have the ability to process this command.";
		break;

	case LW_RETURN_OUT_OF_MEMORY:
		ljhNS::gGlobal.errorInfo = "The incoming data buffer is out of memory.";
		break;

	case LW_RETURN_COMMAND_ERROR:
		ljhNS::gGlobal.errorInfo = "The command structure is incorrect.";
		break;

	case LW_RETURN_TYPE_NOT_EXIST:
		ljhNS::gGlobal.errorInfo = "Type error means that the type does not exist.";
		break;

	case LW_RETURN_TYPE_INPUT_ERROR:
		ljhNS::gGlobal.errorInfo = "Type input error.";
		break;

	case LW_RETURN_FIRMWARE_UPDATE_FAIL:
		break;

	case LW_RETURN_INDEX_NOT_EXIST:
		ljhNS::gGlobal.errorInfo = "The index value does not exist, please pass in the correct index value.";
		break;

	case LW_RETURN_THREAD_QUIT_TIMEOUT:
		ljhNS::gGlobal.errorInfo = "Thread exit timeout.";
		break;

	case LW_RETURN_FILE_LENGTH_ERROR:
		ljhNS::gGlobal.errorInfo = "The file length is incorrect.";
		break;

	case LW_RETURN_FILE_MD5_ERROR:
		ljhNS::gGlobal.errorInfo = "File MD5 verification failed.";
		break;

	case LW_RETURN_HANDLE_MISMATCH:
		ljhNS::gGlobal.errorInfo = "The device handle is not in the search list.";
		break;

	case LW_RETURN_DATA_SIZE_ERROR:
		ljhNS::gGlobal.errorInfo = "The size of the incoming data does not match.";
		break;

	case LW_RETURN_FILE_OPEN_ERROR:
		ljhNS::gGlobal.errorInfo = "File opening failed.";
		break;

	case LW_RETURN_NOT_SUPPORTED:
		ljhNS::gGlobal.errorInfo = "The current device is not supported.";
		break;

	case LW_RETURN_VERSION_ERROR:
		ljhNS::gGlobal.errorInfo = "Protocol version mismatch.";
		break;

	case LW_RETURN_CUSTOM_ERROR:
		break;

	case LW_RETURN_ACTION_INVALID:
		ljhNS::gGlobal.errorInfo = "标定失败，因为设备已经标定，如果想再次标定则须先取消标定。<The action is invalid and has been calibrated. Please cancel the calibration first and then recalibrate.>";
		break;

	case LW_RETURN_REGION_INVALID:
		ljhNS::gGlobal.errorInfo = "安防标定区域无效，其区域内有效点云数低于阈值，请重新选择更合适的区域。<The security calibration area is invalid, and the number of valid point clouds within its area is below the threshold. Please choose a more suitable area again.>";
		break;

	case LW_RETURN_AJSON_KEY_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件键值类型/个数不符合。";
		break;

	case LW_RETURN_AJSON_VALUE_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件键值错误（超范围）。";
		break;
	
	case LW_RETURN_AJSON_LOSS_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件键值错误（缺失）。";
		break;

	case LW_RETURN_AJSON_PORJ_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件项目名错误。";
		break;

	case LW_RETURN_AJSON_VERSION_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件版本号与固件支持版本不匹配。";
		break;

	case LW_RETURN_AJSON_KEY_NAME_ERROR:
		ljhNS::gGlobal.errorInfo = "安防Json配置文件键名错误。";
		break;

	case LW_RETURN_UNDEFINED_ERROR:
		ljhNS::gGlobal.errorInfo = " Undefined error.";
		break;

	default:
		ljhNS::gGlobal.errorInfo = "The return code exceeds the predetermined range.";
		break;
	}

	return ljhNS::gGlobal.errorInfo.c_str();
}

LWReturnCode LWGetFrameReady(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

    return ljhNS::gGlobal.numDeviceMap[handle]->GetFrameReady();
}

LWReturnCode LWGetFrame(LWDeviceHandle handle, LWFrameData *frame, LWFrameType type)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	if (!ljhNS::gGlobal.numDeviceMap[handle]->isReady) return LW_RETURN_DATA_NOT_UPDATED;

	switch (type)
	{
	case LW_DEPTH_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->depthEnable) return LW_RETURN_DATA_TYPE_MISMATCH;
		if(ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_POINTCLOUD_DEPTH_IR_RTY)
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pDepth.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data(), ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2);
		else
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pDepth.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data()+ ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 12, ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2);

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 2;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_USHORT;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pDepth.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		return LW_RETURN_OK;
	}

	case LW_AMPLITUDE_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->ampEnable) return LW_RETURN_DATA_TYPE_MISMATCH;
		memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pAmp.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data() + ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2, ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2);

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 2;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_USHORT;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pAmp.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		return LW_RETURN_OK;
	}

	case LW_IR_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->irEnable) return LW_RETURN_DATA_TYPE_MISMATCH;

		if(ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_IR_RTY)
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pGra.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data(), ljhNS::gGlobal.numDeviceMap[handle]->tofPixels);
		else if ((ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_DEPTH_IR_RTY) || (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_DEPTH_IR_RGB_RTY))
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pGra.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data() + ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2, ljhNS::gGlobal.numDeviceMap[handle]->tofPixels);
		else if (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_POINTCLOUD_IR_RTY)
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pGra.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data() + ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 12, ljhNS::gGlobal.numDeviceMap[handle]->tofPixels);
		else if (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_POINTCLOUD_DEPTH_IR_RTY)
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pGra.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data() + ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 14, ljhNS::gGlobal.numDeviceMap[handle]->tofPixels);
		else
		{
			auto amp_ptr	= (uint16_t*)(ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data() + ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 2);
			auto gra_ptr	= (uint8_t*)ljhNS::gGlobal.numDeviceMap[handle]->pGra.data();
			auto threshold	= ljhNS::gGlobal.numDeviceMap[handle]->irGMMGain;
			for (uint32_t i = 0, val; i < ljhNS::gGlobal.numDeviceMap[handle]->tofPixels; ++i, ++amp_ptr, ++gra_ptr)
			{
				val = (int)(*amp_ptr / 128.0 * threshold);
				*gra_ptr = (val < 255) ? val : 255;
			}
		}

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 1;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_UCHAR;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pGra.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		return LW_RETURN_OK;
	}

	case LW_POINTCLOUD_FRAME:
	{
		if ((ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_POINTCLOUD_IR_RTY)
			|| (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType == LW_POINTCLOUD_DEPTH_IR_RTY))
		{
			memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pPot.data(), ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data(), ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 12);
		}
		else if (ljhNS::gGlobal.numDeviceMap[handle]->depthEnable)
		{
			auto dis_ptr = (uint16_t*)(ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->data());
			auto pot_ptr = (LWVector3f*)(ljhNS::gGlobal.numDeviceMap[handle]->pPot.data());
			auto col = -ljhNS::gGlobal.numDeviceMap[handle]->tofInArg.cx;
			auto row = -ljhNS::gGlobal.numDeviceMap[handle]->tofInArg.cy;
			for (uint32_t i = 0; i < ljhNS::gGlobal.numDeviceMap[handle]->tofHeight; ++i)
			{
				col = -ljhNS::gGlobal.numDeviceMap[handle]->tofInArg.cx;
				for (uint32_t j = 0; j < ljhNS::gGlobal.numDeviceMap[handle]->tofWidth; ++j, ++pot_ptr, ++dis_ptr)
				{
					if (*dis_ptr < PCD_MAX_VALUE)
					{
						pot_ptr->x = col / ljhNS::gGlobal.numDeviceMap[handle]->tofInArg.fx * float(*dis_ptr);
						pot_ptr->y = row / ljhNS::gGlobal.numDeviceMap[handle]->tofInArg.fy * float(*dis_ptr);
						pot_ptr->z = *dis_ptr;
					}
					else
					{
						pot_ptr->x = 0;
						pot_ptr->y = 0;
						pot_ptr->z = 0;
					}
					col += 1.0f;
				}
				row += 1.0f;
			}
		}
		else
		{
			return LW_RETURN_DATA_TYPE_MISMATCH;
		}

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 12;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_VECTOR3F;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 12;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pPot.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		frame->pVariant->IMUModule.calMode	= ljhNS::gGlobal.numDeviceMap[handle]->cal_mode;
		frame->pVariant->IMUModule.eParam	= ljhNS::gGlobal.numDeviceMap[handle]->imuOutArg;

		return LW_RETURN_OK;
	}

	case LW_RGB_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			ljhNS::gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain RGB image data.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!ljhNS::gGlobal.numDeviceMap[handle]->rgbEnable) return LW_RETURN_DATA_TYPE_MISMATCH;
		memcpy(ljhNS::gGlobal.numDeviceMap[handle]->pRgb.data(), ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->data(), ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->size);

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->rgbWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->rgbHeight;
		frame->frameType	= type;
		frame->elemSize		= 3;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_RGB888;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->size;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pRgb.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->time;
		frame->temperature	= {};
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_RGB_TO_DEPTH_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			ljhNS::gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!ljhNS::gGlobal.numDeviceMap[handle]->isR2DEnable.load())
		{
			ljhNS::gGlobal.errorInfo = "The transform function from rgb to depth data must be enabled. Namely: LWSetTransformRgbToDepthEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_RGB_RTY
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_AMPLITUDE_RGB_RTY
			)
			return LW_RETURN_DATA_TYPE_MISMATCH;

		if (ljhNS::gGlobal.r2dTF)
		{
			std::unique_lock<std::mutex> lock{ ljhNS::gGlobal.numDeviceMap[handle]->r2dMutex };
			ljhNS::gGlobal.numDeviceMap[handle]->pR2D.memset_v(0);
			ljhNS::gGlobal.numDeviceMap[handle]->r2dRunCount = THREAD_POOL_SIZE;
			ljhNS::gGlobal.numDeviceMap[handle]->r2dFlag = 0xFFFFFFFF;
			ljhNS::gGlobal.numDeviceMap[handle]->r2dNotify.notify_all();
			if (!ljhNS::gGlobal.numDeviceMap[handle]->r2dNotify.wait_for(lock, std::chrono::milliseconds{ ljhNS::gGlobal.numDeviceMap[handle]->timeout }, [handle] { return (ljhNS::gGlobal.numDeviceMap[handle]->r2dRunCount == 0) || !ljhNS::gGlobal.initEnable.load(); }))
			{
				return LW_RETURN_TIMEOUT;
			}
			if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

			ljhNS::gGlobal.r2dTF = false;
		}
		
		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->tofWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->tofHeight;
		frame->frameType	= type;
		frame->elemSize		= 3;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_RGB888;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->tofPixels * 3;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pR2D.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->rgbCutNode->time;
		frame->temperature	= {};
		frame->pVariant		= nullptr;

		return LW_RETURN_OK;
	}

	case LW_DEPTH_TO_RGB_FRAME:
	case LW_IR_TO_RGB_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			ljhNS::gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!ljhNS::gGlobal.numDeviceMap[handle]->isD2REnable.load())
		{
			ljhNS::gGlobal.errorInfo = "The transform function from depth to rgb data must be enabled. Namely: LWSetTransformDepthToRgbEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if ((type & LW_IR_TO_RGB_FRAME) && (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_IR_RGB_RTY)) return LW_RETURN_DATA_TYPE_MISMATCH;
		if ((type & LW_DEPTH_TO_RGB_FRAME) 
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_RGB_RTY
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_IR_RGB_RTY
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_AMPLITUDE_RGB_RTY
			)
			return LW_RETURN_DATA_TYPE_MISMATCH;

		if (ljhNS::gGlobal.d2rTF)
		{
			std::unique_lock<std::mutex> lock{ ljhNS::gGlobal.numDeviceMap[handle]->d2rMutex };
			ljhNS::gGlobal.numDeviceMap[handle]->d2rInputImg.setTo(0);
			ljhNS::gGlobal.numDeviceMap[handle]->ir2rInputImg.setTo(0);
			ljhNS::gGlobal.numDeviceMap[handle]->d2rRunCount = THREAD_POOL_SIZE;
			ljhNS::gGlobal.numDeviceMap[handle]->d2rFlag = 0xFFFFFFFF;
			ljhNS::gGlobal.numDeviceMap[handle]->d2rNotify.notify_all();
			if(!ljhNS::gGlobal.numDeviceMap[handle]->d2rNotify.wait_for(lock, std::chrono::milliseconds{ ljhNS::gGlobal.numDeviceMap[handle]->timeout }, [handle] { return (ljhNS::gGlobal.numDeviceMap[handle]->d2rRunCount == 0) || !ljhNS::gGlobal.initEnable.load(); }))
			{
				return LW_RETURN_TIMEOUT;
			}
			if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

			ljhNS::gGlobal.d2rTF = false;
		}

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->rgbWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->rgbHeight;
		frame->frameType	= type;
		frame->elemSize		= (type & LW_DEPTH_TO_RGB_FRAME) ? 2 : 1;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pixelFormat	= (type & LW_DEPTH_TO_RGB_FRAME) ? LW_PIXEL_FORMAT_USHORT : LW_PIXEL_FORMAT_UCHAR;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= (type & LW_DEPTH_TO_RGB_FRAME) ? ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels * 2 : ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pFrameData	= (type & LW_DEPTH_TO_RGB_FRAME) ? (char*)ljhNS::gGlobal.numDeviceMap[handle]->d2rOutputImg.data : (char*)ljhNS::gGlobal.numDeviceMap[handle]->ir2rOutputImg.data;
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		return LW_RETURN_OK;
	}

	case LW_D2R_POINTCLOUD_FRAME:
	{
		if (!ljhNS::gGlobal.numDeviceMap[handle]->hasRgbModule.load())
		{
			ljhNS::gGlobal.errorInfo = "This device does not have an RGB module and cannot obtain data of this type.";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (!ljhNS::gGlobal.numDeviceMap[handle]->isD2REnable.load())
		{
			ljhNS::gGlobal.errorInfo = "The transform function from depth to rgb data must be enabled. Namely: LWSetTransformDepthToRgbEnable(true).";
			return LW_RETURN_CUSTOM_ERROR;
		}

		if (ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_RGB_RTY
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_IR_RGB_RTY
			&& ljhNS::gGlobal.numDeviceMap[handle]->dataRecvType != LW_DEPTH_AMPLITUDE_RGB_RTY
			) 
			return LW_RETURN_DATA_TYPE_MISMATCH;

		if (ljhNS::gGlobal.d2rTF) 
		{
			std::unique_lock<std::mutex> lock{ ljhNS::gGlobal.numDeviceMap[handle]->d2rMutex };
			ljhNS::gGlobal.numDeviceMap[handle]->d2rInputImg.setTo(0);
			ljhNS::gGlobal.numDeviceMap[handle]->ir2rInputImg.setTo(0);
			ljhNS::gGlobal.numDeviceMap[handle]->d2rRunCount = THREAD_POOL_SIZE;
			ljhNS::gGlobal.numDeviceMap[handle]->d2rFlag = 0xFFFFFFFF;
			ljhNS::gGlobal.numDeviceMap[handle]->d2rNotify.notify_all();
			if (!ljhNS::gGlobal.numDeviceMap[handle]->d2rNotify.wait_for(lock, std::chrono::milliseconds{ ljhNS::gGlobal.numDeviceMap[handle]->timeout }, [handle] { return (ljhNS::gGlobal.numDeviceMap[handle]->d2rRunCount == 0) || !ljhNS::gGlobal.initEnable.load(); }))
			{
				return LW_RETURN_TIMEOUT;
			}
			if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

			ljhNS::gGlobal.d2rTF = false;
		}

		auto dis_ptr = (uint16_t*)(ljhNS::gGlobal.numDeviceMap[handle]->d2rOutputImg.data);
		auto pot_ptr = (LWVector3f*)(ljhNS::gGlobal.numDeviceMap[handle]->pTPot.data());
		auto col = -ljhNS::gGlobal.numDeviceMap[handle]->rgbInArg.cx;
		auto row = -ljhNS::gGlobal.numDeviceMap[handle]->rgbInArg.cy;
		for (uint32_t i = 0; i < ljhNS::gGlobal.numDeviceMap[handle]->rgbHeight; ++i)
		{
			col = -ljhNS::gGlobal.numDeviceMap[handle]->rgbInArg.cx;
			for (uint32_t j = 0; j < ljhNS::gGlobal.numDeviceMap[handle]->rgbWidth; ++j, ++pot_ptr, ++dis_ptr)
			{
				if (*dis_ptr < PCD_MAX_VALUE)
				{
					pot_ptr->x = col / ljhNS::gGlobal.numDeviceMap[handle]->rgbInArg.fx * float(*dis_ptr);
					pot_ptr->y = row / ljhNS::gGlobal.numDeviceMap[handle]->rgbInArg.fy * float(*dis_ptr);
					pot_ptr->z = *dis_ptr;
				}
				else
				{
					pot_ptr->x = 0;
					pot_ptr->y = 0;
					pot_ptr->z = 0;
				}
				col += 1.0f;
			}
			row += 1.0f;
		}

		frame->width		= ljhNS::gGlobal.numDeviceMap[handle]->rgbWidth;
		frame->height		= ljhNS::gGlobal.numDeviceMap[handle]->rgbHeight;
		frame->frameType	= type;
		frame->elemSize		= 12;
		frame->total		= ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels;
		frame->pixelFormat	= LW_PIXEL_FORMAT_VECTOR3F;
		frame->frameIndex	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->serial;
		frame->bufferSize	= ljhNS::gGlobal.numDeviceMap[handle]->rgbPixels * 12;
		frame->pFrameData	= ljhNS::gGlobal.numDeviceMap[handle]->pTPot.data();
		frame->timestamp	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->time;
		frame->temperature	= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->temperature;
		frame->pVariant		= ljhNS::gGlobal.numDeviceMap[handle]->tofCutNode->variant;

		frame->pVariant->IMUModule.calMode = ljhNS::gGlobal.numDeviceMap[handle]->cal_mode;
		frame->pVariant->IMUModule.eParam = ljhNS::gGlobal.numDeviceMap[handle]->imuOutArg;

		return LW_RETURN_OK;
	}

	default:
		break;
	}

	return LW_RETURN_TYPE_NOT_EXIST;
}

LWReturnCode LWIMURotatePointCloud(LWFrameData* frame, float* rotate_mat)
{
	if (frame->frameType != LW_POINTCLOUD_FRAME && frame->frameType != LW_D2R_POINTCLOUD_FRAME) return LW_RETURN_TYPE_INPUT_ERROR;

	static float cal_mode_yaw = 0.0;
	static int MEMO_LENGTH = 24;
	static int TRSD_GYO_STABEL = 500; //degree/s*1000
	static int TRSD_ANGLE_STABEL = 5; //degree
	float gyo_x_mean = 0.0;
	float gyo_y_mean = 0.0;
	float gyo_z_mean = 0.0;
	static float gyo_x_mean_stable = 0.0;
	static float gyo_y_mean_stable = 0.0;
	static float gyo_z_mean_stable = 0.0;
	static float gyo_x_memo[24];
	static float gyo_y_memo[24];
	static float gyo_z_memo[24];
	static float roll_stable = 0.0;
	static float pitch_stable = 0.0;
	static float yaw_stable = 0.0;
	static float roll_mean = 0.0;
	static float pitch_mean = 0.0;
	static float yaw_mean = 0.0;
	static int index = 0;
	static int stable_flag = 0;
	static clock_t last_time = 0.0;
	static ljhNS::AutoMemoryManager src_mat_ptr(1600 * 1200 * 12);
	static ljhNS::AutoMemoryManager dst_mat_ptr(1600 * 1200 * 12);
	cv::Mat pc_mat_cv(3, frame->total, CV_32F, src_mat_ptr.data());
	cv::Mat pc_mat_result_cv(3, frame->total, CV_32F, dst_mat_ptr.data());
	//static cv::Mat src_mat_ptr(3, 1600*1200, CV_32F);
	//static cv::Mat dst_mat_ptr(3, 1600*1200, CV_32F);
	//cv::Mat pc_mat_cv(src_mat_ptr, cv::Rect(0, 0, frame->total, 3));
	//cv::Mat pc_mat_result_cv(dst_mat_ptr, cv::Rect(0, 0, frame->total, 3));

	int PC_NUM = frame->total;

	auto pc_ori = (LWVector3f*)frame->pFrameData;
	float* pc_x_ptr = reinterpret_cast<float*>(pc_mat_cv.data);
	float* pc_y_ptr = reinterpret_cast<float*>(pc_mat_cv.row(1).data);
	float* pc_z_ptr = reinterpret_cast<float*>(pc_mat_cv.row(2).data);
	for (int i = 0; i < PC_NUM; i++, pc_x_ptr++, pc_y_ptr++, pc_z_ptr++) {
		*pc_x_ptr = pc_ori[i].x;
		*pc_y_ptr = pc_ori[i].y;
		*pc_z_ptr = pc_ori[i].z;
	}

	float imu_yaw = frame->pVariant->IMUModule.imuData.yawAngle / 180 * PI;
	float imu_roll = frame->pVariant->IMUModule.imuData.rollAngle / 180 * PI;
	float imu_pitch = frame->pVariant->IMUModule.imuData.pitchAngle / 180 * PI;

	auto now_clock = clock();
	if (abs((double)now_clock - last_time) / CLOCKS_PER_SEC > 2) {
		roll_stable = imu_roll;
		pitch_stable = imu_pitch;
		yaw_stable = imu_yaw;
		stable_flag = 0;
	}
	last_time = now_clock;

	index = index >= MEMO_LENGTH - 1 ? 0 : index + 1;
	gyo_x_memo[index] = frame->pVariant->IMUModule.imuData.xAxisGyro;
	gyo_y_memo[index] = frame->pVariant->IMUModule.imuData.yAxisGyro;
	gyo_z_memo[index] = frame->pVariant->IMUModule.imuData.zAxisGyro;
	for (int i = 0; i < MEMO_LENGTH; i++) {
		gyo_x_mean += gyo_x_memo[i];
		gyo_y_mean += gyo_y_memo[i];
		gyo_z_mean += gyo_z_memo[i];
		//std::cout << "gyo_y_memo[" << i << "]:" << gyo_y_memo[i] << std::endl;
	}
	gyo_x_mean /= MEMO_LENGTH;
	gyo_y_mean /= MEMO_LENGTH;
	gyo_z_mean /= MEMO_LENGTH;

	if (abs(gyo_x_mean - gyo_x_mean_stable) > TRSD_GYO_STABEL || 	//动态则更新均值线
		abs(gyo_y_mean - gyo_y_mean_stable) > TRSD_GYO_STABEL ||
		abs(gyo_z_mean - gyo_z_mean_stable) > TRSD_GYO_STABEL) {
		gyo_x_mean_stable = gyo_x_mean;
		gyo_y_mean_stable = gyo_y_mean;
		gyo_z_mean_stable = gyo_z_mean;
		stable_flag = 0;
		//std::cout << "角速度动态中" << std::endl;
	}
	else {
		//std::cout << "imu_yaw:" << imu_yaw << "  imu_pitch:" << imu_pitch << "  imu_roll:" << imu_roll << std::endl;
		//std::cout << "yaw_stable:" << yaw_stable << "  pitch_stable:" << pitch_stable << "  roll_stable:" << roll_stable << std::endl;
		if (abs(imu_yaw - yaw_stable) > TRSD_ANGLE_STABEL || 		//静态则先判断是否处于角度是否稳定(包括重开算法间挪动设备造成角度断续)
			abs(imu_pitch - pitch_stable) > TRSD_ANGLE_STABEL ||
			abs(imu_roll - roll_stable) > TRSD_ANGLE_STABEL) {	//若否,则先粗更新角度稳定线
			roll_stable = imu_roll;
			pitch_stable = imu_pitch;
			yaw_stable = imu_yaw;
			stable_flag = 0;
			//std::cout << "欧拉角动态中   更新欧拉角_stable=欧拉角" << std::endl;
		}
		else {														//若角速度和角度都稳定一段时间
			//std::cout << "stable_flag:" << stable_flag << std::endl;
			if (stable_flag == 0) {
				roll_mean = 0.0;
				pitch_mean = 0.0;
				yaw_mean = 0.0;
			}
			if (stable_flag < MEMO_LENGTH / 2) {
				roll_mean += imu_roll;
				pitch_mean += imu_pitch;
				yaw_mean += imu_yaw;
				//std::cout << "稳定中 获取欧拉角数据中" << " stable_flag:" << stable_flag << std::endl;
			}
			else if (stable_flag == MEMO_LENGTH / 2) {
				roll_stable = roll_mean / float(MEMO_LENGTH / 2);
				pitch_stable = pitch_mean / float(MEMO_LENGTH / 2);
				yaw_stable = yaw_mean / float(MEMO_LENGTH / 2);
				//std::cout << "稳定中 获取欧拉角数据完成 计算均值 更新稳定欧拉角" << " stable_flag:" << stable_flag << std::endl;
				//std::cout << "yaw_stable:" << yaw_stable << "  pitch_stable:" << pitch_stable << "  roll_stable:" << roll_stable << std::endl;
			}
			else {
				imu_roll = roll_stable;
				imu_pitch = pitch_stable;
				imu_yaw = yaw_stable;
				//std::cout << "稳态 " << "roll_stable:" << roll_stable << "  pitch_stable:" << pitch_stable << " yaw_stable:" << yaw_stable << std::endl;
			}
			stable_flag++;
		}
	}

	//transform left coordinate to right coordinate
	imu_pitch = -imu_pitch;
	imu_yaw = -imu_yaw;

	//std::cout << "4" << std::endl;

	Eigen::Matrix3d RX_left = algNS::getRX_left(imu_roll);
	Eigen::Matrix3d RY_left = algNS::getRY_left(imu_pitch);
	Eigen::Matrix3d RZ_left = algNS::getRZ_left(imu_yaw);

	Eigen::Matrix3d R = RZ_left * RY_left * RX_left;
	Eigen::Matrix3d DTrans;
	DTrans << -1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	R = DTrans * R * DTrans;

	Eigen::Matrix3d RX_right;
	Eigen::Matrix3d RZ_right;
	Eigen::Matrix3d Re;
	if (frame->pVariant->IMUModule.calMode == 1) {
		//Get roate matrix based on reference coordinate
		RX_right = algNS::getRX_right(PI / 2);
		RZ_right = algNS::getRZ_right(PI);

		Re = RX_right * RZ_right * R;
		//std::cout << "In judge mode 1" << std::endl;
	}
	else if (frame->pVariant->IMUModule.calMode == 2) {
		RX_right = algNS::getRX_right(PI / 2);
		Re = RX_right * R;
		//std::cout << "In judge mode 2" << std::endl;
	}
	else if (frame->pVariant->IMUModule.calMode == 0) {
		ljhNS::gGlobal.errorInfo = "wait for determining cal_mode";
		// std::cout << ljhNS::gGlobal.errorInfo << std::endl;
		return LW_RETURN_CUSTOM_ERROR;
	}

	cv::Mat Re_cv(3, 3, CV_32F);
	for (int row = 0; row < Re_cv.rows; row++) {
		for (int col = 0; col < Re_cv.cols; col++) {
			Re_cv.at<float>(row, col) = (float)Re(row, col);
		}
	}

	cv::Mat R_total = Re_cv * cv::Mat(3, 3, CV_32FC1, &frame->pVariant->IMUModule.eParam);
	pc_mat_result_cv = R_total * pc_mat_cv;

	pc_x_ptr = reinterpret_cast<float*>(pc_mat_result_cv.data);
	pc_y_ptr = reinterpret_cast<float*>(pc_mat_result_cv.data) + PC_NUM;
	pc_z_ptr = reinterpret_cast<float*>(pc_mat_result_cv.data) + 2 * PC_NUM;
	for (int i = 0; i < PC_NUM; i++, pc_x_ptr++, pc_y_ptr++, pc_z_ptr++) {
		pc_ori[i].x = *pc_x_ptr;
		pc_ori[i].y = *pc_y_ptr;
		pc_ori[i].z = *pc_z_ptr;
	}

	if (rotate_mat != nullptr) 
	{
		int R_total_size = R_total.rows * R_total.cols;
		auto* R_total_ptr = reinterpret_cast<float*>(R_total.data);
		for (int i = 0; i < R_total_size; i++) {
			*rotate_mat = *R_total_ptr;
			rotate_mat++;
			R_total_ptr++;
		}
	}

	return LW_RETURN_OK;
}

LWReturnCode LWSavePointCloudAsPCDFile(const char* filename, const LWFrameData* frame, bool binary_mode)
{
	if ((frame->frameType != LW_POINTCLOUD_FRAME) 
		&& (frame->frameType != LW_D2R_POINTCLOUD_FRAME)
		) 
		return LW_RETURN_TYPE_INPUT_ERROR;

	std::ofstream file;

	binary_mode ? file.open(filename, std::ios::binary) : file.open(filename);
	if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;

	auto N = frame->total;
	file << "# .PCD v0.7 - Point Cloud Data file format\n";
	file << "VERSION 0.7\n";
	file << "FIELDS x y z\n";
	file << "SIZE 4 4 4\n";
	file << "TYPE F F F\n";
	file << "COUNT 1 1 1\n";
	file << "WIDTH " << frame->width << "\n";
	file << "HEIGHT " << frame->height << "\n";
	file << "VIEWPOINT 0 0 0 1 0 0 0\n";
	file << "POINTS " << N << "\n";

	if (binary_mode)
	{
		file << "DATA binary\n";
		file.write(frame->pFrameData, frame->bufferSize);
	}
	else
	{
		file << "DATA ascii\n";
		auto ptr_data = (LWVector3f*)frame->pFrameData;

		for (uint32_t i = 0; i < N; ++i, ++ptr_data)
		{
			file << ptr_data->x << " ";
			file << ptr_data->y << " ";
			file << ptr_data->z << "\n";
		}
	}

	file.close();

	return LW_RETURN_OK;
}

LWReturnCode LWSavePointCloudAsPLYFile(const char* filename, const LWFrameData* frame, bool binary_mode)
{
	if ((frame->frameType != LW_POINTCLOUD_FRAME) 
		&& (frame->frameType != LW_D2R_POINTCLOUD_FRAME)
		) 
		return LW_RETURN_TYPE_INPUT_ERROR;

	std::ofstream file;

	binary_mode ? file.open(filename, std::ios::binary) : file.open(filename);
	if (!file.is_open()) return LW_RETURN_FILE_OPEN_ERROR;

	uint32_t N = frame->total;

	file << "ply\n";
	binary_mode ? (file << "format binary_little_endian 1.0\n") : (file << "format ascii 1.0\n");
	file << "comment This is a point cloud file.\n";
	file << "element vertex " << N << "\n";
	file << "property float x\n";
	file << "property float y\n";
	file << "property float z\n";
	file << "end_header\n";

	if (binary_mode)
	{
		file.write(frame->pFrameData, frame->bufferSize);
	}
	else
	{
		auto ptr_data = (float*)frame->pFrameData;
		for (uint32_t i = 0; i < N; ++i)
		{
			file << *ptr_data++ << " ";
			file << *ptr_data++ << " ";
			file << *ptr_data++ << "\n";
		}
	}

	file.close();

	return LW_RETURN_OK;
}

LWReturnCode LWSaveDataAsCSVFile(const char* filename, const LWFrameData* frame)
{
	std::ofstream file;
	file.open(filename);
	if (!file.fail())
	{
		switch (frame->frameType)
		{
		case LW_DEPTH_FRAME:
		case LW_AMPLITUDE_FRAME:
		case LW_DEPTH_TO_RGB_FRAME:
		{
			auto* ptr = (uint16_t*)frame->pFrameData;
			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << *ptr;
				}
				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

        case LW_POINTCLOUD_FRAME:
        case LW_D2R_POINTCLOUD_FRAME:
		{
			auto* ptr = (LWVector3f*)frame->pFrameData;
			for (uint32_t i = 0, n = frame->total; i < n; ++i, ++ptr)
			{
				file << ptr->x << ",";
				file << ptr->y << ",";
				file << ptr->z << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		case LW_RGB_FRAME:
		case LW_RGB_TO_DEPTH_FRAME:
		{
			auto* ptr = (LWRGB888Pixel*)frame->pFrameData;

			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << int(ptr->r);
					file << "," << int(ptr->g);
					file << "," << int(ptr->b);
				}

				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		case LW_IR_FRAME:
		case LW_IR_TO_RGB_FRAME:
		{
			auto* ptr = (uint8_t*)frame->pFrameData;
			for (int32_t row = 0; row < frame->height; ++row)
			{
				for (int32_t col = 0; col < frame->width; ++col, ++ptr)
				{
					if (col > 0) file << ",";
					file << int(*ptr);
				}
				file << "\n";
			}

			file.close();
			return LW_RETURN_OK;
		}

		default:
			return LW_RETURN_TYPE_INPUT_ERROR;
		}

	}

	return LW_RETURN_FILE_OPEN_ERROR;
}

LWReturnCode LWSaveRgbAsImageFile(const char* filename, const LWFrameData* frame)
{
	if ((frame->frameType & LW_RGB_FRAME) || (frame->frameType & LW_RGB_TO_DEPTH_FRAME))
	{
		cv::Mat src(frame->height, frame->width, CV_8UC3, frame->pFrameData);
		cv::Mat dst;
		cv::cvtColor(src, dst, cv::COLOR_RGB2BGR);

		if (cv::imwrite(filename, dst)) return LW_RETURN_OK;

		ljhNS::gGlobal.errorInfo = "The cv::imwrite function failed to execute.";

		return LW_RETURN_CUSTOM_ERROR;
	}

	return LW_RETURN_TYPE_INPUT_ERROR;
}

LWReturnCode LWUpdateFirmware(LWDeviceHandle handle, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->UpdateFirmware(filename);
}

LWReturnCode LWUpdateFirmware1(const char* ip, const char* filename)
{
	LOG_INFO_OUT("<%s>", ip);
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;

	std::string password = "dm-zgyfjch";
	if (std::string(filename).find("dm-se") != std::string::npos)
	{
		password = "dm-se-zgyfjch";
	}

	UpdateToolTask task(ip, 22, "root", password, filename, 3000);
	UpdateResultType result = task.processor();

	if (result == RESULT_OK)
	{
		return LW_RETURN_OK;
	}
	else
	{
		ljhNS::gGlobal.errorInfo = result.desc;
		return LW_RETURN_FIRMWARE_UPDATE_FAIL;
	}
}

LWReturnCode LWSetOutputDO(LWDeviceHandle handle, int32_t channel, int32_t value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetOutputDO(channel, value);
}

LWReturnCode LWGetOutputDO(LWDeviceHandle handle, int32_t channel, int32_t* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetOutputDO(channel, *value);
}

LWReturnCode LWHasPalletIdentifyModule(LWDeviceHandle handle, bool* value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->HasPalletIdentifyModule(*value);
}

LWReturnCode LWUploadRKNNFile(LWDeviceHandle handle, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->UploadRKNNFile(filename);
}

LWReturnCode LWSetPalletConfigureFile(LWDeviceHandle handle, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetPalletConfigureFile(filename);
}

LWReturnCode LWSetPalletConfigureFileFromBuffer(LWDeviceHandle handle, const char* buf, int32_t bufLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetPalletConfigureFileFromBuffer(buf, bufLen);
}

LWReturnCode LWGetPalletConfigureFile(LWDeviceHandle handle, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetPalletConfigureFile(filename);
}

LWReturnCode LWGetPalletConfigureFileToBuffer(LWDeviceHandle handle, char* buf, int32_t bufLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetPalletConfigureFileToBuffer(buf, bufLen);
}

LWReturnCode LWSetPalletIdentifyType(LWDeviceHandle handle, const char* type)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetPalletIdentifyType(type);
}

LWReturnCode LWGetPalletIdentifyType(LWDeviceHandle handle, char* type, int32_t len)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetPalletIdentifyType(type, len);
}

LWReturnCode LWSetPalletIdentifyEnable(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetPalletIdentifyEnable(enable);
}

LWReturnCode LWGetPalletIdentifyEnable(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetPalletIdentifyEnable(*enable);
}

LWReturnCode LWSetTRSDsimilarMax(LWDeviceHandle handle, float val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTRSDsimilarMax(val);
}

LWReturnCode LWGetTRSDsimilarMax(LWDeviceHandle handle, float* val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTRSDsimilarMax(*val);
}

LWReturnCode LWSetTRSDpstMax(LWDeviceHandle handle, float val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTRSDpstMax(val);
}

LWReturnCode LWGetTRSDpstMax(LWDeviceHandle handle, float* val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTRSDpstMax(*val);
}

LWReturnCode LWSetCutHeight(LWDeviceHandle handle, int32_t val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetCutHeight(val);
}

LWReturnCode LWGetCutHeight(LWDeviceHandle handle, int32_t* val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetCutHeight(*val);
}


#ifdef LW_INTERNAL_API

LWReturnCode LWSendFile(LWDeviceHandle handle, const char* fullname, LWFileType type)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SendFile(fullname, type);
}

LWReturnCode LWSetDeviceSN(LWDeviceHandle handle, const char* _SN_, int size)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDeviceSN(_SN_, size);
}

LWReturnCode LWSendOperateCommand(LWDeviceHandle handle, const char* comstr, int size)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SendOperateCommand(comstr, size);
}

LWReturnCode LWSetDRNU(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDRNU(enable);
}

LWReturnCode LWSetBinningMode(LWDeviceHandle handle, LWBinningMode mode)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetBinningMode(mode);
}

LWReturnCode LWSetResolution(LWDeviceHandle handle, LWSensorType sensorType, int32_t width, int32_t height)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetResolution(sensorType, width, height);
}

LWReturnCode LWSetDistortionCalibration(LWDeviceHandle handle, LWSensorType sensorType, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDistortionCalibration(sensorType, enable);
}

LWReturnCode LWSetLaserWorkFrequency(LWDeviceHandle handle, const uint8_t* arr, int size)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetLaserWorkFrequency(arr, size);
}

LWReturnCode LWSetAutoExposureDefaultValue(LWDeviceHandle handle, uint16_t val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetAutoExposureDefaultValue(val);
}

LWReturnCode LWSetIntrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorIntrinsicParam para)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetIntrinsicParam(sensorType, para);
}

LWReturnCode LWSetExtrinsicParam(LWDeviceHandle handle, LWSensorType sensorType, LWSensorExtrinsicParam para)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetExtrinsicParam(sensorType, para);
}

LWReturnCode LWSetIMUExtrinsicParam(LWDeviceHandle handle, LWIMUExtrinsicParam para)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetIMUExtrinsicParam(para);
}

LWReturnCode LWSetTemperatureCompensation(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTemperatureCompensation(enable);
}

LWReturnCode LWGetTemperatureCompensation(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetTemperatureCompensation(*enable);
}

LWReturnCode LWSetTemperatureParams(LWDeviceHandle handle, LWTemperatureParams val)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetTemperatureParams(val);
}

LWReturnCode LWSetLaserEnableStatus(LWDeviceHandle handle, uint32_t flag)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetLaserEnableStatus(flag);
}

LWReturnCode LWGetLaserEnableStatus(LWDeviceHandle handle, uint32_t* flag)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetLaserEnableStatus(*flag);
}

LWReturnCode LWSetDataAlignEnable(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetDataSyncEnable(enable);
}

#endif


#ifdef LW_ZHONGRILONG_PROJ

LW_C_API LWReturnCode LWSynchronizeDeviceSystemTime(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SynchronizeDeviceSystemTime();
}

LW_C_API LWReturnCode LWHasSecurityAbility(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->HasSecurityAbility(*enable);
}

LW_C_API LWReturnCode LWGetSecurityEnable(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityEnable(*enable);
}

LW_C_API LWReturnCode LWSetSecurityCalibrationParams(LWDeviceHandle handle, const int32_t* _array, int32_t size)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSecurityCalibrationParams(_array, size);
}

LW_C_API LWReturnCode LWSecurityCancelCalibration(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SecurityCancelCalibration();
}

LW_C_API LWReturnCode LWGetSecurityCalibrationEnable(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityCalibrationEnable(*enable);
}

LW_C_API LWReturnCode LWSetSecurityAxialAdjustment(LWDeviceHandle handle, int flag, float value)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSecurityAxialAdjustment(flag, value);
}

LW_C_API LWReturnCode LWResetSecurityConfigure(LWDeviceHandle handle)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->ResetSecurityConfigure();
}

LW_C_API LWReturnCode LWGetSecurityCalibrationMatrixParams(LWDeviceHandle handle, float* mat, int32_t* size)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityCalibrationMatrixParams(mat, size);
}

LW_C_API LWReturnCode LWGetSecurityZHSAEnable(LWDeviceHandle handle, bool* enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityZHSAEnable(*enable);
}

LW_C_API LWReturnCode LWSetSecurityZHSAEnable(LWDeviceHandle handle, bool enable)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSecurityZHSAEnable(enable);
}

LW_C_API LWReturnCode LWGetSecurityConfigFile(LWDeviceHandle handle, int type, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityConfigFile(type, filename);
}

LW_C_API LWReturnCode LWSetSecurityConfigFile(LWDeviceHandle handle, int type, const char* filename)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSecurityConfigFile(type, filename);
}

LW_C_API LWReturnCode LWGetSecurityConfigFileToBuffer(LWDeviceHandle handle, int type, char* buffer, int32_t bufLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->GetSecurityConfigFileToBuffer(type, buffer, bufLen);
}

LW_C_API LWReturnCode LWSetSecurityConfigFileFromBuffer(LWDeviceHandle handle, int type, const char* buffer, int32_t bufLen)
{
	if (!ljhNS::gGlobal.initEnable.load()) return LW_RETURN_UNINITIALIZED;
	if (ljhNS::gGlobal.numDeviceMap.find(handle) == ljhNS::gGlobal.numDeviceMap.end())
		return LW_RETURN_HANDLE_MISMATCH;

	return ljhNS::gGlobal.numDeviceMap[handle]->SetSecurityConfigFileFromBuffer(type, buffer, bufLen);
}


#endif //LW_ZHONGRILONG_PROJ
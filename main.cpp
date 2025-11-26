#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <thread>
#include "LWDMApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>



void showMat_Thre(cv::Mat input, int Thre, int waitKey = 50) {
	// 将 CV_32F 映射到 CV_8U 范围
	cv::Mat mappedMat, tempMat;
	tempMat = input.clone();
	tempMat.setTo(0, tempMat > Thre);
	cv::normalize(tempMat, mappedMat, 0, 255, cv::NORM_MINMAX, CV_8U);
	//tempMat.convertTo(mappedMat, CV_8UC1, 255.0 / 6000);

	// 创建灰度图像
	cv::Mat grayscaleImage(mappedMat.size(), CV_8U);

	// 复制数据到灰度图像
	mappedMat.copyTo(grayscaleImage);

	cv::namedWindow("Grayscale Visualization", 0);
	cv::resizeWindow("Grayscale Visualization", 640, 480);

	// 显示可视化结果
	cv::imshow("Grayscale Visualization", grayscaleImage);
	cv::waitKey(waitKey);

	return;
}


int main()
{
	//auto file = fopen("C:/Users/12267/Desktop/DATA/tempf/3-rgb_save.rgb", "rb");
	//auto data = new char[1200*1600*2];
	//fread(data, 3840000, 1, file);

	////cv::Mat src(1200, 1600, CV_8UC2, data);
	//cv::Mat dst;
	//cv::cvtColor(cv::Mat(1200, 1600, CV_8UC2, data), dst, cv::COLOR_BGR5652BGR);

	//cv::namedWindow("rgb", 0);
	//cv::resizeWindow("rgb", 640, 480);

	//cv::imshow("rgb", dst);

	//cv::waitKey(0);

	//delete[] data;

	//return 0;

	// SDK资源初始化
	auto ret = LWInitializeResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWInitializeResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}
	printf("Device initialization successful.\n\n");

	// 获取SDK版本信息
	LWVersionInfo version;
	LWGetLibVersion(&version);
	printf("SDK version: %d.%d.%d.%d\n\n", version.major, version.minor, version.patch, version.reserved);

	// 查找设备
	LWDeviceHandle handleList[5];
	LWDeviceInfo deviceInfoList[5];
	int32_t findCount = 0;
	int32_t index = 0;
	//ret = LWFindDevices(handleList, 5, &findCount);
	ret = LWGetDeviceInfoList(deviceInfoList, 5, &findCount);
	if (ret != LW_RETURN_OK)
	{
		printf("LWFindDevices function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}
	if (findCount < 1)
	{
		printf("No device found.");
		system("pause");
		return 0;
	}

	for (int i = 0; i < findCount; i++)
	{
		printf("设备编号: %d, 设备 SN: %s, 设备 Type: %s, 设备 IP: %s, 本机 IP: %s, 设备句柄: %llu\n",
			i,
			deviceInfoList[i].sn, 
			deviceInfoList[i].type, 
			deviceInfoList[i].ip,
			deviceInfoList[i].local_ip,
			deviceInfoList[i].handle
		);

		handleList[i] = deviceInfoList[i].handle;
	}

	if (findCount > 1)
	{
		printf("\n总共查找到 %d 台设备，请问你要操作设备编号？", findCount);
		std::ignore = scanf("%d", &index);
	}

	// 打开设备
	ret = LWOpenDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWOpenDevice function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	LWSetDataReceiveType(handleList[index], LWDataRecvType::LW_DEPTH_RGB_RTY);

	//ret = LWUpdateFirmware(handleList[index], "C:\\Users\\12267\\Desktop\\DATA\\update_packets\\dm_updata_enc_V1.3.1.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWOpenDevice function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
	//	system("pause");
	//	return 0;
	//}

	/*
	// 设置滤波
	printf("请设置相关滤波，第一个非负整数代表开关(0表示关，1表示开)，第二个非负整数代表阈值。\n注意：前后数字须用空格隔开，键入回车键表示输入完成\n\n");
	LWFilterParam param{false, 5, 100};
	printf("请设置空间滤波：");
	int Bval = 0;
	std::ignore = scanf("%d %d", &Bval, &param.threshold);
	param.enable = Bval;
	ret = LWSetSpatialFilterParams(handleList[index], param);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetSpatialFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("请设置时间均值滤波：");
	std::ignore = scanf("%d %d", &Bval, &param.threshold);
	param.enable = Bval;
	ret = LWSetTimeFilterParams(handleList[index], param);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("请设置时间中值滤波：");
	std::ignore = scanf("%d %d", &Bval, &param.threshold);
	param.enable = Bval;
	ret = LWSetTimeMedianFilterParams(handleList[index], param);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeMedianFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("请设置飞点滤波：");
	std::ignore = scanf("%d %d", &Bval, &param.threshold);
	param.enable = Bval;
	ret = LWSetFlyingPixelsFilterParams(handleList[index], param);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFlyingPixelsFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("请设置置信度滤波：");
	std::ignore = scanf("%d %d", &Bval, &param.threshold);
	param.enable = Bval;
	ret = LWSetConfidenceFilterParams(handleList[index], param);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetConfidenceFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("\n请设置深度数据映射到RGB的使能开关(0表示关，1表示开)：");
	std::ignore = scanf("%d", &Bval);
	ret = LWSetTransformDepthToRgbEnable(handleList[index], Bval);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTransformDepthToRgbEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	printf("\n请设置RGB数据映射到深度的使能开关(0表示关，1表示开)：");
	std::ignore = scanf("%d", &Bval);
	ret = LWSetTransformRgbToDepthEnable(handleList[index], Bval);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTransformRgbToDepthEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 设置HDR模式
	printf("\n请设置HDR模式\n---双频非HDR模式: 0\n---单频HDR模式: 1\n---双频HDR模式: 2\n---单频高精度HDR模式: 3\n---双频高精度HDR模式: 4\n---单频非HDR模式: 241\n你的选择是: ");
	uint32_t mode = 0;
	std::ignore = scanf("%u", &mode);
	ret = LWSetHDRMode(handleList[index], LWHDRMode(mode));
	if (ret != LW_RETURN_OK)
	{
		printf("\LWSetHDRMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 设置积分时间
	int32_t et[3] = { 2000 };
	printf("\n请设置3个曝光时间（例如：1000 150 20）：");
	std::ignore = scanf("%d %d %d", &et[0], &et[1], &et[2]);
	ret = LWSetExposureTime(handleList[index], LW_TOF_SENSOR, et, 1);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetExposureTime function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 设置帧率
	printf("\n请设置帧率：");
	int val = 0;
	std::ignore = scanf("%d", &val);
	ret = LWSetFrameRate(handleList[index], val);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}
	
	// 设置判定阈值
	//printf("请设置判定阈值：");
	//int Tval = 0;
	//std::ignore = scanf("%d", &Tval);

	std::string filename;
	std::ofstream file;
	std::time_t timestamp = std::time(nullptr);
	// 将时间戳转换为本地时间的 tm 结构体
	auto local_time = std::localtime(&timestamp);
	std::ostringstream oss;
	oss << std::put_time(local_time, "%Y-%m-%d %H-%M-%S+");
	filename += oss.str();
	//filename += ("温度阈值" + std::to_string(Tval));
	filename += ("帧率" + std::to_string(val));
	if (mode == 0) filename += "fps+双频非HDR+";
	else if (mode == 1) filename += "fps+单频HDR+";
	else if (mode == 2) filename += "fps+双频HDR+";
	else if (mode == 3) filename += "fps+单频高精度HDR+";
	else if (mode == 4) filename += "fps+双频高精度HDR+";
	else filename += "fps+单频非HDR+";
	filename += (std::to_string(et[0]) + "us.csv");
	
	// 设置数据接收类型
	printf("\n设置数据传输类型\n---IR: 0\n---深度+幅度+RGB: 1\n---RGB: 2\n---深度+IR: 3\n---点云+IR: 4\n---深度+RGB: 5\n你的选择是: ");
	LWDataRecvType type = LWDataRecvType::LW_DEPTH_AMPLITUDE_RGB_RTY;
	std::ignore = scanf("%d", &val);
	if (val == 0) type = LWDataRecvType::LW_IR_RTY;
	else if (val == 1) type = LWDataRecvType::LW_DEPTH_AMPLITUDE_RGB_RTY;
	else if (val == 2) type = LWDataRecvType::LW_RGB_RTY;
	else if (val == 3) type = LWDataRecvType::LW_DEPTH_IR_RTY;
	else if (val == 4) type = LWDataRecvType::LW_POINTCLOUD_IR_RTY;
	else type = LWDataRecvType::LW_DEPTH_RGB_RTY;
	ret = LWSetDataReceiveType(handleList[index], type);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetDataReceiveType function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 设置触发模式
	ret = LWSetTriggerMode(handleList[index], LWTriggerMode::LW_TRIGGER_ACTIVE);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetDataReceiveType function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 打开文件
	file.open(filename);
	if (file.fail())
	{
		printf("打开文件失败\n");
		system("pause");
		return 0;
	}
	//file << "芯片温度,激光器温度1,激光器温度2,中心幅值,帧率\n";
	file << "芯片温度,激光器温度1,激光器温度2,帧率\n";
*/
	// 开启数据流
	//LWSetDataReceiveType(handleList[index], LWDataRecvType::LW_DEPTH_RGB_RTY);
	ret = LWStartStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStartStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}
/*
	printf("\n设置数据接收类型\n---IR: 0\n---深度1\n---点云: 2\n---幅度: 3\n---RGB: 4\n---深度toRGB: 5\n---RGBto深度: 6\n---IRtoRGB: 7\n你的选择是: ");
	LWFrameType type1;
	std::ignore = scanf("%d", &val);
	if (val == 0) type1 = LWFrameType::LW_IR_FRAME;
	else if (val == 1) type1 = LWFrameType::LW_DEPTH_FRAME;
	else if (val == 2) type1 = LWFrameType::LW_POINTCLOUD_FRAME;
	else if (val == 3) type1 = LWFrameType::LW_AMPLITUDE_FRAME;
	else if (val == 4) type1 = LWFrameType::LW_RGB_FRAME;
	else if (val == 5) type1 = LWFrameType::LW_DEPTH_TO_RGB_FRAME;
	else if (val == 6) type1 = LWFrameType::LW_RGB_TO_DEPTH_FRAME;
	else if (val == 7) type1 = LWFrameType::LW_IR_TO_RGB_FRAME;
	else
	{
		printf("Input type error!\n\n");
		system("pause");
		return 0;
	}

	printf("\n请设置帧率刷新间隔(分钟)：");
	float temp = 0;
	std::ignore = scanf("%f", &temp);
	int64_t interval = temp * 60;
*/
	// 数据读取
	printf("\n\n");
	std::this_thread::sleep_for(std::chrono::seconds(3));
	LWFrameData frame;
	int64_t t0 = 0;
	int64_t t1 = 0;
	int64_t fps = 0;
	int64_t conut = 0;
	while (true)
	{
		ret = LWGetFrameReady(handleList[index]);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
			continue;
		}

		ret = LWGetFrame(handleList[index], &frame, LW_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_DEPTH_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_IR_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_POINTCLOUD_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_D2R_POINTCLOUD_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_POINTCLOUD_FRAME);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrame function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
			break;
		}
		
		auto imageMat = cv::Mat(1200, 1600, CV_8UC3, frame.pFrameData);
		cv::namedWindow("colorImageWindow", 0);
		cv::resizeWindow("colorImageWindow", 800, 600);
		cv::imshow("colorImageWindow", imageMat);
		cv::waitKey(10);

		++conut;
		if (t0 == frame.timestamp.tv_sec) continue;

		//ret = LWGetFrameRate(handleList[index], &val);
		//if (ret != LW_RETURN_OK)
		//{
		//	printf("LWGetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		//	break;
		//}
		

		//if ((frame.timestamp.tv_sec - t1) % interval == 0)
		//{
		//	fps = conut / (frame.timestamp.tv_sec - t0);
		//	printf(//"芯片温度：%f	激光器温度1：%f	激光器温度2：%f  中心幅值：%d	帧率值：%d\n", 
		//		"芯片温度：%f	激光器温度1：%f	激光器温度2：%f	帧率值：%d\n",
		//		frame.temperature.chip, frame.temperature.laser1, frame.temperature.laser2,
		//		//((unsigned short*)frame.pFrameData)[640 * 480 / 2 + 320], 
		//		fps);
		//	file
		//		<< frame.temperature.chip << ", "
		//		<< frame.temperature.laser1 << ", "
		//		<< frame.temperature.laser2 << ", "
		//		//<< ((unsigned short*)frame.pFrameData)[640 * 480 / 2 + 320] << "," 
		//		<< fps << "\n";
		//}
		
		//if (std::max(frame.temperature.laser1, frame.temperature.laser2) > Tval)
		//{
		//	// 将时间戳转换为本地时间的 tm 结构体
		//	local_time = std::localtime(&timestamp);
		//	oss.clear();
		//	oss << std::put_time(local_time, "	出现异常了!!!	\n	触发时间：%Y-%m-%d %H:%M:%S");
		//	MessageBox(NULL, oss.str().c_str(), "光衰检测", MB_OK | MB_ICONEXCLAMATION);
		//	// 设置帧率
		//	printf("\n请设置帧率：");
		//	scanf("%d", &val);
		//	ret = LWSetFrameRate(handleList[index], val);
		//	if (ret != LW_RETURN_OK)
		//	{
		//		printf("LWSetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		//		return 0;
		//	}
		//	// 设置判定阈值
		//	printf("请设置判定阈值：");
		//	scanf("%d", &Tval);
		//	printf("\n");
		//}
		//Sleep(60000);

		t0 = frame.timestamp.tv_sec;
		conut = 0;
	}
	//file.close();
	printf("数据读取完毕！！！\n\n\n");

	// 停止数据流并关闭设备
	ret = LWStopStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStopStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	ret = LWCloseDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWCloseDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		system("pause");
		return 0;
	}

	// 释放资源
	ret = LWCleanupResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWCleanupResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	}

	printf("退出！！！\n\n\n");

	system("pause");
	return 0;
}

void networkAbnormalCallback(LWDeviceHandle handle, const char* error, void* pUserData)
{
	printf("网络出错了：%llu, %s\n", handle, error);
}
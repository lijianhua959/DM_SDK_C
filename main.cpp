
#include <iostream>
#include <fstream>
#include "LWDMApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


// 不要做耗时处理， 尽量轻便
void frameReadyCallback(LWDeviceHandle handle, void* pUserData)
{
	auto ret = LWGetFrameReady(handle);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));

		// 根据实际情况做相应处理

		return;
	}
}
// 不要做耗时处理， 尽量轻便
void networkMonitoringCallback(LWDeviceHandle handle, const char* error, void* pUserData)
{
	// When the device does not have an RGB module, rgb is a null pointer.
	if (pUserData != nullptr)
	{
		// 
	}

	/// Do some custom operations after disconnection.
	printf("\n\nDevice Handle: %llu\nError: %s\n\n", handle, error);
	/// 
}

// 鼠标回调函数
void onMouse(int event, int x, int y, int flags, void* userdata)
{
	// 将传入的void指针转换回Mat类型
	cv::Mat* image = static_cast<cv::Mat*>(userdata);

	// 判断是否为鼠标左键双击事件
	if (event == cv::EVENT_LBUTTONDBLCLK)
	{
		// 确保坐标点在图像范围内
		if (x >= 0 && x < image->cols && y >= 0 && y < image->rows)
		{
			std::cout << "双击坐标: (" << x << ", " << y << ")" << std::endl;

			// 根据图像通道数判断是灰度图还是彩色图
			if (image->channels() == 1)
			{
				// 灰度图：像素值类型为 uchar
				ushort pixelValue = image->at<ushort>(y, x);
				std::cout << "像素值 (灰度): " << static_cast<int>(pixelValue) << std::endl;
			}
			else if (image->channels() == 3)
			{
				// 彩色图：像素值类型为 Vec3b (BGR顺序)
				cv::Vec3b pixelBGR = image->at<cv::Vec3b>(y, x);
				std::cout << "像素值 (BGR): "
					<< "B=" << static_cast<int>(pixelBGR[0]) << ", "
					<< "G=" << static_cast<int>(pixelBGR[1]) << ", "
					<< "R=" << static_cast<int>(pixelBGR[2]) << std::endl;
			}
			std::cout << "------------------------" << std::endl;
		}
	}
}


int main1()
{
	int len = 240 * 320 * 2;
	auto data = new char[len];
	std::ifstream fileHandle;
	fileHandle.open("C:\\Users\\12267\\Desktop\\22-distance_save.data", std::ios::in | std::ios::binary);
	if (fileHandle.is_open())
	{
		fileHandle.read(data, len);

		//// 保存
		//std::ofstream fileHandle_;
		//fileHandle_.open("C:\\Users\\12267\\Desktop\\22-distance_save.csv", std::ios::out);
		//unsigned short* p = (unsigned short*)data;
		//for (int i = 0; i < 240; i++)
		//{
		//	for (int j = 0, k = 0; j < 320; j++)
		//	{
		//		fileHandle_ << *p++ << ',';
		//	}
		//	fileHandle_ << '\n';
		//}

		// Depth图显示
		cv::Mat dst;
		cv::Mat src(240, 320, CV_16UC1, data);
		//src.convertTo(dst, CV_8UC1, 255.0 / 6000);
		cv::namedWindow("dst", 0);
		// 设置鼠标回调函数，并将图像指针作为用户数据传入
		cv::setMouseCallback("dst", onMouse, &src);
		//cv::resizeWindow("dst", 640, 480);
		cv::imshow("dst", src);
		cv::waitKey(0);
	}


	delete []data;
	return 0;
}


int main(int argc, char* argv[])
{
	std::cout << "_MSC_VER: " << _MSC_VER << std::endl;

	auto ret = LWInitializeResources();
	if (ret != LW_RETURN_OK)
	{
		printf("LWInitializeResources function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	// 注册网络回调函数
	LWRegisterNetworkMonitoringCallback(networkMonitoringCallback, nullptr);

	////printf("固件更新中......\n");
	////ret = LWUpdateFirmware1("192.168.1.200", "/home/ljh/data/updata_enc_v1.0.31.sh");
	//ret = LWUpdateFirmware1("192.168.1.200", "C:/Users/12267/Desktop/test/dm_updata_enc_V1.4.3.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWUpdateFirmware function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("固件更新完成\n");
	//return 0;

	LWDeviceHandle handleList[5];
	LWDeviceInfo deviceInfoList[5];
	int32_t findCount = 0;
	ret = LWGetDeviceInfoList(deviceInfoList, 5, &findCount);
	if (ret != LW_RETURN_OK)
	{
		printf("LWFindDevices function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	if (findCount < 1)
	{
		printf("No device found.");
		return 0;
	}

	for (int i = 0; i < findCount; i++)
	{
		printf("设备编号: %d, \n设备 SN: %s, 设备 Type: %s, \n设备 IP: %s, 本机 IP: %s, \n设备句柄: %llu\n\n",
			i,
			deviceInfoList[i].sn,
			deviceInfoList[i].type,
			deviceInfoList[i].ip,
			deviceInfoList[i].local_ip,
			deviceInfoList[i].handle
		);

		handleList[i] = deviceInfoList[i].handle;
	}

	// 选择设备
	int index = 0;
	lablie:
	if (findCount > 1)
	{
		std::cout << "Please enter the index value: ";
		std::cin >> index;
	}

	////printf("固件更新中......\n");
	////ret = LWUpdateFirmware1(deviceInfoList[index].ip, "/home/ljh/data/updata_enc_v1.0.31.sh");
	//ret = LWUpdateFirmware1(deviceInfoList[index].ip, "C:/Users/12267/Desktop/DATA/update_packets/updata_enc_v1.0.28.sh");
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWUpdateFirmware function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("固件更新完成\n");
	//return 0;

	// 打开设备
	ret = LWOpenDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("\nLWOpenDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		if (findCount > 1) goto lablie;
		return 0;
	}

	// 显示设备版本信息
	LWVersionInfo dv;
	LWVersionInfo fv;
	ret = LWGetDeviceVersion(handleList[index], &fv, &dv);
	if (ret != LW_RETURN_OK)
	{
		printf("LWGetDeviceVersion function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}
	printf("\n\n固件版本: %d.%d.%d.%d\n\n", fv.major, fv.minor, fv.patch, fv.reserved);
	printf("驱动版本: %d.%d.%d.%d\n\n", dv.major, dv.minor, dv.patch, dv.reserved);

	// 发送畸变文件
	//ret = LWSendFile(handleList[0], "C:/Users/12267/Desktop/DATA/DrnuLut_high.txt", LW_DRNU_HIGH);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSendFile function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}
	//return 0;

	//// 网络配置
	//LWNetworkInfo ninfo;
	//ninfo.type = 0x01;
	//memcpy(ninfo.ip, "192.168.1.200", 13);
	//memcpy(ninfo.netmask, "255.255.255.0", 13);
	//ret = LWSetNetworkInfo(handleList[index], ninfo);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetNetworkInfo function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//_sleep(25000);
	//ret = LWOpenDevice(handleList[index]);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("\nLWOpenDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//LWNetworkInfo ninfo;
	//ret = LWGetNetworkInfo(handleList[index], &ninfo);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWGetNetworkInfo function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//printf("IP: %s, Type: %u, Mask: %s\n", ninfo.ip, ninfo.type, ninfo.netmask);

	ret = LWSetExposureMode(handleList[index], LW_TOF_SENSOR, LWExposureMode::LW_EXPOSURE_MANUAL);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetExposureMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetHDRMode(handleList[index], LWHDRMode::LW_DFN_NOT_HDR);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetHDRMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetFrameRate(handleList[index], 25);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFrameRate function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	int earr[3] = { 1000, 150, 50 };
	ret = LWSetExposureTime(handleList[index], LW_TOF_SENSOR, earr, 3);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetExposureTime function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	//// 设置分辨率
	//ret = LWSetResolution(handleList[index], LWSensorType::LW_RGB_SENSOR, 640,480);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetResolution function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}
	//ret = LWOpenDevice(handleList[index]);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("\nLWOpenDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//	return 0;
	//}

	//ret = LWSetTransformDepthToRgbEnable(handleList[index], true);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetTransformDepthToRgbEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}

	//ret = LWSetTransformRgbToDepthEnable(handleList[index], true);
	//if (ret != LW_RETURN_OK)
	//{
	//	printf("LWSetTransformRgbToDepthEnable function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
	//}

	ret = LWSetTriggerMode(handleList[index], LW_TRIGGER_ACTIVE);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTriggerMode function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetDataReceiveType(handleList[index], LWDataRecvType::LW_DEPTH_RGB_RTY);
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetDataReceiveType function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetSpatialFilterParams(handleList[index], LWFilterParam{ false, 3 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetSpatialFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetTimeFilterParams(handleList[index], LWFilterParam{ false, 3, 100 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetTimeMedianFilterParams(handleList[index], LWFilterParam{ false, 3, 100 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetTimeMedianFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetFlyingPixelsFilterParams(handleList[index], LWFilterParam{ false, 5 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetFlyingPixelsFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWSetConfidenceFilterParams(handleList[index], LWFilterParam{ false, 5 });
	if (ret != LW_RETURN_OK)
	{
		printf("LWSetConfidenceFilterParams function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWStartStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWStartStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWFrameData frame;
	int64_t t0 = 0;
	int64_t num = 0;
	int64_t count = 0;
	int64_t err_count = 0;
	while (true)
	{
		ret = LWGetFrameReady(handleList[index]);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrameReady function call failed: %s\n", LWGetReturnCodeDescriptor(ret));
			if (++err_count > 3) break;
			continue;
		}
		err_count = 0;

		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_RGB_TO_DEPTH_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_DEPTH_TO_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_IR_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_AMPLITUDE_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_RGB_FRAME);
		//ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_POINTCLOUD_FRAME);
		ret = LWGetFrame(handleList[index], &frame, LWFrameType::LW_DEPTH_FRAME);
		if (ret != LW_RETURN_OK)
		{
			printf("LWGetFrame function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
			continue;
		}

		// 帧率显示
		if (t0 != frame.timestamp.tv_sec)
		{
			printf("\n---------------FPS: %lld ", count / (frame.timestamp.tv_sec - t0));

			t0 = frame.timestamp.tv_sec;
			count = 0;
		}
		++count;

		//// 帧信息显示
		//++num;
		//printf("\nnum: %u, frame index: %u, timestamp: %lld.%lld, chip: %.2f℃, laser1: %.2f℃, laser2: %.2f℃, frameType: %u, width: %u, height: %u, bufferSize: %u, elemSize: %u, total: %u",
		//	num,
		//	frame.frameIndex,
		//	frame.timestamp.tv_sec, frame.timestamp.tv_usec,
		//	frame.temperature.chip, frame.temperature.laser1, frame.temperature.laser2,
		//	frame.frameType,
		//	frame.width, frame.height,
		//	frame.bufferSize,
		//	frame.elemSize,
		//	frame.total);

		//// IR图像显示
		//cv::Mat img(frame.height, frame.width, CV_8UC1, frame.pFrameData);
		//cv::namedWindow("img", 0);
		//cv::resizeWindow("img", 640, 480);
		//cv::imshow("img", img);
		//cv::waitKey(1);

		// Depth图显示
		cv::Mat dst;
		cv::Mat src(frame.height, frame.width, CV_16UC1, frame.pFrameData);
		src.convertTo(dst, CV_8UC1, 255.0 / 6000);
		cv::namedWindow("dst", 0);
		cv::resizeWindow("dst", 640, 480);
		cv::imshow("dst", dst);
		cv::waitKey(30);

		//// RGB图像显示
		//cv::Mat img(frame.height, frame.width, CV_8UC3, frame.pFrameData);
		//cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
		//cv::namedWindow("img", 0);
		//cv::resizeWindow("img", 640, 480);
		//cv::imshow("img", img);
		//cv::waitKey(30);

		//// 保存图像
		//cv::imwrite("C:/Users/12267/Desktop/kimg.bmp", img);
		//cv::imwrite("C:/Users/12267/Desktop/img1.png", img);
		//cv::imwrite("C:/Users/12267/Desktop/kimg.jpg", img);

		//// 保存数据
		//ret = LWSaveDataAsCSVFile("C:/Users/12267/Desktop/dep.csv", &frame);
		//if (ret != LW_RETURN_OK)
		//{
		//	printf("LWGetFrame function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		//	//break;
		//}

		if (frame.frameIndex > 400) break;
	}

	ret = LWStopStream(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("\nLWStopStream function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	ret = LWCloseDevice(handleList[index]);
	if (ret != LW_RETURN_OK)
	{
		printf("LWCloseDevice function call failed: %s\n\n", LWGetReturnCodeDescriptor(ret));
		return 0;
	}

	LWCleanupResources();

	printf("\n退出\n");

	return 0;
}

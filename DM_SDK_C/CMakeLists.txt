# CMakeList.txt: DM_SDK_C 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#

project("dm_c_sdk")


if(WIN32)# Windows 平台

	if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64位系统
		if(CMAKE_BUILD_TYPE STREQUAL "Debug")
			set(OpenCV_DIR D:/ThirdPartyLib/opencv/x64d/x64/vc17/staticlib)
			set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x64d)
		else()
			set(OpenCV_DIR D:/ThirdPartyLib/opencv/x64/x64/vc17/staticlib)
			set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x64)
		endif()
		set(JpegTurbo_DIR D:/ThirdPartyLib/libjpeg_turbo/x64)
		set(OpenSSL_ROOT_DIR D:/ThirdPartyLib/openssl-3.4.0/x64)
	else() # 32位系统
		set(OpenCV_DIR D:/ThirdPartyLib/opencv/x86/x86/vc17/staticlib)
		set(JpegTurbo_DIR D:/ThirdPartyLib/libjpeg_turbo/x86)
		set(OpenSSL_ROOT_DIR D:/ThirdPartyLib/openssl-3.4.0/x86)
		set(Update_ROOT_DIR D:/ThirdPartyLib/Firmware_Update/x86)	
	endif()

	find_package(OpenCV REQUIRED)

	# 加载第三方库头文件
	#include_directories(${JpegTurbo_DIR}/include)
	include_directories(${OpenSSL_ROOT_DIR}/include)
	include_directories(${Update_ROOT_DIR}/include)
	include_directories("D:/ThirdPartyLib/eigen-3.4.0")

	# 将源代码添加到此项目的动态库。
	add_library (dm_c_sdk SHARED "LWDMApi.cpp" "LWDMApi.h" "LWDMType.h" "resource.h" "Version.rc" )

	# 链接库
	target_link_libraries(dm_c_sdk 
	${OpenCV_LIBS} 
	#${JpegTurbo_DIR}/lib/turbojpeg-static.lib 
	${OpenSSL_ROOT_DIR}/lib/libcrypto.lib 
	${OpenSSL_ROOT_DIR}/lib/libssl.lib 
	${Update_ROOT_DIR}/lib/libssh2.lib 
	${Update_ROOT_DIR}/lib/libMasterUpdate.lib 
	crypt32.lib
	)

	# 将生成的动态库复制到可执行程序的目录下
	add_custom_command(TARGET dm_c_sdk POST_BUILD COMMAND "${CMAKE_COMMAND}" -E copy dm_c_sdk.dll "../")

elseif(UNIX) # Linux 平台
	
	set(OpenCV_DIR /home/ljh/third_party_libs/opencv/linux/lib/cmake/opencv4)
	set(OPENSSL_ROOT_DIR /home/ljh/third_party_libs/openssl/linux)
	set(Update_ROOT_DIR /home/ljh/third_party_libs/firmware_update/linux)

	# set(OpenCV_DIR /home/ljh/third_party_libs/opencv/arm7.5/lib/cmake/opencv4)
	# set(OPENSSL_ROOT_DIR /home/ljh/third_party_libs/openssl/arm7.5)

	# set(OpenCV_DIR /home/ljh/third_party_libs/opencv/arm11.2/lib/cmake/opencv4)
	# set(OPENSSL_ROOT_DIR /home/ljh/third_party_libs/openssl/arm11.2)

	find_package(Eigen3 REQUIRED)
	find_package(OpenCV REQUIRED)
	find_package(OpenSSL REQUIRED)

	include_directories(/usr/include/eigen3)
	include_directories(${OPENSSL_INCLUDE_DIR})
	include_directories(${Update_ROOT_DIR}/include)

	# 将源代码添加到此项目的动态库。
	add_library (dm_c_sdk SHARED "LWDMApi.cpp" "LWDMApi.h" "LWDMType.h")

	# 链接库
	target_link_libraries(dm_c_sdk 
	${OpenCV_LIBS} 
	${OPENSSL_CRYPTO_LIBRARIES}
	${Update_ROOT_DIR}/lib/libssh2.a
	${Update_ROOT_DIR}/lib/libfirmware_update.a
	)

	# 设置版本号
	set_target_properties(dm_c_sdk PROPERTIES VERSION ${MAJOR_VAR}.${MINOR_VAR}.${PATCH_VAR}.${REVISION_VAR})

else() # 其他平台

	message(STATUS "Current platform: Unknown")

endif()

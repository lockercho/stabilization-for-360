// stdafx.h : 可在此標頭檔中包含標準的系統 Include 檔，
// 或是經常使用卻很少變更的
// 專案專用 Include 檔案
//

#pragma once

// define _NO_CRT_STDIO_INLINE 1 and link legacy_stdio_definitions.lib to workaround clang/c2 build fail
#define _NO_CRT_STDIO_INLINE 1

#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES_COUNT 1

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // 從 Windows 標頭排除不常使用的成員
// Windows 標頭檔:
#include <windows.h>

// C RunTime 標頭檔
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

#include <iostream>
#include <thread>

// TODO:  在此參考您的程式所需要的其他標頭

#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#define _USE_MATH_DEFINES // for C
#include <math.h>

#define EIGEN_DONT_VECTORIZE // clang/c2 didn't support intrinsic, Intrinsic not yet implemented!
#include <Eigen/Eigen>

#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>
#include <opengv/math/quaternion.hpp>

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>

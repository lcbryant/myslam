/*************************************************************************
	> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年07月18日 星期六 15时14分22秒
    > 说明：rgbd-slam教程所用到的基本函数（C风格）
 ************************************************************************/
#pragma once

// 各种头文件
// C++标准库
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    cv::Mat rgb, depth;      //该帧对应的彩色图与深度图
    cv::Mat desp;            //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

//双目相机参数
struct STEREO_CAMERA_PARAMETERS
{
    double fx, fy, cx, cy, baseline, inx, iny, outx, outy;
};

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera);

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera);

// 参数读取类
class ParameterReader
{
public:
    ParameterReader(string filename = "./parameters.txt")
    {
        cout << filename << endl;
        ifstream fin(filename.c_str());
        if (!fin)
        {
            cerr << "parameter file does not exist." << endl;
            return;
        }
        while (!fin.eof())
        {
            string str;
            getline(fin, str);
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr(0, pos);
            string value = str.substr(pos + 1, str.length());
            data[key] = value;

            if (!fin.good())
                break;
        }
    }
    string getData(string key)
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

public:
    map<string, string> data;
};

class XCTool
{
public:
};

class XCKey
{
public:
    string frameID;
    double tx, ty, tz;
    double qx, qy, qz, qw;
};

class XCAssociationKey
{
public:
    string rgb, full_rgb, d, full_d;
};

class XCKITTIKey
{
public:
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    double tx, ty, tz;
};

string FindDFileByRGB(vector<XCAssociationKey> &aKeyVec, string rgb);

//点云初始化
PointCloud::Ptr image2PointCloudInverse(cv::Mat &rgb, string depthPath, CAMERA_INTRINSIC_PARAMETERS &camera);

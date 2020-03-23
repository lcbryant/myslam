#include <iostream>
#include <thread>
#include "slamBase.h"
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

void JoinStereo(string camTransFile, string dataMainPath);

int main(int argc, char** argv)
{
    std::cout<<"请输入位资文件和数据文件:"<<std::endl;
    string camTransFile,dataMainPath;
    cin>>camTransFile;
    cin>>dataMainPath;
    JoinStereo(camTransFile,dataMainPath);
    return 0;
}

void JoinStereo(string camTransFile, string dataMainPath) {
	ParameterReader pd("../parameters.txt");
	// 相机内参
	STEREO_CAMERA_PARAMETERS camera;
	camera.fx = atof(pd.getData("camera.fx").c_str());
	camera.fy = atof(pd.getData("camera.fy").c_str());
	camera.cx = atof(pd.getData("camera.cx").c_str());
	camera.cy = atof(pd.getData("camera.cy").c_str());
	camera.baseline = atof(pd.getData("camera.baseline").c_str());
	camera.inx = atof(pd.getData("camera.inx").c_str());
	camera.iny = atof(pd.getData("camera.iny").c_str());
	camera.outx = atof(pd.getData("camera.outx").c_str());
	camera.outy = atof(pd.getData("camera.outy").c_str());
	cout<<"相机参数已经读取完毕"<<endl;
	
	int n=0;
	string tmp;
	ifstream fin((dataMainPath+"/times.txt").c_str());
	while(getline(fin,tmp)){
		n++;
	};
	fin.close();
	cout<<"共有"<<n<<"份文件带读取"<<endl;

    ifstream fcamTrans(camTransFile.c_str());
	vector<XCKITTIKey> keyVec;
	for(int i=0;i<n;i++)
	{
		XCKITTIKey tkey;
		fcamTrans >> tkey.r00;
		fcamTrans >> tkey.r01;
		fcamTrans >> tkey.r02;
		fcamTrans >> tkey.tx;
		fcamTrans >> tkey.r10;
		fcamTrans >> tkey.r11;
		fcamTrans >> tkey.r12;
		fcamTrans >> tkey.ty;
		fcamTrans >> tkey.r20;
		fcamTrans >> tkey.r21;
		fcamTrans >> tkey.r22;
		fcamTrans >> tkey.tz;
		keyVec.push_back(tkey);
	}
 
 
	//初始化点云
	vector<string> leftPathVec, rightPathVec,depthLeftPathVec,depthRightPathVec;
	for (int i = 0; i < keyVec.size(); i++) {
		char ts[7];
		sprintf(ts, "%06d", i);
		leftPathVec.push_back(dataMainPath + "/image_0/" + ts+".png");
		rightPathVec.push_back(dataMainPath + "/image_1/" + ts + ".png");
		depthLeftPathVec.push_back(dataMainPath + "/L/" + ts + "_disp.pgm");
		depthRightPathVec.push_back(dataMainPath + "/R/" + ts + "_disp.pgm");
    }
	cout<<"已经完成图像路径的读取"<<endl;
    vector<PointCloud::Ptr> pcVec;
	CAMERA_INTRINSIC_PARAMETERS tcam;
	tcam.fx = camera.fx;
	tcam.fy = camera.fy;
	tcam.cx = camera.cx;
	tcam.cy = camera.cy;
	tcam.scale = 1000;
	for (int i = 0; i < keyVec.size(); i++) {
		FRAME tframe;
		tframe.rgb = cv::imread(leftPathVec[i]);
		tframe.depth = cv::imread(depthLeftPathVec[i], -1);
		pcVec.push_back(image2PointCloudInverse(tframe.rgb, depthLeftPathVec[i], tcam));
		cout<<"已完成第"<<i+1<<"份读取"<<endl;
	}

    //减裁
   for (auto& pciter : pcVec) {
		PointCloud::Ptr cloud_filtered3(new PointCloud());
		//z剪裁
		pcl::PassThrough<pcl::PointXYZRGBA> zpass;
		zpass.setInputCloud(pciter);
		zpass.setFilterFieldName("z");
		zpass.setFilterLimits(0, 2.5);
		zpass.filter(*cloud_filtered3);
		pciter = cloud_filtered3;
 
		PointCloud::Ptr cloud_filtered4(new PointCloud());
		//x剪裁
		pcl::PassThrough<pcl::PointXYZRGBA> xpass;
		xpass.setInputCloud(cloud_filtered3);
		xpass.setFilterFieldName("x");
		xpass.setFilterLimits(-1000.0, 1000.0);
		xpass.filter(*cloud_filtered4);
 
		//y剪裁
		PointCloud::Ptr cloud_filtered5(new PointCloud());
		pcl::PassThrough<pcl::PointXYZRGBA> ypass;
		ypass.setInputCloud(cloud_filtered4);
		ypass.setFilterFieldName("y");
		ypass.setFilterLimits(-0.6, 10.0);
		ypass.filter(*cloud_filtered5);
 
		pciter = cloud_filtered5;

   }
	//滤波
	bool bGridFilter = false;
	for (auto& pciter : pcVec) {
		if (bGridFilter) {
			PointCloud::Ptr cloud_filtered(new PointCloud());
			pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
			sor.setInputCloud(pciter);
			sor.setLeafSize(0.01f, 0.01f, 0.01f);
			sor.filter(*cloud_filtered);
			pciter = cloud_filtered;
		}
 
		PointCloud::Ptr cloud_filtered2(new PointCloud());
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor2;
		sor2.setInputCloud(pciter);
		sor2.setMeanK(50);
		sor2.setStddevMulThresh(1.0);
		sor2.filter(*cloud_filtered2);
		pciter = cloud_filtered2;
    }

    //旋转
    for (int i = 0; i<keyVec.size(); i++) {
 
		cv::Mat R;
 
		R = (cv::Mat_<double>(3, 3) <<
			keyVec[i].r00, keyVec[i].r01, keyVec[i].r02,
			keyVec[i].r10, keyVec[i].r11, keyVec[i].r12,
			keyVec[i].r20, keyVec[i].r21, keyVec[i].r22
			);
	
		R = R.inv();
		Eigen::Matrix3d r;
		cv::cv2eigen(R, r);
 
		// 将平移向量和旋转矩阵转换成变换矩阵
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
 
		Eigen::AngleAxisd angle(r);
		cout << "translation" << endl;
 
		double tscale = 5;
 
		T = angle;
		T(0, 3) = keyVec[i].tx / tscale;
		T(1, 3) = keyVec[i].ty / tscale;
		T(2, 3) = keyVec[i].tz / tscale;
 
		PointCloud::Ptr toutput(new PointCloud());
		pcl::transformPointCloud(*pcVec[i], *toutput, T.matrix());
		pcVec[i] = toutput;
	}
 
	PointCloud::Ptr allOutput(new PointCloud()),allAfterFilter(new PointCloud());
 
	bool bExtractPlane = false;
	for (auto& pciter : pcVec) {
		//???
		if (bExtractPlane) {
			PointCloud::Ptr cloud_filtered3(new PointCloud());
			PointCloud::Ptr cloud_f(new PointCloud());
 
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
			// Optional
			seg.setOptimizeCoefficients(true);
			// Mandatory
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(1000);
			seg.setDistanceThreshold(0.01);
 
			// Create the filtering object
			pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
 
			int i = 0, nr_points = (int)pciter->points.size();
			// While 30% of the original cloud is still there
			while (pciter->points.size() > 0.3 * nr_points)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud(pciter);
				seg.segment(*inliers, *coefficients);
				if (inliers->indices.size() == 0)
				{
					std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
					break;
				}
 
				// Extract the inliers
				extract.setInputCloud(pciter);
				extract.setIndices(inliers);
				extract.setNegative(false);
				extract.filter(*cloud_filtered3);
				std::cerr << "PointCloud representing the planar component: " << cloud_filtered3->width * cloud_filtered3->height << " data points." << std::endl;
 
 
				// Create the filtering object
				extract.setNegative(true);
				extract.filter(*cloud_f);
				pciter.swap(cloud_f);
				i++;
				cout << "%extract\n";
				break;
			}
 
			pciter = cloud_filtered3;
		}
	}
	cout << "Pairing...\n";
	
	vector<int> Ids;
	//用来储存关键帧的ID
	ifstream f((dataMainPath+"/keyframeId.txt").c_str());
	while(getline(f,tmp)){
		Ids.push_back(atoi(tmp.c_str()));
	}
	f.close();
	//储存了关键帧的ID
	for (int i = 0; i < Ids.size(); i++) {
	 		*allOutput += *pcVec[Ids[i]];
	 	}
	
	//进行一次降采样
	cout<<"当前文件共有"<<allOutput->points.size()<<"个点，是否需要进行降采样处理"<<endl;
	cout<<"输入1为需要"<<endl;
	cin>>tmp;
	if(tmp=="1")
	{
		double scale0;
		cout<<"请输入你想定义的叶子节点大小单位为cm"<<endl;
		cin>>scale0;
	
		PointCloud::Ptr cloud_filtered(new PointCloud());
		pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
		sor.setInputCloud(allOutput);
		sor.setLeafSize(0.01f*scale0, 0.01f*scale0, 0.01f*scale0);
		sor.filter(*cloud_filtered);
		allOutput = cloud_filtered;
	}

	
	pcl::io::savePCDFile(dataMainPath+".pcd", *allOutput);
	cout<<"当前文件共有"<<allOutput->points.size()<<"个点"<<endl;
	cout << "Final result saved." << endl;
 
	// pcl::visualization::CloudViewer viewer("viewer");
	// struct callback_args cb_args;
	// cb_args.clicked_points_3d = allOutput;
	// cb_args.viewerPtr = &viewer;
	// viewer.registerPointPickingCallback(pp_callback, (void*)&cb_args);
	// viewer.showCloud(allOutput);
	// while (!viewer.wasStopped())
	// {
 
	// }

}

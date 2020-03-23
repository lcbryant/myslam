// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <time.h>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp =
     std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());//获取当前时间点
    std::time_t timestamp =  tp.time_since_epoch().count(); //计算距离1970-1-1,00:00的时间长度
    return timestamp;
}

int main(int argc,char *argv[])
{
  std::string dir;
  std::cout<<"请输入你需要创建的文件夹名称:\n";
  std::cin>>dir;
  std::cin.sync();

  if(access(dir.c_str(),0)==-1)
  {
    std::cout<<dir<<" is not existing"<<std::endl;
    mkdir(dir.c_str(),0777);
    mkdir((dir+"/"+"image_0").c_str(),0777);
    mkdir((dir+"/"+"image_1").c_str(),0777);

  }
  //创建数据集文件夹

  std::string leftimage_file=dir+"/image_0/";
  std::string rightimage_file=dir+"/image_1/";
  std::string times_file=dir+"/times.txt";
  std::ofstream time_write(times_file.c_str());
  std::time_t t0;

  auto &&api=API::Create(argc,argv);
  if (!api) return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->EnableStreamData(Stream::DEPTH);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");

  std::int32_t count = 0;
  std::cout << "Press 'Q' to finish colleting." << std::endl;

  while (true) {
    api->WaitForStreams();

    static api::StreamData left_data;
    static api::StreamData right_data;
    //static api::StreamData depth_data;

    auto right_data_tmp = api->GetStreamData(Stream::RIGHT_RECTIFIED);
    if (!right_data_tmp.frame.empty()) {
      right_data = right_data_tmp;
    }
    auto left_data_tmp = api->GetStreamData(Stream::LEFT_RECTIFIED);
    if (!left_data_tmp.frame.empty()) {
      left_data = left_data_tmp;
    }

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      cv::imshow("frame", img);
    }

    std::time_t t = getTimeStamp();
    if(count==0)
    {
      t0=t;
    }
    t=t-t0;
    // auto depth_data_tmp = api->GetStreamData(Stream::DEPTH);
    // if (!depth_data_tmp.frame.empty()) {
    //   depth_data = depth_data_tmp;
    // }
    // if (!depth_data.frame.empty()) {
    //   cv::imshow("depth_real", depth_data.frame);  // CV_16UC1
    // }


    char key = static_cast<char>(cv::waitKey(200));
    if (key == 'Q' || key == 'q' ){
      break;
    }
    else{
      if (!left_data.frame.empty()
          && !right_data.frame.empty()) {
        std::string l_name;
        std::string r_name;

        char ts[7];
        sprintf(ts,"%06d",count);
        l_name= leftimage_file+ts+".png";
        r_name=rightimage_file+ts+".png";

        if(count!=0){
          time_write<<"\n";
        }
        time_write<<t/100.0;
        cv::imwrite(l_name.c_str(), left_data.frame);
        cv::imwrite(r_name.c_str(), right_data.frame);

        std::cout << "Saved image "<<count<<" success to current directory" << std::endl;
        ++count;
      }
    }

    // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
    //   break;
    // } else if (key == 32 || key == 's' || key == 'S') {
    //   if (!left_data.frame.empty()
    //       && !right_data.frame.empty()) {
    //     std::string l_name;
    //     std::string r_name;

    //     char ts[7];
    //     sprintf(ts,"%06d",count);
    //     l_name= leftimage_file+ts+".png";
    //     r_name=rightimage_file+ts+".png";

    //     if(count!=0){
    //       time_write<<"\n";
    //     }
    //     time_write<<t;
    //     cv::imwrite(l_name.c_str(), left_data.frame);
    //     cv::imwrite(r_name.c_str(), right_data.frame);

    //     std::cout << "Saved image "<<count<<" success to current directory" << std::endl;
    //     ++count;
    //   }
    // }
  }
  time_write.close();

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
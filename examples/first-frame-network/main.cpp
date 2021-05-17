/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include "../../sdk/src/cameras/itof-camera/tofi/floatTolin.h"

/*#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif
*/
using namespace aditof;

int main(int argc, char *argv[]) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    if (argc < 2) {
        LOG(ERROR) << "No ip provided! ./first-frame-network ip!";
        return 0;
    }

    std::string ip = argv[1];

    System system;

    std::vector<std::shared_ptr<Camera>> cameras;
    system.getCameraListAtIp(cameras, ip);
    if (cameras.empty()) {
        LOG(WARNING) << "No cameras found";
        return 0;
    }

    auto camera = cameras.front();
    status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }

    std::vector<std::string> frameTypes;
    camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        std::cout << "no frame type available!";
        return 0;
    }
    status = camera->setFrameType("pcm"); //hardcoded for now as it is the simples kind of frame
    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    // std::vector<std::string> modes;
    // camera->getAvailableModes(modes);
    // if (modes.empty()) {
    //     LOG(ERROR) << "no camera modes available!";
    //     return 0;
    // }
    // status = camera->setMode(modes.front());
    // if (status != Status::OK) {
    //     LOG(ERROR) << "Could not set camera mode!";
    //     return 0;
    // }
	
	int i=0;
//   auto start = std::chrono::steady_clock::now();
//	cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
/*while(cv::waitKey(1) != 27 &&
           getWindowProperty("Display Image", cv::WND_PROP_AUTOSIZE) >=
               0 ) {
           
    cv::Mat mat;
  */  
    aditof::Frame frame;
    status = camera->requestFrame(&frame);
    if (status != Status::OK) {
        LOG(ERROR) << "Could not request frame!";
        return 0;
    } else {
        LOG(INFO) << "succesfully requested frame!";
    }

    uint16_t *data1;
    uint16_t data[1024*1024];

    status = frame.getData("ir", &data1);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not get frame data!";
        return 0;
    }

    if (!data1) {
        LOG(ERROR) << "no memory allocated in frame";
        return 0;
    }
	for (unsigned int i = 0; i < 1024 * 1024; ++i) {
	data[i]=data1[i] >> 4;
	if(data[i]<2048){
         data[i] = Convert11bitFloat2LinearVal(data1[i]);
     	} else {
     		data[i]=0;
     	}
     }
     
//    mat = cv::Mat(1024, 1024, CV_16UC1, data);
//     int max_value_of_IR_pixel = (1 << 12) - 1;

    /* Distance factor IR */
//    double distance_scale_ir = 255.0 / max_value_of_IR_pixel;

//    mat.convertTo(mat, CV_8U, distance_scale_ir);
  //  cv::cvtColor(mat, mat, cv::COLOR_GRAY2RGB);
    //imshow("Display Image", mat);
    //int k=cv::waitKey(10) & 0XFF;
    //cv::waitKey(1);
    //mat.release();
    //cv::destroyAllWindows();
for( int i = 0; i < 1024 * 1024;i++)
std::cout << data[i];
    //}
    //auto finish = std::chrono::steady_clock::now();
   // double elapsed_seconds = std::chrono::duration_cast<
 // std::chrono::duration<double> >(finish - start).count();
 // LOG(INFO) << "TIME: " << elapsed_seconds;
    return 0;
}

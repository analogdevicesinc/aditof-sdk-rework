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
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef OPENCV2
#include <opencv2/contrib/contrib.hpp>
#endif

#include "../aditof_opencv.h"

using namespace aditof;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    Status status = Status::OK;

    System system1;
    System system2;


    cv::Mat logo = cv::imread("config/logo.png"); 
    if (logo.empty()) {
        LOG(ERROR) << "Could not open or find the logo";
        return 0;
    }

    //Change the logo to a format that opencv can use
    //logo.convertTo(logo, CV_8U,255.0);

    std::vector<std::shared_ptr<Camera>> cameras1;
    std::vector<std::shared_ptr<Camera>> cameras2;
    system1.getCameraList(cameras1);//,"10.42.0.17");
    usleep(1000000);
    system2.getCameraList(cameras2);//,"10.42.1.31");
    usleep(1000000);
    if (cameras1.empty()) {
        LOG(WARNING) << "No cameras found!";
        return 0;
    }

    auto camera1 = cameras1.front();
    auto camera2 = cameras2.front();
    //status = camera->initialize();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return 0;
    }
    status = camera1->setControl("initialization_config","config/config_walden_nxp.json");
    //status = camera1->setControl("loadModuleData", "call");

    status = camera2->setControl("initialization_config","config/config_walden_nxp.json");
    //status = camera2->setControl("loadModuleData", "call");
    if (status != Status::OK) {
        LOG(INFO) << "No CCB/CFG data found in camera module,";
        return 0;
    }
status = camera1->initialize();
status = camera2->initialize();

    std::vector<std::string> frameTypes;
    camera1->getAvailableFrameTypes(frameTypes);
    camera2->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "No frame type available!";
        return 0;
    }

    status = camera1->setFrameType("pcm");    usleep(1000000);

 //   status = camera2->setFrameType("pcm");    usleep(1000000);

    if (status != Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return 0;
    }

    status = camera1->start();
    status = camera2->start();
    if (status != Status::OK) {
        LOG(ERROR) << "Could not start the camera!";
        return 0;
    }

    aditof::CameraDetails cameraDetails;
    camera1->getDetails(cameraDetails);
        camera2->getDetails(cameraDetails);

    aditof::Frame frame1;
    aditof::Frame frame2;

    cv::namedWindow("Display Image1", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Display Image2", cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) != 27 &&
           getWindowProperty("Display Image", cv::WND_PROP_AUTOSIZE) >= 0) {

        /* Request frame from camera */
        status = camera1->requestFrame(&frame1);
        status = camera2->requestFrame(&frame2);
        if (status != Status::OK) {
            LOG(ERROR) << "Could not request frame!";
            return 0;
        }

        /* Convert from frame to depth mat */
        cv::Mat mat1;
        cv::Mat mat2;
        status = fromFrameToIrMat(frame1, mat1);
        status = fromFrameToIrMat(frame2, mat2);

        if (status != Status::OK) {
            LOG(ERROR) << "Could not convert from frame to mat!";
            return 0;
        }

        //Read the center point distance value
        cv::Point2d pointxy(512, 512);
        //int m_distanceVal = static_cast<int>(mat.at<ushort>(pointxy));

        /* Convert from raw values to values that opencv can understand */
        mat1.convertTo(mat2, CV_8U,0.2,5);
        mat2.convertTo(mat2, CV_8U,0.2,5);

        /* Apply a rainbow color map to the mat to better visualize the
         * depth data */
        applyColorMap(mat1, mat1, cv::COLORMAP_WINTER);
        applyColorMap(mat2, mat2, cv::COLORMAP_WINTER);

        //Draw the center point
        char text[20];
        //sprintf(text, "%dmm", m_distanceVal);
       // cv::drawMarker(mat, pointxy, cv::Scalar(255, 255, 255),
       //                cv::MARKER_CROSS);
       // cv::circle(mat, pointxy, 8, cv::Scalar(255, 255, 255));
       // cv::putText(mat, text, pointxy + cv::Point2d(10, 20),
       //             cv::FONT_HERSHEY_DUPLEX, 3,
       //             cv::Scalar(255, 255, 255),4);
       // 
       // cv::Mat insertLogo(mat, cv::Rect(50, 900,200,79));
       // cv::addWeighted(insertLogo, 0.85 , logo,
       //                 1.0F - 0.85, 0, insertLogo);

        /* Display the image */
        //imshow("Display Image1", mat1);
        //imshow("Display Image2", mat2);
        LOG(INFO) << "SUCCESFULLY REQU FRAME <<<<<<";

    }

    return 0;
}

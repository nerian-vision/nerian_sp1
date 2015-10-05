/*******************************************************************************
 * Copyright (c) 2015 Nerian Vision Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *******************************************************************************/

#include <ros/ros.h>
#include <visiontransfer/asynctransfer.h>
#include <visiontransfer/reconstruct3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <iomanip>
#include <nerian_sp1/StereoCameraInfo.h>
#include <boost/smart_ptr.hpp>
#include <colorcoder.h>
#include <sdlwindow.h>

using namespace std;

/**
 * \brief A simple node that receives data from the SP1 stereo vision system
 * and forwards it to ROS.
 *
 * The SP1 stereo vision system is a stand-alone device for real-time stereo
 * matching. It transmits its processing results through gigagibt ethernet
 * which are then received by this node. The node converts the received data
 * into ROS messages, which contain the following data:
 *
 * - Point cloud of reconstructed 3D locations
 * - Disparity map with optional color coding
 * - Rectified left camera image
 *
 * In addition, camera calibration information are also published. For
 * configuration parameters, please see the provided example launch file.
 * For more information about the SP1 stereo vision system, please visit
 * http://nerian.com/products/sp1-stereo-vision/
 */

class Sp1Node {
public:
    Sp1Node() {
    }

    ~Sp1Node() {
    }

    /**
     * \brief Performs general initializations
     */
    void init() {
        ros::NodeHandle privateNh("~");

        // Read all ROS parameters
        if (!privateNh.getParam("point_cloud_intensity_channel", intensityChannel)) {
            intensityChannel = true;
        }

        if (!privateNh.getParam("color_code_disparity_map", colorCodeDispMap)) {
            colorCodeDispMap = true;
        }

        if (!privateNh.getParam("color_code_legend", colorCodeLegend)) {
            colorCodeLegend = false;
        }

        if (!privateNh.getParam("frame", frame)) {
            frame = "world";
        }

        if (!privateNh.getParam("port", port)) {
            port = "7681";
        }

        if (!privateNh.getParam("use_tcp", useTcp)) {
            useTcp = false;
        }

        if (!privateNh.getParam("host", host)) {
            host = "";
        }

        if (!privateNh.getParam("ros_coordinate_system", rosCoordinateSystem)) {
            rosCoordinateSystem = true;
        }

        if (!privateNh.getParam("calibration_file", calibFile)) {
            calibFile = "";
        }

        if (!privateNh.getParam("disparity_window", dispWindow)) {
            dispWindow = false;
        }

        // Create debugging window
        if(dispWindow) {
            window.reset(new SDLWindow(640, 480, "Disparity Map"));
        }

        // Create publishers
        disparityPublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_sp1/disparity_map", 5)));
        imagePublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_sp1/left_image", 5)));

        if(calibFile == "" ) {
            ROS_WARN("No camera calibration file configured. Point cloud and camera information publishing is disabled!");
        } else {
             if (!calibStorage.open(calibFile, cv::FileStorage::READ)) {
                throw std::runtime_error("Error reading calibration file: " + calibFile);
            }

            cameraInfoPublisher.reset(new ros::Publisher(nh.advertise<nerian_sp1::StereoCameraInfo>(
                "/nerian_sp1/stereo_camera_info", 1)));
            cloudPublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(
                "/nerian_sp1/point_cloud", 5)));

            // For point cloud publishing we have to do more initializations
            initPointCloud(privateNh);
        }
    }

    /**
     * \brief The main loop of this node
     */
    int run() {
        ros::Time stamp = ros::Time::now();
        ros::Time lastLogTime;
        int lastLogFrames = 0;

        int leftWidth, leftHeight, leftStride;
        unsigned char* leftData = NULL;

        int dispWidth, dispHeight, dispStride;
        unsigned char* dispData = NULL;


        AsyncTransfer asyncTransfer(useTcp ? ImageTransfer::TCP_CLIENT : ImageTransfer::UDP_CLIENT,
            host.c_str(), port.c_str());

        while(ros::ok()) {
            int imageNum, width, height, stride;
            ImageProtocol::ImageFormat format;
            unsigned char* data = NULL;

            // Receive image data
            data = asyncTransfer.collectReceivedImage(imageNum, width, height, stride, format, 0.5);

            // Find out which image this was
            if(data != NULL && imageNum == 0) {
                stamp = ros::Time::now();
                leftWidth = width;
                leftHeight = height;
                leftStride = stride;
                leftData = data;
            } else if(data != NULL && imageNum == 1) {
                dispWidth = width;
                dispHeight = height;
                dispStride = stride;
                dispData = data;
            }

            if(leftData != NULL && dispData != NULL) {
                // We have received the disparity map and the left camera image

                if(leftWidth != dispWidth || leftHeight != dispHeight) {
                    throw std::runtime_error("Invalid image sizes!");
                }

                // Publish the selected messages
                if(imagePublisher->getNumSubscribers() > 0) {
                    publishImageMsg(leftWidth, leftHeight, leftStride, leftData, stamp);
                }

                if(disparityPublisher->getNumSubscribers() > 0 || window != NULL) {
                    publishDispMapMsg(dispWidth, dispHeight, dispStride, dispData, stamp);
                }

                if(cloudPublisher != NULL && cloudPublisher->getNumSubscribers() > 0) {
                    publishPointCloudMsg(dispWidth, dispHeight, leftStride, dispStride,
                        leftData, dispData, stamp);
                }

                if(cameraInfoPublisher != NULL && cameraInfoPublisher->getNumSubscribers() > 0) {
                    publishCameraInfo(stamp);
                }

                // Display some simple statistics
                frameNum++;
                if(stamp.sec != lastLogTime.sec) {
                    if(lastLogTime != ros::Time()) {
                        double dt = (stamp - lastLogTime).toSec();
                        double fps = (frameNum - lastLogFrames) / dt;
                        ROS_INFO("%.1f fps", fps);
                    }
                    lastLogFrames = frameNum;
                    lastLogTime = stamp;
                }

                leftData = NULL;
                dispData = NULL;
            }
        }
    }

private:
    // ROS related objects
    ros::NodeHandle nh;
    boost::scoped_ptr<ros::Publisher> cloudPublisher;
    boost::scoped_ptr<ros::Publisher> disparityPublisher;
    boost::scoped_ptr<ros::Publisher> imagePublisher;
    boost::scoped_ptr<ros::Publisher> cameraInfoPublisher;

    // Parameters
    bool intensityChannel;
    bool useTcp;
    bool colorCodeDispMap;
    bool colorCodeLegend;
    bool rosCoordinateSystem;
    std::string port;
    std::string frame;
    std::string host;
    std::string calibFile;
    bool dispWindow;

    // Other members
    int frameNum;
    boost::scoped_ptr<Reconstruct3D> recon3d;
    boost::scoped_ptr<ColorCoder> colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    sensor_msgs::PointCloud2Ptr pointCloudMsg;
    cv::FileStorage calibStorage;
    nerian_sp1::StereoCameraInfoPtr camInfoMsg;
    ros::Time lastCamInfoPublish;
    boost::scoped_ptr<SDLWindow> window;

    /**
     * \brief Publishes a rectified left camera image
     */
    void publishImageMsg(int width, int height, int stride, unsigned char* data,
            ros::Time stamp) {
        cv_bridge::CvImage cvImg;
        cvImg.header.frame_id = frame;
        cvImg.header.stamp = stamp;
        cvImg.header.seq = frameNum;

        cvImg.image = cv::Mat_<unsigned char>(height, width, data, stride);
        sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
        msg->encoding = "mono8";
        imagePublisher->publish(msg);
    }

    /**
     * \brief Publishes the disparity map as 16-bit grayscale image or color coded
     * RGB image
     */
    void publishDispMapMsg(int width, int height, int stride, unsigned char* data,
            ros::Time stamp) {
        cv_bridge::CvImage cvImg;
        cvImg.header.frame_id = frame;
        cvImg.header.stamp = stamp;
        cvImg.header.seq = frameNum;

        cv::Mat_<unsigned short> monoImg(height, width,
            reinterpret_cast<unsigned short*>(data), stride);
        string encoding = "";

        if(!colorCodeDispMap) {
            cvImg.image = monoImg;
            encoding = "mono16";
        } else {
            if(colCoder == NULL) {
                colCoder.reset(new ColorCoder(0, 16*111, true, true));
                if(colorCodeLegend) {
                    // Create the legend
                    colDispMap = colCoder->createLegendBorder(width, height, 1.0/16.0);
                } else {
                    colDispMap = cv::Mat_<cv::Vec3b>(height, width);
                }
            }

            cv::Mat_<cv::Vec3b> dispSection = colDispMap(cv::Rect(0, 0, width, height));
            colCoder->codeImage(monoImg, dispSection);
            cvImg.image = colDispMap;
            encoding = "bgr8";
        }

        if(disparityPublisher->getNumSubscribers() > 0) {
            sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
            msg->encoding = encoding;
            disparityPublisher->publish(msg);
        }

        if(window != NULL) {
            if(window->getSize() != cvImg.image.size()) {
                window->resize(cvImg.image.size());
            }
            window->displayImage(cvImg.image);
            window->processEvents(false);
        }
    }

    /**
     * \brief Reconstructs the 3D locations form the disparity map and publishes them
     * as point cloud.
     */
    void publishPointCloudMsg(int width, int height, int leftStride, int dispStride,
            unsigned char* leftData, unsigned char* dispData, ros::Time stamp) {
        // Get 3D points
        float* pointMap = recon3d->createPointMap(reinterpret_cast<unsigned short*>(dispData),
            width, height, dispStride, 0);

        // Create message object and set header
        pointCloudMsg->header.stamp = stamp;
        pointCloudMsg->header.frame_id = frame;
        pointCloudMsg->header.seq = frameNum;

        // Copy 3D points
        if(pointCloudMsg->data.size() != width*height*4*sizeof(float)) {
            // Allocate buffer
            pointCloudMsg->data.resize(width*height*4*sizeof(float));

            // Set basic data
            pointCloudMsg->width = width;
            pointCloudMsg->height = height;
            pointCloudMsg->is_bigendian = false;
            pointCloudMsg->point_step = 4*sizeof(float);
            pointCloudMsg->row_step = width * pointCloudMsg->point_step;
            pointCloudMsg->is_dense = false;
        }

        memcpy(&pointCloudMsg->data[0], pointMap, width*height*4*sizeof(float));

        // Copy intensity values
        if(intensityChannel) {
            // Get pointers to the beginnig and end of the point cloud
            unsigned char* cloudStart = &pointCloudMsg->data[0];
            unsigned char* cloudEnd = &pointCloudMsg->data[0] + width*height*4*sizeof(float);

            // Get pointer to the current pixel and end of current row
            unsigned char* imagePtr = &leftData[0];
            unsigned char* rowEndPtr = imagePtr + width;

            for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                    cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {
                *cloudPtr = *imagePtr;

                imagePtr++;
                if(imagePtr == rowEndPtr) {
                    // Progress to next row
                    imagePtr += (leftStride - width);
                    rowEndPtr = imagePtr + width;
                }
            }
        }

        cloudPublisher->publish(pointCloudMsg);
    }

    /**
     * \brief Performs all neccessary initializations for point cloud+
     * publishing
     */
    void initPointCloud(ros::NodeHandle& privateNh) {
        // Read disparity-to-depth mapping matrix from calibration data
        std::vector<float> qMatrix;
        calibStorage["Q"] >> qMatrix;
        if(qMatrix.size() != 16) {
            throw std::runtime_error("Q matrix has invalid size!");
        }

        if(rosCoordinateSystem) {
            // Transform Q matrix to match the ROS coordinate system:
            // Swap y/z axis, then swap x/y axis, then invert y and z axis.
            std::vector<float> qNew(16);
            qNew[0] = qMatrix[8];   qNew[1] = qMatrix[9];
            qNew[2] = qMatrix[10];  qNew[3] = qMatrix[11];

            qNew[4] = -qMatrix[0];  qNew[5] = -qMatrix[1];
            qNew[6] = -qMatrix[2];  qNew[7] = -qMatrix[3];

            qNew[8] = -qMatrix[4];  qNew[9] = -qMatrix[5];
            qNew[10] = -qMatrix[6]; qNew[11] = -qMatrix[7];

            qNew[12] = qMatrix[12]; qNew[13] = qMatrix[13];
            qNew[14] = qMatrix[14]; qNew[15] = qMatrix[15];

            qMatrix = qNew;
        }

        // Initialize 3D reconstruction class
        recon3d.reset(new Reconstruct3D(&qMatrix[0]));

        // Initialize message
        pointCloudMsg.reset(new sensor_msgs::PointCloud2);

        // Set channel information.
        sensor_msgs::PointField fieldX;
        fieldX.name ="x";
        fieldX.offset = 0;
        fieldX.datatype = sensor_msgs::PointField::FLOAT32;
        fieldX.count = 1;
        pointCloudMsg->fields.push_back(fieldX);

        sensor_msgs::PointField fieldY;
        fieldY.name ="y";
        fieldY.offset = sizeof(float);
        fieldY.datatype = sensor_msgs::PointField::FLOAT32;
        fieldY.count = 1;
        pointCloudMsg->fields.push_back(fieldY);

        sensor_msgs::PointField fieldZ;
        fieldZ.name ="z";
        fieldZ.offset = 2*sizeof(float);
        fieldZ.datatype = sensor_msgs::PointField::FLOAT32;
        fieldZ.count = 1;
        pointCloudMsg->fields.push_back(fieldZ);

        if(intensityChannel) {
            sensor_msgs::PointField fieldI;
            fieldI.name ="intensity";
            fieldI.offset = 3*sizeof(float);
            fieldI.datatype = sensor_msgs::PointField::UINT8;
            fieldI.count = 1;
            pointCloudMsg->fields.push_back(fieldI);
        }
    }

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(ros::Time stamp) {
        if(camInfoMsg == NULL) {
            // Initialize the camera info structure
            camInfoMsg.reset(new nerian_sp1::StereoCameraInfo);

            camInfoMsg->header.frame_id = frame;
            camInfoMsg->header.seq = 0;

            std::vector<int> sizeVec;
            calibStorage["size"] >> sizeVec;
            if(sizeVec.size() != 2) {
                std::runtime_error("Calibration file format error!");
            }

            camInfoMsg->left_info.header = camInfoMsg->header;
            camInfoMsg->left_info.width = sizeVec[0];
            camInfoMsg->left_info.height = sizeVec[1];
            camInfoMsg->left_info.distortion_model = "plumb_bob";
            calibStorage["D1"] >> camInfoMsg->left_info.D;
            readCalibrationArray("M1", camInfoMsg->left_info.K);
            readCalibrationArray("R1", camInfoMsg->left_info.R);
            readCalibrationArray("P1", camInfoMsg->left_info.P);
            camInfoMsg->left_info.binning_x = 1;
            camInfoMsg->left_info.binning_y = 1;
            camInfoMsg->left_info.roi.do_rectify = false;
            camInfoMsg->left_info.roi.height = 0;
            camInfoMsg->left_info.roi.width = 0;
            camInfoMsg->left_info.roi.x_offset = 0;
            camInfoMsg->left_info.roi.y_offset = 0;

            camInfoMsg->right_info.header = camInfoMsg->header;
            camInfoMsg->right_info.width = sizeVec[0];
            camInfoMsg->right_info.height = sizeVec[1];
            camInfoMsg->right_info.distortion_model = "plumb_bob";
            calibStorage["D2"] >> camInfoMsg->right_info.D;
            readCalibrationArray("M2", camInfoMsg->right_info.K);
            readCalibrationArray("R2", camInfoMsg->right_info.R);
            readCalibrationArray("P2", camInfoMsg->right_info.P);
            camInfoMsg->right_info.binning_x = 1;
            camInfoMsg->right_info.binning_y = 1;
            camInfoMsg->right_info.roi.do_rectify = false;
            camInfoMsg->right_info.roi.height = 0;
            camInfoMsg->right_info.roi.width = 0;
            camInfoMsg->right_info.roi.x_offset = 0;
            camInfoMsg->right_info.roi.y_offset = 0;

            readCalibrationArray("Q", camInfoMsg->Q);
            readCalibrationArray("T", camInfoMsg->T_left_right);
            readCalibrationArray("R", camInfoMsg->R_left_right);
        }

        double dt = (stamp - lastCamInfoPublish).toSec();
        if(dt > 1.0) {
            // Publish once per second
            camInfoMsg->header.stamp = stamp;
            camInfoMsg->left_info.header.stamp = stamp;
            camInfoMsg->right_info.header.stamp = stamp;
            cameraInfoPublisher->publish(camInfoMsg);

            lastCamInfoPublish = stamp;
        }
    }

    /**
     * \brief Reads a vector from the calibration file to a boost:array
     */
    template<class T>
    void readCalibrationArray(const char* key, T& dest) {
        std::vector<double> doubleVec;
        calibStorage[key] >> doubleVec;

        if(doubleVec.size() != dest.size()) {
            std::runtime_error("Calibration file format error!");
        }

        std::copy(doubleVec.begin(), doubleVec.end(), dest.begin());
    }
};

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "nerian_sp1");
        Sp1Node node;
        node.init();
        return node.run();
    } catch(const std::exception& ex) {
        ROS_FATAL("Exception occured: %s", ex.what());
        return 1;
    }
}

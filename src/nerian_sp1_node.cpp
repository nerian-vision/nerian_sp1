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
#include <colorcoder.h>

using namespace std;

/**
 * A simple node that receives data from the SP1 stereo vision system
 * and forwards it to ROS. It can output the following messages
 *
 * - Point cloud of reconstructed 3D locations
 * - Disparity map with optional color coding
 * - Rectified left camera image
 *
 * For configuration please see the provided example launch file.
 */

class Sp1Node {
public:
    Sp1Node(): cloudPublisher(NULL), disparityPublisher(NULL), imagePublisher(NULL), frameNum(0), recon3d(NULL),
        colCoder(0, 16*111, true, true) {
    }

    ~Sp1Node() {
        if(cloudPublisher != NULL) {
            delete cloudPublisher;
        }
        if(disparityPublisher != NULL) {
            delete disparityPublisher;
        }
        if(imagePublisher != NULL) {
            delete imagePublisher;
        }
        if(recon3d != NULL) {
            delete recon3d;
        }
    }

    /**
     * Performs general initializations
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

        // Create publishers
        disparityPublisher = new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_sp1/disparity_map", 10));
        imagePublisher = new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_sp1/left_image", 10));

        if(calibFile == "") {
            ROS_WARN("No camera calibration file configured. Point cloud publishing will be disabled!");
        } else {
            // For point cloud publishing we have to do more initializations
            initPointCloud(privateNh);
        }
    }

    /**
     * The main loop of this node
     */
    int run() {
        ros::Time stamp = ros::Time::now();
        ros::Time lastLogTime = ros::Time::now();
        int lastLogFrames = 0;

        int leftWidth, leftHeight, leftStride;
        unsigned char* leftData = NULL;

        int dispWidth, dispHeight, dispStride;
        unsigned char* dispData = NULL;


        AsyncTransfer asyncTrasnfer(useTcp ? ImageTransfer::TCP_CLIENT : ImageTransfer::UDP_CLIENT,
            host.c_str(), port.c_str());

        while(ros::ok()) {
            int imageNum, width, height, stride;
            ImageProtocol::ImageFormat format;
            unsigned char* data = NULL;

            // Receive image data
            data = asyncTrasnfer.collectReceivedImage(imageNum, width, height, stride, format, 0.5);

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

                if(disparityPublisher->getNumSubscribers() > 0) {
                    publishDispMapMsg(dispWidth, dispHeight, dispStride, dispData, stamp);
                }

                if(cloudPublisher != NULL && cloudPublisher->getNumSubscribers() > 0) {
                    publishPointCloudMsg(dispWidth, dispHeight, leftStride, dispStride,
                        leftData, dispData, stamp);
                }

                // Display some simple statistics
                frameNum++;
                if(stamp.sec != lastLogTime.sec) {
                    double dt = (stamp - lastLogTime).toSec();
                    double fps = (frameNum - lastLogFrames) / dt;
                    ROS_INFO("%.1f fps", fps);
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
    ros::Publisher* cloudPublisher;
    ros::Publisher* disparityPublisher;
    ros::Publisher* imagePublisher;

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

    // Other members
    int frameNum;
    Reconstruct3D* recon3d;
    ColorCoder colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    sensor_msgs::PointCloud2Ptr pointCloudMsg;

    /**
     * Publishes a rectified left camera image
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
     * Publishes the disparity map as 16-bit grayscale image or color coded
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
            if(colDispMap.data == NULL) {
                if(colorCodeLegend) {
                    // Create the legend
                    colDispMap = colCoder.createLegendBorder(width, height, 1.0/16.0);
                } else {
                    colDispMap = cv::Mat_<cv::Vec3b>(height, width);
                }
            }

            cv::Mat_<cv::Vec3b> dispSection = colDispMap(cv::Rect(0, 0, width, height));
            colCoder.codeImage(monoImg, dispSection);
            cvImg.image = colDispMap;
            encoding = "bgr8";
        }

        sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
        msg->encoding = encoding;
        disparityPublisher->publish(msg);
    }

    /**
     * Reconstructs the 3D locations form the disparity map and publishes them
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

            pointCloudMsg->fields[0].count = width * height;
            pointCloudMsg->fields[1].count = width * height;
            pointCloudMsg->fields[2].count = width * height;

            if(intensityChannel) {
                pointCloudMsg->fields[3].count = width * height;
            }
        }

        memcpy(&pointCloudMsg->data[0], pointMap, width*height*4*sizeof(float));

        // Copy intensity values
        if(intensityChannel) {
            for(int y=0; y<height; y++) {
                for(int x=0; x<width; x++) {
                    pointCloudMsg->data[(y*width + x)*4*sizeof(float) + 3*sizeof(float)]
                        = leftData[y * leftStride + x];
                }
            }
        }

        cloudPublisher->publish(pointCloudMsg);
    }

    /**
     * Performs all neccessary initializations for point cloud+
     * publishing
     */
    void initPointCloud(ros::NodeHandle& privateNh) {
        cloudPublisher = new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(
            "/nerian_sp1/point_cloud", 10));

        // Read disparity-to-depth mapping matrix from calibration data
        cv::FileStorage fs;
        if (!fs.open(calibFile, cv::FileStorage::READ)) {
            throw std::runtime_error("Error reading calibration file!");
        }

        std::vector<float> qMatrix;
        fs["Q"] >> qMatrix;
        if(qMatrix.size() != 16) {
            throw std::runtime_error("Q matrix has invalid size!");
        }

        if(rosCoordinateSystem) {
            // Transform Q matrix to match the ROS coordinate system:

            // Swap y / z axis
            swap(qMatrix[4], qMatrix[8]);
            swap(qMatrix[5], qMatrix[9]);
            swap(qMatrix[6], qMatrix[10]);
            swap(qMatrix[7], qMatrix[11]);

            // Swap x / y axis
            swap(qMatrix[0], qMatrix[4]);
            swap(qMatrix[1], qMatrix[5]);
            swap(qMatrix[2], qMatrix[6]);
            swap(qMatrix[3], qMatrix[7]);

            // Invert y axis
            qMatrix[4] = -qMatrix[4];
            qMatrix[5] = -qMatrix[5];
            qMatrix[6] = -qMatrix[6];
            qMatrix[7] = -qMatrix[7];

            // Invert z axis
            qMatrix[8] = -qMatrix[8];
            qMatrix[9] = -qMatrix[9];
            qMatrix[10] = -qMatrix[10];
            qMatrix[11] = -qMatrix[11];
        }

        // Initialize 3D reconstruction class
        recon3d = new Reconstruct3D(&qMatrix[0]);

        // Initialize message
        pointCloudMsg.reset(new sensor_msgs::PointCloud2);

        // Set channel information.
        sensor_msgs::PointField fieldX;
        fieldX.name ="x";
        fieldX.offset = 0;
        fieldX.datatype = sensor_msgs::PointField::FLOAT32;
        pointCloudMsg->fields.push_back(fieldX);

        sensor_msgs::PointField fieldY;
        fieldY.name ="y";
        fieldY.offset = sizeof(float);
        fieldY.datatype = sensor_msgs::PointField::FLOAT32;
        pointCloudMsg->fields.push_back(fieldY);

        sensor_msgs::PointField fieldZ;
        fieldZ.name ="z";
        fieldZ.offset = 2*sizeof(float);
        fieldZ.datatype = sensor_msgs::PointField::FLOAT32;
        pointCloudMsg->fields.push_back(fieldZ);

        if(intensityChannel) {
            sensor_msgs::PointField fieldI;
            fieldI.name ="intensity";
            fieldI.offset = 3*sizeof(float);
            fieldI.datatype = sensor_msgs::PointField::UINT8;
            pointCloudMsg->fields.push_back(fieldI);
        }
    }
};

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "nerian_sp1");
        Sp1Node node;
        node.init();
        return node.run();
    } catch(const std::exception& ex) {
        cerr << "Exception occured: " << ex.what() << endl;
        return 1;
    }
}

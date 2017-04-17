/* ============================================================
 *
 * This file is a part of the RSBVideoReceiver project.
 *
 * Copyright (C) 2011 by Sebastian Wrede <swrede at techfak dot uni-bielefeld dot de>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================ */

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <signal.h>

#include <boost/shared_ptr.hpp>

#include <rsc/Version.h>
#include <rsc/misc/langutils.h>
#include <rsc/logging/LoggerFactory.h>
#include <rsc/threading/InterruptedException.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsc/runtime/TypeStringTools.h>

#include <rsb/filter/ScopeFilter.h>
#include <rsb/converter/Converter.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/MetaData.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv.hpp>

#include <rst/converters/opencv/IplImageConverter.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>


rsb::Informer<IplImage>::Ptr informer;
rsc::logging::LoggerPtr logger;

static void interruptHandler(int /*signal*/) {
	exit(EXIT_SUCCESS);
}

class ReleaseImage {

public:
	ReleaseImage(rsc::logging::LoggerPtr log):logger(log) {};

	/**
	 * Releases an IplImage.
	 * @param img The IplImage pointer.
	 */
	void operator()(IplImage* img) {
		RSCTRACE(logger,"Release IplImage (img = " << img << ", &img = " << &img << ")");
		if (img == 0) return;
		RSCTRACE(logger,"Releasing image (data = " << &(img->imageData) << ", origin = " << &(img->imageDataOrigin) << ")");
		cvReleaseImage(&img);
		img = 0;
	};

private:
    rsc::logging::LoggerPtr logger;
};


static void logIplImageMetadata(rsc::logging::LoggerPtr log, IplImage* image) {
	RSCINFO(log,"Testing IplImage with the following properties:");
	RSCINFO(log,"Size: " << image->imageSize);
	RSCINFO(log,"Depth: " << image->depth);
	RSCINFO(log,"Size (WxH): " << image->width << "x" << image->height);
	RSCINFO(log,"#Channels: " << image->nChannels);
	RSCINFO(log,"Colormodel: " << image->colorModel);
	RSCINFO(log,"ROI: " << image->roi);
	RSCINFO(log,"Data order (0==Interleaved): " << image->dataOrder);
	RSCINFO(log,"Width-Step: " << image->widthStep);
}

void image_callback( const sensor_msgs::Image::ConstPtr& image){

    // convert sensor image to cv::Mat
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image, "bgr8");
    cv::Mat frame = img_ptr->image;

    // convert to IplImage*
    int depth=frame.depth();
    int channels=frame.channels();
    IplImage *iplImage=cvCreateImage(cvSize(frame.cols,frame.rows),8,channels);
    iplImage->widthStep = frame.step;
    iplImage->imageData=(char*)frame.data;

    // display using C-API to visualize the image to be streamed
    cv::imshow("Published Image", frame);
    cv::waitKey(1);

    // warp in shared_ptr
    boost::shared_ptr<IplImage> img = boost::shared_ptr<IplImage>(iplImage, ReleaseImage(logger));

    // send image to RSB, memory is deallocated through shared_ptr
    informer->publish(img);

}


int main(int argc, char **argv) {

#if RSC_VERSION_NUMERIC >= 000600
    rsc::logging::LoggerFactory& loggerFactory = rsc::logging::LoggerFactory::getInstance();
	loggerFactory.reconfigure(rsc::logging::Logger::LEVEL_INFO);
#else
    rsc::logging::LoggerFactory* loggerFactory = rsc::logging::LoggerFactory::getInstance();
	loggerFactory->reconfigure(rsc::logging::Logger::LEVEL_INFO);
#endif

    //init node
    ros::init(argc, argv, "video_bridge");
    ros::NodeHandle nh;

    logger = rsc::logging::Logger::getLogger("video_sender.main");

	RSCINFO(logger,"Starting video_bridge");

	// Register the converters
	rsb::converter::Converter<std::string>::Ptr imageP(
			new rst::converters::opencv::IplImageConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(imageP);


	// Image scope
	std::string imageScopeAddress("/video");

	// Handle ctrl+c interrupts
	signal(SIGINT, interruptHandler);

	// Create informer
	rsb::Factory &factory = rsb::getFactory();
    informer = factory.createInformer<IplImage>(imageScopeAddress);

    cv::namedWindow("Published Image", CV_WINDOW_AUTOSIZE);

    // create subscriber
        ros::Subscriber sub = nh.subscribe("image_raw", 1000, image_callback);

    ros::spin();
	return EXIT_SUCCESS;
}



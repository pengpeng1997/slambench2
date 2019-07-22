/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "include/LifelongSLAM.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GyroSensor.h>
#include <io/sensor/OdomSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>


#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <fstream>

using namespace slambench::io ;

/*
 *
 * The dataset folder contains :
 * > accelerometer.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */


bool analyseLifelongSLAMFolder(const std::string &dirname) {

	static const std::vector<std::string> requirements = {
			"accel.csv",
            "gyro.csv",
            "odom.csv",
			"bgr.txt",
			"bgr",
			"depth.txt",
			"depth",
			"GroundTruth.csv"
	};

	try {
		if ( !boost::filesystem::exists( dirname ) ) return false;

		boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
		for ( auto requirement : requirements ) {
			bool seen = false;

			for ( boost::filesystem::directory_iterator itr( dirname ); itr != end_itr; ++itr ) {
				if (requirement == itr->path().filename()) {
					seen = true;
				}
			}

			if (!seen) {
				std::cout << "File not found: <dataset_dir>/" << requirement << std::endl;
				return false;
			}
		}
	} catch (boost::filesystem::filesystem_error& e)  {
		std::cerr << "I/O Error with directory " << dirname << std::endl;
		std::cerr << e.what() << std::endl;
		return false;
	}

	return true;
}


bool loadLifelongSLAMDepthData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics,const CameraSensor::distortion_coefficients_t &distortion,  const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type) {

	DepthSensor *depth_sensor = new DepthSensor("Depth");
	depth_sensor->Index = 0;
	depth_sensor->Width = 848;
	depth_sensor->Height = 480;
	depth_sensor->FrameFormat = frameformat::Raster;
	depth_sensor->PixelFormat = pixelformat::D_I_16;
	depth_sensor->DisparityType = disparity_type;
	depth_sensor->Description = "Depth";
	depth_sensor->CopyPose(pose);
	depth_sensor->CopyIntrinsics(intrinsics);
	depth_sensor->CopyDisparityParams(disparity_params);
	depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
	depth_sensor->CopyRadialTangentialDistortion(distortion);
	depth_sensor->Index = file.Sensors.size();
	depth_sensor->Rate = 30.0;

	file.Sensors.AddSensor(depth_sensor);

	std::string line;

	std::ifstream infile(dirname + "/" + "depth.txt");

	boost::smatch match;

	while (std::getline(infile, line)){


		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

		  int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
		  std::string depthfilename = match[3];

		  ImageFileFrame *depth_frame = new ImageFileFrame();
		  depth_frame->FrameSensor  = depth_sensor;
		  depth_frame->Timestamp.S  = timestampS;
		  depth_frame->Timestamp.Ns = timestampNS;

		  std::stringstream frame_name;
		  frame_name << dirname << "/" << depthfilename ;
		  depth_frame->Filename = frame_name.str();

		  if(access(depth_frame->Filename.c_str(), F_OK) < 0) {
				printf("No depth image for frame (%s)\n", frame_name.str().c_str());
				perror("");
				return false;
		  }

		  file.AddFrame(depth_frame);



		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}
	}
	return true;
}


bool loadLifelongSLAMRGBData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

	CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
	rgb_sensor->Index = 0;
	rgb_sensor->Width = 848;
	rgb_sensor->Height = 480;
	rgb_sensor->FrameFormat = frameformat::Raster;
	rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
	rgb_sensor->Description = "RGB";
	rgb_sensor->CopyPose(pose);
	rgb_sensor->CopyIntrinsics(intrinsics);
	rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
	rgb_sensor->CopyRadialTangentialDistortion(distortion);
	rgb_sensor->Index =file.Sensors.size();
	rgb_sensor->Rate = 30.0;

	file.Sensors.AddSensor(rgb_sensor);

	std::string line;

	std::ifstream infile(dirname + "/" + "bgr.txt");

	boost::smatch match;

	while (std::getline(infile, line)){


		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

		  int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
		  std::string rgbfilename = match[3];

		  ImageFileFrame *rgb_frame = new ImageFileFrame();
		  rgb_frame->FrameSensor = rgb_sensor;
		  rgb_frame->Timestamp.S  = timestampS;
		  rgb_frame->Timestamp.Ns = timestampNS;

		  std::stringstream frame_name;
		  frame_name << dirname << "/" << rgbfilename ;
		  rgb_frame->Filename = frame_name.str();

		  if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
		    printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
		    perror("");
		    return false;
		  }

		  file.AddFrame(rgb_frame);

		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}

	}
	return true;
}

bool loadLifelongSLAMGreyData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

	CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
	grey_sensor->Index = 0;
	grey_sensor->Width = 848;
	grey_sensor->Height = 480;
	grey_sensor->FrameFormat = frameformat::Raster;
	grey_sensor->PixelFormat = pixelformat::G_I_8;
	grey_sensor->Description = "Grey";

	grey_sensor->CopyPose(pose);
	grey_sensor->CopyIntrinsics(intrinsics);
	grey_sensor->CopyRadialTangentialDistortion(distortion);
	grey_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
	grey_sensor->Index =file.Sensors.size();
	grey_sensor->Rate = 30.0;

	file.Sensors.AddSensor(grey_sensor);

	std::string line;

	std::ifstream infile(dirname + "/" + "bgr.txt");

	boost::smatch match;

	while (std::getline(infile, line)){


		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {

		  int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
		  std::string rgbfilename = match[3];

		  ImageFileFrame *grey_frame = new ImageFileFrame();
		  grey_frame->FrameSensor = grey_sensor;
		  grey_frame->Timestamp.S  = timestampS;
		  grey_frame->Timestamp.Ns = timestampNS;

		  std::stringstream frame_name;
		  frame_name << dirname << "/" << rgbfilename ;
		  grey_frame->Filename = frame_name.str();

		  if(access(grey_frame->Filename.c_str(), F_OK) < 0) {
		    printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
		    perror("");
		    return false;
		  }

		  file.AddFrame(grey_frame);

		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}

	}
	return true;
}


bool loadLifelongSLAMGroundTruthData(const std::string &dirname , SLAMFile &file) {

	GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
	gt_sensor->Index = file.Sensors.size();
	gt_sensor->Description = "GroundTruthSensor";
	file.Sensors.AddSensor(gt_sensor);

	if(!gt_sensor) {
		std::cout << "gt sensor not found..." << std::endl;
		return false;
	} else {
		std::cout << "gt sensor created..." << std::endl;
	}


	std::string line;

	boost::smatch match;
	std::ifstream infile(dirname + "/" + "GroundTruth.csv");

	while (std::getline(infile, line)){
		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            
          int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
          std::string temp3 = match[3], temp4 = match[4], temp5 = match[5], temp6 = match[6], temp7 = match[7], temp8 = match[8], temp9 = match[9];
          char *p3=(char*)temp3.data();
          char *p4=(char*)temp4.data();
          char *p5=(char*)temp5.data();
          char *p6=(char*)temp6.data();
          char *p7=(char*)temp7.data();
          char *p8=(char*)temp8.data();
          char *p9=(char*)temp9.data();

		  float tx =  std::atof(p3);
		  float ty =  std::atof(p4);
		  float tz =  std::atof(p5);
		  float QX =  std::atof(p6);
		  float QY =  std::atof(p7);
		  float QZ =  std::atof(p8);
		  float QW =  std::atof(p9);

		  Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW,QX,QY,QZ).toRotationMatrix();
		  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
		  pose.block(0,0,3,3) = rotationMat;

		  pose.block(0,3,3,1) << tx , ty , tz;


		  SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
		  gt_frame->FrameSensor = gt_sensor;
		  gt_frame->Timestamp.S  = timestampS;
		  gt_frame->Timestamp.Ns = timestampNS;
		  gt_frame->Data = malloc(gt_frame->GetSize());


		  memcpy(gt_frame->Data,pose.data(),gt_frame->GetSize());

		  file.AddFrame(gt_frame);


		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}


	}
	return true;
}


bool loadLifelongSLAMAccelerometerData(const std::string &dirname , SLAMFile &file) {

	AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
	accelerometer_sensor->Index = file.Sensors.size();
	accelerometer_sensor->Description = "AccelerometerSensor";
	file.Sensors.AddSensor(accelerometer_sensor);

	if(!accelerometer_sensor) {
		std::cout << "accelerometer_sensor not found..." << std::endl;
		return false;
	}else {
		std::cout << "accelerometer_sensor created..." << std::endl;
	}


	std::string line;

	  boost::smatch match;
	  std::ifstream infile(dirname + "/" + "accel.csv");

	while (std::getline(infile, line)){

		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {

		 int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
          std::string temp3 = match[3], temp4 = match[4], temp5 = match[5];
          char *p3=(char*)temp3.data();
          char *p4=(char*)temp4.data();
          char *p5=(char*)temp5.data();

		  float ax =  std::atof(p3);
		  float ay =  std::atof(p4);
		  float az =  std::atof(p5);

		  SLAMInMemoryFrame *accelerometer_frame = new SLAMInMemoryFrame();
		  accelerometer_frame->FrameSensor = accelerometer_sensor;
		  accelerometer_frame->Timestamp.S  = timestampS;
		  accelerometer_frame->Timestamp.Ns = timestampNS;
		  accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
		  ((float*)accelerometer_frame->Data)[0] = ax;
		  ((float*)accelerometer_frame->Data)[1] = ay;
		  ((float*)accelerometer_frame->Data)[2] = az;

		  file.AddFrame(accelerometer_frame);


		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}


	}
	return true;
}


bool loadLifelongSLAMGyroData(const std::string &dirname , SLAMFile &file) {

	GyroSensor *gyro_sensor = new GyroSensor("Gyro");
	gyro_sensor->Index = file.Sensors.size();
	gyro_sensor->Description = "GyroSensor";
	file.Sensors.AddSensor(gyro_sensor);

	if(!gyro_sensor) {
		std::cout << "gyro_sensor not found..." << std::endl;
		return false;
	}else {
		std::cout << "gyro_sensor created..." << std::endl;
	}


	std::string line;

	  boost::smatch match;
	  std::ifstream infile(dirname + "/" + "gyro.csv");

	while (std::getline(infile, line)){

		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {

		 int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());

          std::string temp3 = match[3], temp4 = match[4], temp5 = match[5];
          char *p3=(char*)temp3.data();
          char *p4=(char*)temp4.data();
          char *p5=(char*)temp5.data();

		  float wx =  std::atof(p3);
		  float wy =  std::atof(p4);
		  float wz =  std::atof(p5);

		  SLAMInMemoryFrame *gyro_frame = new SLAMInMemoryFrame();
		  gyro_frame->FrameSensor = gyro_sensor;
		  gyro_frame->Timestamp.S  = timestampS;
		  gyro_frame->Timestamp.Ns = timestampNS;
		  gyro_frame->Data = malloc(gyro_frame->GetSize());
		  ((float*)gyro_frame->Data)[0] = wx;
		  ((float*)gyro_frame->Data)[1] = wy;
		  ((float*)gyro_frame->Data)[2] = wz;

		  file.AddFrame(gyro_frame);


		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}


	}
	return true;
}

bool loadLifelongSLAMOdomData(const std::string &dirname , SLAMFile &file) {

	OdomSensor *odom_sensor = new OdomSensor("Odom");
	odom_sensor->Index = file.Sensors.size();
	odom_sensor->Description = "OdomSensor";
	file.Sensors.AddSensor(odom_sensor);

	if(!odom_sensor) {
		std::cout << "odom_sensor not found..." << std::endl;
		return false;
	}else {
		std::cout << "odom_sensor created..." << std::endl;
	}


	std::string line;

	  boost::smatch match;
	  std::ifstream infile(dirname + "/" + "odom.csv");

	while (std::getline(infile, line)){

		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?),([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {

		int timestampS = std::stoi(match[1]);
		int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
        std::string temp3 = match[3], temp4 = match[4], temp5 = match[5], temp6 = match[6], temp7 = match[7], temp8 = match[8], temp9 = match[9],
                    temp10 = match[10], temp11 = match[11], temp12 = match[12], temp13 = match[13], temp14 = match[14], temp15 = match[15];
          char *p3=(char*)temp3.data();
          char *p4=(char*)temp4.data();
          char *p5=(char*)temp5.data();
          char *p6=(char*)temp6.data();
          char *p7=(char*)temp7.data();
          char *p8=(char*)temp8.data();
          char *p9=(char*)temp9.data();
          char *p10=(char*)temp10.data();
          char *p11=(char*)temp11.data();
          char *p12=(char*)temp12.data();
          char *p13=(char*)temp13.data();
          char *p14=(char*)temp14.data();
          char *p15=(char*)temp15.data();

		  float px =  std::atof(p3);
		  float py =  std::atof(p4);
		  float pz =  std::atof(p5);
		  float ox =  std::atof(p6);
		  float oy =  std::atof(p7);
		  float oz =  std::atof(p8);
		  float ow =  std::atof(p9);
          float lx =  std::atof(p10);
		  float ly =  std::atof(p11);
		  float lz =  std::atof(p12);
          float ax =  std::atof(p13);
		  float ay =  std::atof(p14);
		  float az =  std::atof(p15);


		  SLAMInMemoryFrame *odom_frame = new SLAMInMemoryFrame();
		  odom_frame->FrameSensor = odom_sensor;
		  odom_frame->Timestamp.S  = timestampS;
		  odom_frame->Timestamp.Ns = timestampNS;
		  odom_frame->Data = malloc(odom_frame->GetSize());
		  ((float*)odom_frame->Data)[0] = px;
		  ((float*)odom_frame->Data)[1] = py;
		  ((float*)odom_frame->Data)[2] = pz;
          ((float*)odom_frame->Data)[3] = ox;
		  ((float*)odom_frame->Data)[4] = oy;
		  ((float*)odom_frame->Data)[5] = oz;
          ((float*)odom_frame->Data)[6] = ow;
          ((float*)odom_frame->Data)[7] = lx;
		  ((float*)odom_frame->Data)[8] = ly;
		  ((float*)odom_frame->Data)[9] = lz;
          ((float*)odom_frame->Data)[10] = ax;
		  ((float*)odom_frame->Data)[11] = ay;
		  ((float*)odom_frame->Data)[12] = az;

		  file.AddFrame(odom_frame);


		} else {
		  std::cerr << "Unknown line:" << line << std::endl;
		  return false;
		}


	}
	return true;
}


SLAMFile* LifelongSLAMReader::GenerateSLAMFile () {

	if(!(grey || rgb || depth)) {
		std::cerr <<  "No sensors defined\n";
		return nullptr;
	}

	std::string dirname = input;

	if (!analyseLifelongSLAMFolder(dirname))	{
		std::cerr << "Invalid folder." << std::endl;
		return nullptr;
	}


	SLAMFile * slamfilep = new SLAMFile();
	SLAMFile & slamfile  = *slamfilep;

	Sensor::pose_t pose = Eigen::Matrix4f::Identity();

	//////  Default are freiburg1

    CameraSensor::intrinsics_t intrinsics_rgb;
	DepthSensor::intrinsics_t intrinsics_depth;

	CameraSensor::distortion_coefficients_t distortion_rgb;
	DepthSensor::distortion_coefficients_t distortion_depth;

	DepthSensor::disparity_params_t disparity_params =  {0.001,0.0};
	DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;


	// if (dirname.find("freiburg1") != std::string::npos) {
	// 	std::cout << "This dataset is assumed to be using freiburg1." << std::endl;

		// for (int i = 0; i < 4; i++) {
		// 	intrinsics_rgb[i]   = fr1_intrinsics_rgb[i];
		// 	intrinsics_depth[i] = fr1_intrinsics_depth[i];
		// 	distortion_rgb[i]   = fr1_distortion_rgb[i];
		// 	distortion_depth[i] = fr1_distortion_depth[i];
		// }

	// } else if (dirname.find("freiburg2") != std::string::npos) {
	// 	std::cout << "This dataset is assumed to be using freiburg2." << std::endl;
		for (int i = 0; i < 4; i++) {
			intrinsics_rgb[i]   = fr2_intrinsics_rgb[i];
			intrinsics_depth[i] = fr2_intrinsics_depth[i];
			distortion_rgb[i]   = fr2_distortion_rgb[i];
			distortion_depth[i] = fr2_distortion_depth[i];
		}

	// } else  {
	// 	std::cout << "Camera calibration might be wrong !." << std::endl;
	// }




	/**
	 * load Depth
	 */

	if(depth && !loadLifelongSLAMDepthData(dirname, slamfile,pose,intrinsics_depth,distortion_depth,disparity_params,disparity_type)) {
		std::cout << "Error while loading LifelongSLAM depth information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load Grey
	 */

	if(grey && !loadLifelongSLAMGreyData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb)) {
		std::cout << "Error while loading LifelongSLAM Grey information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load RGB
	 */

	if(rgb && !loadLifelongSLAMRGBData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb)) {
		std::cout << "Error while loading LifelongSLAM RGB information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load GT
	 */
	if(gt && !loadLifelongSLAMGroundTruthData(dirname, slamfile)) {
		std::cout << "Error while loading gt information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load Accelerometer: This one failed ???what???
	 */
	if(accelerometer && !loadLifelongSLAMAccelerometerData(dirname, slamfile)) {
		std::cout << "Error while loading Accelerometer information." << std::endl;
		delete slamfilep;
		return nullptr;

	}
    //I write for gyro and odom like acc, now you tell me it failed...

	if(gyro && !loadLifelongSLAMGyroData(dirname, slamfile)) {
		std::cout << "Error while loading Gyro information." << std::endl;
		delete slamfilep;
		return nullptr;

	}

    if(odom && !loadLifelongSLAMOdomData(dirname, slamfile)) {
		std::cout << "Error while loading Odom information." << std::endl;
		delete slamfilep;
		return nullptr;

	}

	return slamfilep;
	}

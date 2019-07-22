//Copyright(c) 2019 Intel Corporation.

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_LIFELONGSLAM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_LIFELONGSLAM_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "../../dataset-tools/include/DatasetReader.h"

namespace slambench {

namespace io {

class LifelongSLAMReader :  public DatasetReader {
	
private :
   //to be assigned values
   	

	// I took those numbers from the TUM dataset paper
	//static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb   = { 0.80828125, 1.076041667, 0.4978125, 0.531875 };
	//static constexpr DepthSensor::intrinsics_t  fr1_intrinsics_depth = { 0.92359375, 1.229375,    0.5171875, 0.4875   };
	static constexpr CameraSensor::intrinsics_t fr2_intrinsics_rgb   = {0.7210506943036925, 0.902508290608724, 0.7210916483177329, 0.5197354634602864};
	static constexpr DepthSensor::intrinsics_t  fr2_intrinsics_depth = {0.49781097556060216, 0.8896006902058919,  0.49781097556060216, 0.5108768781026204};
	static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb   = {0.9799446625, 1.3110521729166666, 0.5060022234375, 0.53062055};
	static constexpr DepthSensor::intrinsics_t  fr1_intrinsics_depth = {0.9799446625, 1.3110521729166666, 0.5060022234375, 0.53062055};

	// I took those numbers from ORBSLAM2 examples

	static constexpr float fr1_fps =  30.0 ;
	static constexpr float fr1_bf =  40.0 ;
	static constexpr float fr1_ThDepth =  40.0 ;
	static constexpr float fr1_DepthMapFactor =  5000.0 ;

	static constexpr float fr2_fps =  30.0 ;
	static constexpr float fr2_bf =  40.0 ;
	static constexpr float fr2_ThDepth =  40.0 ;
	static constexpr float fr2_DepthMapFactor =  5208.0 ;


	//static constexpr CameraSensor::distortion_coefficients_t fr1_distortion_rgb   = { 0.262383 ,	 -0.953104,	 -0.005358,	 0.002628 ,	 1.163314  };
	static constexpr CameraSensor::distortion_coefficients_t fr2_distortion_rgb   = { 0.0, 0.0, 0.0, 0.0, 0.0 };

	//static constexpr DepthSensor::distortion_coefficients_t  fr1_distortion_depth = { 0.262383,	 -0.953104,	 -0.005358,	 0.002628, 	 1.163314   };
	static constexpr DepthSensor::distortion_coefficients_t  fr2_distortion_depth = {  0.0, 0.0, 0.0, 0.0, 0.0};

	static constexpr CameraSensor::distortion_coefficients_t fr1_distortion_rgb   = { 0.096471, -0.112393, -0.000752, 0.003361, 0.000000  };
	static constexpr DepthSensor::distortion_coefficients_t  fr1_distortion_depth = { 0.096471, -0.112393, -0.000752, 0.003361, 0.000000  };
	
public :
	std::string input;
	bool rgb = true, depth = true, gt = true, accelerometer = false, gyro = false, odom = false, grey = true;

	LifelongSLAMReader(std::string name) : DatasetReader(name) {

		this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the LifelongSLAM dataset directory",   &this->input, NULL));
		this->addParameter(TypedParameter<bool>("grey",     "grey",       "set to true or false to specify if the GREY stream need to be include in the slam file.",   &this->grey, NULL));
		this->addParameter(TypedParameter<bool>("rgb",     "rgb",       "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->rgb, NULL));
		this->addParameter(TypedParameter<bool>("depth",     "depth",       "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
		this->addParameter(TypedParameter<bool>("gt",     "gt",       "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",   &this->gt, NULL));
		this->addParameter(TypedParameter<bool>("acc",    "accelerometer",       "set to true or false to specify if the ACCELEROMETER stream need to be include in the slam file.",   &this->accelerometer, NULL));
		this->addParameter(TypedParameter<bool>("gyro",    "gyro",       "set to true or false to specify if the GYRO stream need to be include in the slam file.",   &this->gyro, NULL));
		this->addParameter(TypedParameter<bool>("odom",    "odom",       "set to true or false to specify if the ODOMETRY stream need to be include in the slam file.",   &this->odom, NULL));
		//要合并成imu
	}


	SLAMFile* GenerateSLAMFile ();

};
//copyed from TUM.h
}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_LIFELONGSLAM_H_ */
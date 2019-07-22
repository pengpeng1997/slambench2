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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace slambench {

namespace io {
	const int INF = 99;
    typedef std::pair<int, int> trans_direction;

    Eigen::Matrix4f compute_trans_matrix(std::string input_name_1, std::string input_name_2, std::string filename);

class LifelongSLAMReader :  public DatasetReader {

public :
	std::string input;
	bool rgb = true, depth = true, gt = true, stereo = false, accelerometer = true, gyro = true, odom = true, grey = true;

	LifelongSLAMReader(std::string name) : DatasetReader(name) {

		this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the LifelongSLAM dataset directory",   &this->input, NULL));
		this->addParameter(TypedParameter<bool>("grey",     "grey",       "set to true or false to specify if the GREY stream need to be include in the slam file.",   &this->grey, NULL));
		this->addParameter(TypedParameter<bool>("rgb",     "rgb",       "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->rgb, NULL));
		this->addParameter(TypedParameter<bool>("depth",     "depth",       "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
		this->addParameter(TypedParameter<bool>("sgrey",     "stereo-fisheye-grey",       "set to true or false to specify if the STEREO GREY stream need to be include in the slam file.",   &this->stereo, NULL));
		this->addParameter(TypedParameter<bool>("acc",    "accelerometer",       "set to true or false to specify if the ACCELEROMETER stream need to be include in the slam file.",   &this->accelerometer, NULL));
		this->addParameter(TypedParameter<bool>("gyro",    "gyro",       "set to true or false to specify if the GYRO stream need to be include in the slam file.",   &this->gyro, NULL));
		this->addParameter(TypedParameter<bool>("odom",    "odom",       "set to true or false to specify if the ODOMETRY stream need to be include in the slam file.",   &this->odom, NULL));
		this->addParameter(TypedParameter<bool>("gt",     "gt",       "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",   &this->gt, NULL));
		
	}


	SLAMFile* GenerateSLAMFile ();

};
//copyed from TUM.h
}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_LIFELONGSLAM_H_ */
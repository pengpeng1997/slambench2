/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef SLAMBENCH_CONFIGURATION_LIFELONG_H_
#define SLAMBENCH_CONFIGURATION_LIFELONG_H_


#include <SLAMBenchLibraryHelperLifelong.h>
#include "SLAMBenchConfiguration.h"



class SLAMBenchConfigurationLifelong : public SLAMBenchConfiguration {
public:
    SLAMBenchConfigurationLifelong();

private :

    std::vector<slambench::io::InputInterface*> input_interfaces;

public :

	static void compute_loop_algorithm(SLAMBenchConfiguration* config, bool *stay_on, SLAMBenchUI *ui);
    void InitGroundtruth(bool with_point_cloud = true);
    bool add_input(std::string input_file);
    void add_slam_library(std::string so_file, std::string identifier);
	slambench::io::InputInterface *GetCurrentInputInterface();
	const slambench::io::SensorCollection &GetSensors();
	void AddInputInterface(slambench::io::InputInterface *input_ref);
    void init_sensors();
    void LoadNextInputInterface();

};





#endif /* SLAMBENCH_CONFIGURATION_LIFELONG_H_ */

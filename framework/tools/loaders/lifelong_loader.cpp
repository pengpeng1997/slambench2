/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <SLAMBenchConfiguration.h>
#include <metrics/ATEMetric.h>
#include <metrics/RPEMetric.h>
#include <outputs/TrajectoryAlignmentMethod.h>
#include <outputs/OutputManagerWriter.h>
#include <metrics/DurationMetric.h>
#include <metrics/PowerMetric.h>
#include <metrics/MemoryMetric.h>
#include "ColumnWriter.h"
#include <SLAMBenchException.h>
#include <SLAMBenchConfigurationLifelong.h>


std::string default_output_filename;
std::string output_filename;

std::string alignment_technique = "original";
std::string default_alignment_technique = "original";
TypedParameter<std::string> file_output_parameter ("fo", "file-output", "File to write slamfile containing outputs", &output_filename, &default_output_filename);
TypedParameter<std::string> alignment_type_parameter("a",     "alignment-technique",      "Select an alignment technique by name, if not found, default used (default,new).", &alignment_technique, &default_alignment_technique);

int main(int argc, char * argv[])
{

	try {

		auto config = new SLAMBenchConfigurationLifelong();

		//***************************************************************************************
		// Start the argument processing
		//***************************************************************************************

		config->addParameter(file_output_parameter);
		config->GetParameterManager().ReadArgumentsOrQuit(argc, argv);

		//***************************************************************************************
		// At this point the datasets/libraries/sensors are loaded with their arguments set.
		//***************************************************************************************




		//***************************************************************************************
		// We initialise the configuration, means to retrieve groundtruth and set the alignement
		//***************************************************************************************

		config->InitGroundtruth(false);


		//***************************************************************************************
		// We prepare the logging and create the global metrics
		//***************************************************************************************

		config->start_statistics();
	
		//***************************************************************************************
		// We init the algos now because we need their output already
		// TODO: if pose and map were by default we could init the algo much later,
		//       thus move memory metric later
		//***************************************************************************************

		config->InitAlgorithms();
		config->alignment_technique_ = alignment_technique;
		config->init_cw();
		

	
		//***************************************************************************************
		// We run the experiment
		//***************************************************************************************

		SLAMBenchConfigurationLifelong::compute_loop_algorithm (config,NULL,NULL);

		//***************************************************************************************
		// End of experiment, we output the map
		//***************************************************************************************

		// TODO: Only one output file does not do the job for more than one SLAM systems, output directory maybe ?

		SLAMBenchLibraryHelper *main_lib = nullptr;

		if(output_filename != "" && config->GetLoadedLibs().size() > 1) {
			std::cerr << "Can only write outputs to file when there is only one lib loaded" << std::endl;
			return 1;
		} else if(output_filename != "") {
			// enable all writeable outputs
			SLAMBenchLibraryHelper *lib = config->GetLoadedLibs().front();
			main_lib = lib;

			lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->SetActive(true);
		}


		if(output_filename != "") {
			slambench::TimeStamp timestamp = main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->GetMostRecentValue().first;
			main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD)->SetActive(true);
			main_lib->c_sb_update_outputs(main_lib, &timestamp);

			std::cout << "Writing outputs to " << output_filename << std::endl;
			slambench::outputs::OutputManagerWriter omw;
			SLAMBenchLibraryHelper *lib = *config->GetLoadedLibs().begin();

			omw.Write(lib->GetOutputManager(), output_filename);
			std::cout << "Done writing outputs." << std::endl;
		}


		std::cout << "End of program." << std::endl;

		delete config;


	} catch (const SLAMBenchException& e) {

		std::cout << "An error occurred during the execution." << std::endl;
		std::cout << e.what() << std::endl;

	}

	return 0;
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfigurationLifelong.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"

#include <io/FrameBufferSource.h>
#include <io/openni2/ONI2FrameStream.h>
#include <io/openni2/ONI2InputInterface.h>

#include <io/openni15/ONI15FrameStream.h>
#include <io/openni15/ONI15InputInterface.h>

#include <io/InputInterface.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>

#include <metrics/Metric.h>
#include <metrics/ATEMetric.h>
#include <metrics/PowerMetric.h>

#include <values/Value.h>
#include <outputs/Output.h>

#include <boost/optional.hpp>

#include <iostream>
#include <stdexcept>

#include <iomanip>
#include <map>



#include <dlfcn.h>
#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}


void SLAMBenchConfigurationLifelong::add_slam_library(std::string so_file, std::string identifier) {

    std::cerr << "new library name: " << so_file  << std::endl;

    void* handle = dlopen(so_file.c_str(),RTLD_LAZY);

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << std::endl;
        exit(1);
    }

    char *start=(char *)so_file.c_str();
    char *iter = start;
    while(*iter!=0){
        if(*iter=='/')
            start = iter+1;
        iter++;
    }
    std::string libName=std::string(start);
    libName=libName.substr(3, libName.length()-14);
    auto lib_ptr = new SLAMBenchLibraryHelperLifelong (identifier, libName, this->get_log_stream(),  this->GetCurrentInputInterface());
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_slam_configuration);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_frame);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_once);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_clean_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_outputs);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_relocalize);
    this->slam_libs.push_back(lib_ptr);


    size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    if (!lib_ptr->c_sb_new_slam_configuration(lib_ptr)) {
        std::cerr << "Configuration construction failed." << std::endl;
        exit(1);
    }
    size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    std::cerr << "Configuration consumed " << post-pre  << " bytes" << std::endl;

    GetParameterManager().AddComponent(lib_ptr);

    std::cerr << "SLAM library loaded: " << so_file << std::endl;

}

void libs_callback(Parameter* param, ParameterComponent* caller) {

    SLAMBenchConfigurationLifelong* config = dynamic_cast<SLAMBenchConfigurationLifelong*> (caller);

    TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (std::string library_name : parameter->getTypedValue()) {


        std::string library_filename   = "";
        std::string library_identifier = "";


        auto pos = library_name.find("=");
        if (pos != std::string::npos)  {
            library_filename   = library_name.substr(0, pos);
            library_identifier = library_name.substr(pos+1);
        } else {
            library_filename = library_name;
        }
        config->add_slam_library(library_filename,library_identifier);
    }
}

void dataset_callback(Parameter* param, ParameterComponent* caller) {

    SLAMBenchConfigurationLifelong* config = dynamic_cast<SLAMBenchConfigurationLifelong*> (caller);

    if (!config) {
        std::cerr << "Extremely bad usage of the force..." << std::endl;
        std::cerr << "It happened that a ParameterComponent* can not be turned into a SLAMBenchConfiguration*..." << std::endl;
        exit(1);
    }

    TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (std::string input_name : parameter->getTypedValue()) {
        config->add_input(input_name);
    }
    config->init_sensors();
}

bool SLAMBenchConfigurationLifelong::add_input(std::string input_file) {

    FILE * input_desc = fopen(input_file.c_str(), "r");
    if (input_desc == nullptr) {
        throw std::logic_error( "Could not open the input file" );
    }
    this->AddInputInterface(new slambench::io::FileStreamInputInterface(input_desc, new slambench::io::SingleFrameBufferSource()));

    return true;
}


SLAMBenchConfigurationLifelong::SLAMBenchConfigurationLifelong  () {

    initialised_ = false;
    this->input_interface = NULL;
    this->log_stream = NULL;
    this->slam_library_names = {};

    // Run Related
    this->addParameter(TypedParameter<unsigned int>("fl",     "frame-limit",      "last frame to compute",                   &this->frame_limit, &default_frame_limit));
    this->addParameter(TypedParameter<std::string>("o",     "log-file",      "Output log file",                   &this->log_file, &default_log_file, log_callback));
    this->addParameter(TypedParameter<std::vector<std::string>>("i",     "input" ,        "Specify the input file or mode." ,  &this->input_files, &default_input_files , dataset_callback));
    this->addParameter(TypedParameter<std::vector<std::string> >("load",  "load-slam-library" , "Load a specific SLAM library."     , &this->slam_library_names, &default_slam_libraries , libs_callback ));
    this->addParameter(TriggeredParameter("dse",   "dse",    "Output solution space of parameters.",    dse_callback));
    this->addParameter(TriggeredParameter("h",     "help",   "Print the help.", help_callback));
    this->addParameter(TypedParameter<bool>("realtime",     "realtime-mode",      "realtime frame loading mode",                   &this->realtime_mode_, &default_is_false));
    this->addParameter(TypedParameter<double>("realtime-mult",     "realtime-multiplier",      "realtime frame loading mode",                   &this->realtime_mult_, &default_realtime_mult));

    param_manager_.AddComponent(this);

};

void SLAMBenchConfigurationLifelong::InitGroundtruth(bool with_point_cloud) {

    if(initialised_) {
        return;
    }
    auto interface = GetCurrentInputInterface();
    if(interface != nullptr) {
        auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(interface->GetFrames());
        input_stream_ = gt_buffering_stream;

        if(realtime_mode_) {
            std::cerr << "Real time mode enabled" << std::endl;
            input_stream_ = new slambench::io::RealTimeFrameStream(input_stream_, realtime_mult_, true);
        } else {
            std::cerr << "Process every frame mode enabled" << std::endl;
        }

        GetGroundTruth().LoadGTOutputsFromSLAMFile(interface->GetSensors(), gt_buffering_stream->GetGTFrames(), with_point_cloud);
    }

    auto gt_trajectory = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    if(gt_trajectory == nullptr) {
        // Warn if there is no ground truth
        std::cerr << "Dataset does not provide a GT trajectory" << std::endl;
    }



    initialised_ = true;
}


void SLAMBenchConfigurationLifelong::compute_loop_algorithm(SLAMBenchConfiguration* config, bool *remote_stay_on, SLAMBenchUI *ui) {

    auto config_lifelong = dynamic_cast<SLAMBenchConfigurationLifelong*>(config);
    assert(config_lifelong->initialised_);

    for (auto lib : config_lifelong->slam_libs) {

        auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if(trajectory == nullptr) {
            std::cerr << "Algo does not provide a main pose output" << std::endl;
            exit(1);
        }
    }

    // ********* [[ MAIN LOOP ]] *********

    unsigned int frame_count = 0;
    unsigned int bags_count = 0;
    bool ongoing = false;

    slambench::io::InputInterface* interface = config_lifelong->GetCurrentInputInterface();
    while(interface!=nullptr)
    {


        // No UI tool in Lifelong SLAM for now


        // ********* [[ LOAD A NEW FRAME ]] *********

        if(config_lifelong->input_stream_ == nullptr) {
            std::cerr << "No input loaded." << std::endl;
            break;
        }

        slambench::io::SLAMFrame * current_frame = config_lifelong->input_stream_->GetNextFrame();

        while(current_frame!= nullptr)
        {

            // ********* [[ NEW FRAME PROCESSED BY ALGO ]] *********

            for (auto lib : config_lifelong->slam_libs) {


                // ********* [[ SEND THE FRAME ]] *********
                ongoing=not lib->c_sb_update_frame(lib,current_frame);

                // This algorithm hasn't received enough frames yet.
                if(ongoing) {
                    continue;
                }

                // ********* [[ PROCESS ALGO START ]] *********
                lib->GetMetricManager().BeginFrame();



                if (not lib->c_sb_process_once (lib)) {
                    std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
                    exit(1);
                }

                slambench::TimeStamp ts = current_frame->Timestamp;
                if(!lib->c_sb_update_outputs(lib, &ts)) {
                    std::cerr << "Failed to get outputs" << std::endl;
                    exit(1);
                }

                lib->GetMetricManager().EndFrame();

            }



            // ********* [[ FINALIZE ]] *********

            current_frame->FreeData();


            if(!ongoing) {
                config->FireEndOfFrame();
                if (ui) ui->stepFrame();
                frame_count += 1;

                if (config_lifelong->frame_limit) {
                    if (frame_count >= config_lifelong->frame_limit) {
                        break;
                    }
                }
            }
            // we're done with the frame
            current_frame = config_lifelong->input_stream_->GetNextFrame();
        }
        std::cerr << "Last frame in bag processed." << std::endl;
        // Load next bag if it exists
        config_lifelong->LoadNextInputInterface();
        interface = config_lifelong->GetCurrentInputInterface();
        // Mihai: a bit redundant, could be done nicer
        if(interface == nullptr)
        {
            std::cerr << "Last bag processed." << std::endl;
            break;
        }

        for (auto lib : config_lifelong->slam_libs) {
            lib->updateInputInterface(interface);
            current_frame = config_lifelong->input_stream_->GetNextFrame();
            //Mihai: need assertion / safety mechanism to avoid ugly errors
            bool res = dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->c_sb_relocalize(lib, current_frame);

            // Mihai: Might want to add a reset function to feed the pose?
            if(!res)
            {
                auto gt_frame = dynamic_cast<slambench::io::GTBufferingFrameStream*>(config_lifelong->input_stream_)->GetGTFrames()->GetFrame(0);
                lib->c_sb_update_frame(lib,gt_frame);
            }
        }
    }

}



slambench::io::InputInterface * SLAMBenchConfigurationLifelong::GetCurrentInputInterface()
{
    if(input_interfaces.front() == nullptr) {
        throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces.front();
}

const slambench::io::SensorCollection& SLAMBenchConfigurationLifelong::GetSensors()
{
    return this->GetCurrentInputInterface()->GetSensors();
}


void SLAMBenchConfigurationLifelong::AddInputInterface(slambench::io::InputInterface *input_ref) {
    input_interfaces.push_back(input_ref);
}

void SLAMBenchConfigurationLifelong::init_sensors() {

    for (slambench::io::Sensor *sensor : this->GetCurrentInputInterface()->GetSensors()) {
        GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
    }
}

void SLAMBenchConfigurationLifelong::LoadNextInputInterface() {
    input_interfaces.pop_back();
    reset_sensors();
    // TODO: decide if this should be handled differently
    if(input_interfaces.front() == nullptr)
        return;

    init_sensors();
    InitGroundtruth();
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/SensorCollection.h"
#include "io/sensor/Sensor.h"

#include <stdexcept>

using namespace slambench::io;

Sensor &SensorCollection::at(unsigned int sensor_idx) {
	return *_container.at(sensor_idx);
}

size_t SensorCollection::size() const {
	return _container.size();
}

void SensorCollection::AddSensor(Sensor* sensor) {
	if(sensor == nullptr) {
		throw std::logic_error("Cannot add a null sensor to SensorCollection");
	}
	sensor->Index = _container.size();
	_container.push_back(sensor);
}

Sensor* SensorCollection::GetSensor(const Sensor::sensor_type_t & type) {
	for(auto &i : _container) {
		if(i->GetType() == type) {
			return i;
		}
	}
	
	return nullptr;
}

const Sensor* SensorCollection::GetSensor(const Sensor::sensor_type_t & type) const {
	for(auto &i : _container) {
		if(i->GetType() == type) {
			return i;
		}
	}
	
	return nullptr;
}

std::vector<Sensor*> SensorCollection::GetSensors(const Sensor::sensor_type_t & type) {
	std::vector<Sensor*> output;
	for(auto &i : _container) {
		if(i->GetType() == type) {
			output.push_back(i);
		}
	}
	return output;
}

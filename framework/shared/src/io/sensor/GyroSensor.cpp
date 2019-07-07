/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/sensor/GyroSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t GyroSensor::kGyroType = "Gyro";

GyroSensor::GyroSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kGyroType) {

}
size_t GyroSensor::GetFrameSize(const SLAMFrame *frame) const  {
	(void)frame;
	return 3 * sizeof(float);
}

class GyroSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class GyroDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name,const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != GyroSensor::kGyroType) {
			return false;
		}

		*s = new GyroSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration gyro_reg(GyroSensor::kGyroType, slambench::io::SensorDatabaseEntry(new GyroSerialiser(), new GyroDeserialiser(), false, false));



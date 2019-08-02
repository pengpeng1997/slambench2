

#ifndef IO_GYROSENSOR_H
#define IO_GYROSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class GyroSensor : public Sensor {
		public:
			const static sensor_type_t kGyroType;

			float GyroscopeNoiseDensity;
			float GyroscopeDriftNoiseDensity;
			float GyroscopeBiasDiffusion;
			float GyroscopeSaturation;
			
			GyroSensor(const sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_GYROSENSOR_H */
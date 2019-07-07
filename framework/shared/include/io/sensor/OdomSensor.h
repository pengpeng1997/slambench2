

#ifndef IO_ODOMSENSOR_H
#define IO_ODOMSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class OdomSensor : public Sensor {
		public:
			const static sensor_type_t kOdomType;
			
			OdomSensor(const sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_ODOMSENSOR_H */
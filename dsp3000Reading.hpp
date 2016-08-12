/**
 * Author: Matthias Goldhoorn (matthias.goldhoorn@dfki.de)
 * Company: Deutsches Forschungszentrum für Künstliche Intelligenz - Robotics Innovation Center (DFKI RIC)
 * Year 2010
 * Desc:
 *
*/
#ifndef DSP3000READING_H
#define DSP3000READING_H

#include <base/Time.hpp>

namespace fog_kvh 
{
	enum dsp3000Mode{
		RATE,
		INCREMENTAL,
		INTEGRATED
	};
	
	struct dsp3000Config{
		dsp3000Mode mode;
		bool reset;
	};

	struct dsp3000Reading{
		base::Time time;
		double rotation;
		int packedID;
	};

};


#endif 

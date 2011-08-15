/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Dsp3000.hpp"
#include <base/float.h>
#include <aggregator/TimestampEstimator.hpp>

using namespace fog_kvh;

Dsp3000::Dsp3000(std::string const& name)
    : TaskBase(name)
{
	currentMode = RATE;
	ifg=0;
	timestamp_estimator=0;
	id=0;
}


Dsp3000::~Dsp3000(){
    if(ifg) delete ifg;
    if(timestamp_estimator) delete timestamp_estimator;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Dsp3000::configureHook()
{
     activity = getActivity<RTT::extras::FileDescriptorActivity>();
     if (! TaskBase::configureHook())
         return false;

	ifg = new Driver();
        //timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2));
        timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2),base::Time::fromSeconds(0.01),INT_MAX);

	if(!ifg->init(_port.value()))
        {
            fprintf(stderr,"Cannot initialize FOG Driver\n");
            return false;
        }

	currentMode = RATE;
	ifg->toRate();
	ifg->reset();
	
	/** Angular velocity of the FOG gyros use the IMUSensors class **/
	ifgData = new base::samples::IMUSensors;
	
	/** Set to zero the other axis **/
	ifgData->gyro[0] = (double)base::unset<float>();
	ifgData->gyro[1] = (double)base::unset<float>();

	/** Set to zero de other sensors that FOG does not have **/
	ifgData->acc[0] = (double)base::unset<float>();
	ifgData->acc[1] = (double)base::unset<float>();
	ifgData->acc[2] = (double)base::unset<float>();

	ifgData->mag[0] = (double)base::unset<float>();
        ifgData->mag[1] = (double)base::unset<float>();
        ifgData->mag[2] = (double)base::unset<float>();

	return true;
}


bool Dsp3000::startHook()
{
    if (! TaskBase::startHook())
        return false;
    ifg->clear();
    if(activity)
    {
	activity->watch(ifg->getReadFD());
	activity->setTimeout(_timeout.get());
    }
    return true;
}



void Dsp3000::updateHook()
{
    TaskBase::updateHook();
    activity = getActivity<RTT::extras::FileDescriptorActivity>(); 
    if (activity)
    {
        if (activity->hasError() || activity->hasTimeout())
            return exception(IO_ERROR);
    }


    dsp3000Config config;
    while(_config.read(config, false) == RTT::NewData){
    	if(config.reset){
		ifg->reset();
	}
	if(config.mode == RATE){
		ifg->toRate();
	}else if(config.mode == INCREMENTAL){
		ifg->toIncremental();
	}else if(config.mode == ::INTEGRATED){
		ifg->toIntegradted();
	}else{
		fprintf(stderr,"Cannot set unknown mode\n");
		return exception();
	}
	currentMode = config.mode;
    }

    double rotation; /**< double to get values from the FOG driver**/

    /** Read the value from the FOG **/
    ifgData->time = timestamp_estimator->update(base::Time::now());
    if (!ifg->getState(rotation))
        return exception(IO_ERROR);
    
    /** Store the value in the IMUSensors datatype **/
    ifgData->gyro[2] = rotation;
    
    /** write the object in the port **/
    if(currentMode == RATE)
	    _rotation.write(*ifgData);
    //TODO Handling for integrated values to igc message
	

    /** Write the integrated output also in the other port (RigidBodyState) **/
    base::samples::RigidBodyState reading;
    if(currentMode == RATE){
	    static double time=0.010574;
	    sum += ifgData->gyro[2]*time;
	    reading.time = ifgData->time;
	    reading.orientation = Eigen::AngleAxisd(sum, Eigen::Vector3d::Unit(2)); 
	    reading.angular_velocity = Eigen::Vector3d(0,0,ifgData->gyro[2]);
	    //TODO add covariances
	    _orientation_samples.write(reading);
    }else if(currentMode == INTEGRATED){
    		reading.time = ifgData->time;
		reading.orientation = Eigen::AngleAxisd(ifgData->gyro[2], Eigen::Vector3d::Unit(2));
    		_orientation_samples.write(reading);
    }else{
    	fprintf(stderr,"Warning current mode for fog not implemented yet\n");
    }

    
}


// void Dsp3000::errorHook()
// {
//     TaskBase::errorHook();
// }
void Dsp3000::stopHook()
{
    TaskBase::stopHook();
    if(activity)
    {
	activity->clearAllWatches();
    }
}

void Dsp3000::cleanupHook()
{
    TaskBase::cleanupHook();
    if(ifg) delete ifg;
    ifg = 0;
    if(timestamp_estimator) delete timestamp_estimator;
    timestamp_estimator = 0;
    
    delete ifgData;
    ifgData = NULL;
}


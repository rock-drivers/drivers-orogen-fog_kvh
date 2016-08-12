/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Dsp3000Task.hpp"
#include <fog_kvh/dsp3000.h>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <base/Float.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace fog_kvh;

Dsp3000Task::Dsp3000Task(std::string const& name)
    : Dsp3000TaskBase(name)
{
	currentMode = RATE;
	driver=0;
	timestamp_estimator=0;
	id=0;
}


Dsp3000Task::~Dsp3000Task(){
    if(driver) delete driver;
    if(timestamp_estimator) delete timestamp_estimator;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Dsp3000Task::configureHook()
{
     activity = getActivity<RTT::extras::FileDescriptorActivity>();
     if (! Dsp3000TaskBase::configureHook())
         return false;

	driver = new Dsp3000Driver();
        //timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2));
        timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2),base::Time::fromSeconds(0.01),INT_MAX);

	if(!driver->init(_port.value()))
        {
            fprintf(stderr,"Cannot initialize FOG Driver\n");
            return false;
        }

	currentMode = RATE;
	driver->toRate();
	driver->reset();
	
	/** Angular velocity of the FOG gyros use the IMUSensors class **/
	/** Set to zero the other axis **/
	driverData.gyro[0] = (double)base::unset<float>();
	driverData.gyro[1] = (double)base::unset<float>();

	/** Set to zero de other sensors that FOG does not have **/
	driverData.acc[0] = (double)base::unset<float>();
	driverData.acc[1] = (double)base::unset<float>();
	driverData.acc[2] = (double)base::unset<float>();

	driverData.mag[0] = (double)base::unset<float>();
        driverData.mag[1] = (double)base::unset<float>();
        driverData.mag[2] = (double)base::unset<float>();

	return true;
}


bool Dsp3000Task::startHook()
{
    if (! Dsp3000TaskBase::startHook())
        return false;
    driver->clear();
    if(activity)
    {
	activity->watch(driver->getReadFD());
	activity->setTimeout(_timeout.get());
    }
    timestamp_estimator->reset();
    return true;
}



void Dsp3000Task::updateHook()
{
    Dsp3000TaskBase::updateHook();
    activity = getActivity<RTT::extras::FileDescriptorActivity>(); 
    if (activity)
    {
        if (activity->hasError() || activity->hasTimeout())
            return exception(IO_ERROR);
    }


    dsp3000Config config;
    while(_config.read(config, false) == RTT::NewData){
    	if(config.reset){
		driver->reset();
	}
	if(config.mode == RATE){
		driver->toRate();
	}else if(config.mode == INCREMENTAL){
		driver->toIncremental();
	}else if(config.mode == ::INTEGRATED){
		driver->toIntegradted();
	}else{
		fprintf(stderr,"Cannot set unknown mode\n");
		return exception();
	}
	currentMode = config.mode;
    }

    double rotation; /**< double to get values from the FOG driver**/

    /** Read the value from the FOG **/
    driverData.time = timestamp_estimator->update(base::Time::now());
    if (!driver->getState(rotation))
        return exception(IO_ERROR);
    
    /** Store the value in the IMUSensors datatype **/
    driverData.gyro[2] = rotation;
    
    /** write the object in the port **/
    if(currentMode == RATE)
	    _rotation.write(driverData);
    //TODO Handling for integrated values to igc message
	

    /** Write the integrated output also in the other port (RigidBodyState) **/
    base::samples::RigidBodyState reading;
    if(currentMode == RATE){
	    static double time=0.010574;
	    sum += driverData.gyro[2]*time;
	    reading.time = driverData.time;
	    reading.orientation = Eigen::AngleAxisd(sum, Eigen::Vector3d::Unit(2)); 
	    reading.angular_velocity = Eigen::Vector3d(0,0,driverData.gyro[2]);
	    //TODO add covariances
	    _orientation_samples.write(reading);
    }else if(currentMode == INTEGRATED){
    		reading.time = driverData.time;
		reading.orientation = Eigen::AngleAxisd(driverData.gyro[2], Eigen::Vector3d::Unit(2));
    		_orientation_samples.write(reading);
    }else{
    	fprintf(stderr,"Warning current mode for fog not implemented yet\n");
    }

    _timestamp_estimator_status.write(timestamp_estimator->getStatus());
    
}


// void Dsp3000Task::errorHook()
// {
//     Dsp3000TaskBase::errorHook();
// }
void Dsp3000Task::stopHook()
{
    Dsp3000TaskBase::stopHook();
    if(activity)
    {
	activity->clearAllWatches();
    }
}

void Dsp3000Task::cleanupHook()
{
    Dsp3000TaskBase::cleanupHook();
    if(driver) delete driver;
    driver = 0;
    if(timestamp_estimator) delete timestamp_estimator;
    timestamp_estimator = 0;
}


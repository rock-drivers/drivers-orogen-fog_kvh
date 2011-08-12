/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <TimestampEstimator.hpp>

using namespace dsp3000;

Task::Task(std::string const& name)
    : TaskBase(name)
{
	currentMode = sensorData::RATE;
	ifg=0;
	timestamp_estimator=0;
	id=0;
}


Task::~Task(){
    if(ifg) delete ifg;
    if(timestamp_estimator) delete timestamp_estimator;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
     activity = getActivity<RTT::extras::FileDescriptorActivity>();
     if (! TaskBase::configureHook())
         return false;

	ifg = new Driver();
        //timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2));
        timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2),base::Time::fromSeconds(0.01),INT_MAX);

	if(!ifg->init(_port.value()))
        {
            fprintf(stderr,"Cannot initialize iFG Driver\n");
            return false;
        }

	currentMode = sensorData::RATE;
	ifg->toRate();
	ifg->reset();
	
	/** Angular velocity of the FOG gyros use the IMUSensors class **/
	ifgData = new base::samples::IMUSensors;
	
	/** Set to zero the other axis **/
	ifgData->gyro[0] = NaN;
	ifgData->gyro[1] = NaN;

	/** Set to zero de other sensors that FOG does not have **/
	ifgData->acc[0] = NaN;
	ifgData->acc[1] = NaN;
	ifgData->acc[2] = NaN;

	ifgData->mag[0] = 0.00;
        ifgData->mag[1] = 0.00;
        ifgData->mag[2] = 0.00;

	return true;
}


bool Task::startHook()
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



void Task::updateHook()
{
    TaskBase::updateHook();
    activity = getActivity<RTT::extras::FileDescriptorActivity>(); 
    if (activity)
    {
        if (activity->hasError() || activity->hasTimeout())
            return exception(IO_ERROR);
    }


    sensorData::dsp3000Config config;
    while(_config.read(config, false) == RTT::NewData){
    	if(config.reset){
		ifg->reset();
	}
	if(config.mode == sensorData::RATE){
		ifg->toRate();
	}else if(config.mode == sensorData::INCREMENTAL){
		ifg->toIncremental();
	}else if(config.mode == sensorData::INTEGRATED){
		ifg->toIntegradted();
	}else{
		fprintf(stderr,"Cannot set unknown mode\n");
		return exception();
	}
	currentMode = config.mode;
    }

    double rotation;
    //sensorData::dsp3000Reading ifgData;
    ifgData->time = timestamp_estimator->update(base::Time::now());
    if (!ifg->getState(rotation))
        return exception(IO_ERROR);
    
    ifgData->gyro[2] = rotation;
    
    if(currentMode == sensorData::RATE)
	    _rotation.write(*ifgData);
    //TODO Handling for integrated values to igc message
	

	
    base::samples::RigidBodyState reading;
    if(currentMode == sensorData::RATE){
	    static double time=0.010574;
	    sum += ifgData->gyro[2]*time;
	    reading.time = ifgData->time;
	    reading.orientation = Eigen::AngleAxisd(sum, Eigen::Vector3d::Unit(2)); 
	    reading.angular_velocity = Eigen::Vector3d(0,0,ifgData->gyro[2]);
	    //TODO add covariances
	    _orientation_samples.write(reading);
    }else if(currentMode == sensorData::INTEGRATED){
    		reading.time = ifgData->time;
		reading.orientation = Eigen::AngleAxisd(ifgData->gyro[2], Eigen::Vector3d::Unit(2));
    		_orientation_samples.write(reading);
    }else{
    	fprintf(stderr,"Warning current mode for fog not implemented yet\n");
    }

    
}


// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
void Task::stopHook()
{
    TaskBase::stopHook();
    if(activity)
    {
	activity->clearAllWatches();
    }
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    if(ifg) delete ifg;
    ifg = 0;
    if(timestamp_estimator) delete timestamp_estimator;
    timestamp_estimator = 0;
    
    delete ifgData;
    ifgData = NULL;
}


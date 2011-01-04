/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <TimestampEstimator.hpp>

using namespace dsp3000;

Task::Task(std::string const& name)
    : TaskBase(name)
{
	currentMode = sensorData::RATE;
}


Task::~Task(){
    delete ifg;
    delete timestamp_estimator;
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
        timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(2));

	if(!ifg->init(_port.value()))
        {
            fprintf(stderr,"Cannot initialize iFG Driver\n");
            return false;
        }

        if(activity)
        {
            activity->watch(ifg->getReadFD());
            activity->setTimeout(_timeout.get());
        }
	currentMode = sensorData::RATE;
	ifg->toRate();
	ifg->reset();
	return true;
}


bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    ifg->clear();
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
    if(_config.read(config) == RTT::NewData){
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

    sensorData::dsp3000Reading ifgData;
    ifgData.time = timestamp_estimator->update(base::Time::now());
    if (!ifg->getState(ifgData.rotation))
        return exception(IO_ERROR);

    if(currentMode == sensorData::RATE)
	    _rotation.write(ifgData);
    //TODO Handling for integrated values to igc message

    base::samples::RigidBodyState reading;
    if(currentMode == sensorData::RATE){
	    static double time=0.010574;
	    sum += ifgData.rotation*time;
	    reading.time = ifgData.time;
	    reading.orientation = Eigen::AngleAxisd(sum, Eigen::Vector3d::Unit(2)); 
	    reading.angular_velocity = Eigen::Vector3d(0,0,ifgData.rotation);
	    //TODO add covariances

    }else if(currentMode == sensorData::INTEGRATED){
    		reading.time = ifgData.time;
		reading.orientation = Eigen::AngleAxisd(ifgData.rotation, Eigen::Vector3d::Unit(2));

    }else{
    	fprintf(stderr,"Warning current mode for fog not implemented yet\n");
    }
}


// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete ifg;
    ifg = 0;
    delete timestamp_estimator;
    timestamp_estimator = 0;
}


/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "StereoOdometer.hpp"

using namespace viso2;

StereoOdometer::StereoOdometer(std::string const& name)
    : StereoOdometerBase(name)
{
}

StereoOdometer::StereoOdometer(std::string const& name, RTT::ExecutionEngine* engine)
    : StereoOdometerBase(name, engine)
{
}

StereoOdometer::~StereoOdometer()
{
}

void StereoOdometer::left_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    left_image = left_frame_sample;

    /* Set to true the new image **/
    flag.leftFrameSamples = true;

    return;
}

void StereoOdometer::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    const uint8_t *l_image_data, *r_image_data;

    right_image = static_cast< ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > >(right_frame_sample);

    if (flag.leftFrameSamples)
    {
        flag.rightFrameSamples = true;

        /** Get the images to plain pointers **/
        l_image_data = reinterpret_cast < const uint8_t* >(&left_image->image);
        r_image_data = reinterpret_cast < const uint8_t* >(&right_image->image);
        int32_t dims[] = {left_image->size.width, left_image->size.height, left_image->size.width};

        viso->process (l_image_data, r_image_data, dims);



        flag.reset();
    }
    else
    {
        flag.rightFrameSamples = true;
    }



    
    return;
}
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See StereoOdometer.hpp for more detailed
// documentation about them.

bool StereoOdometer::configureHook()
{
    if (! StereoOdometerBase::configureHook())
        return false;

    /** Read the parameter configuration values **/
    param.base = _viso2_parameters.value().base;
    param.ransac_iters = _viso2_parameters.value().ransac_iters;
    param.inlier_threshold = _viso2_parameters.value().inlier_threshold;
    param.reweighting = _viso2_parameters.value().reweighting;
    param.match = _viso2_parameters.value().match;
    param.bucket = _viso2_parameters.value().bucket;
    param.calib = _viso2_parameters.value().calib;

    /** Initialize variables **/

    viso.reset(new VisualOdometryStereo(param));

    left_image = NULL;
    right_image = NULL;

    flag.reset();

    return true;
}
bool StereoOdometer::startHook()
{
    if (! StereoOdometerBase::startHook())
        return false;
    return true;
}
void StereoOdometer::updateHook()
{
    StereoOdometerBase::updateHook();
}
void StereoOdometer::errorHook()
{
    StereoOdometerBase::errorHook();
}
void StereoOdometer::stopHook()
{
    StereoOdometerBase::stopHook();
}
void StereoOdometer::cleanupHook()
{
    StereoOdometerBase::cleanupHook();
}

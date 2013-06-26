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
    imagePair[0].time = left_frame_sample->time;
    imagePair[0].left = *left_frame_sample;

    RTT::log(RTT::Warning) << "[LeftFrameCallback] Frame arrived at: " <<left_frame_sample->time<< RTT::endlog();

    /* Set to true the new image **/
    flag.leftFrameSamples = true;

    return;
}

void StereoOdometer::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    uint8_t *l_image_data, *r_image_data;

    imagePair[0].right = *right_frame_sample;

    RTT::log(RTT::Warning) << "[RightFrameCallback] Frame arrived at: " <<right_frame_sample->time<< RTT::endlog();


    if (flag.leftFrameSamples)
    {
        base::Time frameDelta_t = imagePair[0].right.time-imagePair[0].left.time;
        flag.rightFrameSamples = true;
        RTT::log(RTT::Warning) << "[StereOdometer] SyncroDiff: " <<frameDelta_t.toSeconds() << RTT::endlog();

        /** Get the images to plain pointers **/
        l_image_data = imagePair[0].left.getImagePtr();
        r_image_data = imagePair[0].right.getImagePtr();
        int32_t dims[] = {imagePair[0].left.size.width, imagePair[0].left.size.height, imagePair[0].left.size.width};

        if (viso->process (l_image_data, r_image_data, dims) )
        {
            /** on success, update current pose **/
            pose = pose * Matrix::inv(viso->getMotion());

            /** output some statistics **/
            double num_matches = viso->getNumberOfMatches();
            double num_inliers = viso->getNumberOfInliers();
            std::cout << ", Matches: " << num_matches;
            std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
            std::cout << pose << std::endl << std::endl;

            /** Map the viso2 data to an Eigen matrix **/
            register int k = 0;
            double mdata[16];
            base::Matrix4d poseM;
            for (register int  i=0; i<4; i++)
                for (register int j=0; j<4; j++)
                        mdata[k++] = pose.val[j][i];

            poseM = Eigen::Map<const base::Matrix4d> (&(mdata[0]), 4, 4);
            std::cout << poseM << std::endl << std::endl;

            /** Store in RigidBodyState to port out the pose **/
            Eigen::Quaternion<double> attitude(poseM.block<3,3> (0,0));
            poseOut.time = imagePair[0].left.time;
            poseOut.orientation = attitude;
            poseOut.position = poseM.col(3).block<3,1>(0,0);
            std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
            std::cout << "Position:\n"<< poseOut.position<<"\n";

            _pose_samples_out.write(poseOut);

            /** Debug port to LeftRight correspondence **/
            ::base::samples::frame::Frame *frame_ptr = intraFrame_out.write_access();

            frame_ptr->init(2*imagePair[0].left.getWidth(), imagePair[0].left.getHeight(),
                                                    imagePair[0].left.getDataDepth(), imagePair[0].left.getFrameMode(),
                                                    -1, 2*imagePair[0].left.getNumberOfBytes());

            frame_ptr->time = imagePair[0].left.time;
            frame_ptr->received_time = imagePair[0].left.received_time;
            frame_ptr->attributes = imagePair[0].left.attributes;

            std::cout<<"rightFrame size:"<<imagePair[0].right.image.size()<<"\n";
            std::cout<<"leftFrame size:"<<imagePair[0].left.image.size()<<"\n";
            std::cout<<"intraFrame size:"<<frame_ptr->image.size()<<"\n";
            std::vector<uint8_t> intraimage (frame_ptr->image.size(), 0);
            std::cout<<"intraimage.size(): "<<intraimage.size()<<"\n";

            //frame_ptr->image = imagePair[0].left.image;
            //memcpy(&frame_ptr->image[0], l_image_data, imagePair[0].left.image.size());
            memcpy(&frame_ptr->image[0], &intraimage[0], intraimage.size());
            //memcpy (reinterpret_cast<char*>(&intraimage[0]), reinterpret_cast<char*>(&imagePair[0].left.image[0]), imagePair[0].left.image.size());
            //memcpy (reinterpret_cast<char*>(&intraimage[imagePair[0].left.image.size()]), reinterpret_cast<char*>(&imagePair[0].right.image[0]), imagePair[0].right.image.size());
            std::cout<<"intraimage.size(): "<<intraimage.size()<<"\n";

            //frame_ptr->setImage(reinterpret_cast<char*>(&intraimage), frame_ptr->image.size());

            //register int l = 0;
            //for (std::vector<uint8_t>::iterator it = imagePair[0].left.image.begin() ; it != imagePair[0].left.image.end(); ++it)
            //{
            //    frame_ptr->image[l] = *it;
            //    frame_ptr->image[l+imagePair[0].left.image.size()] = imagePair[0].right.image[l];
            //    l++;
            //}

            intraFrame_out.reset(frame_ptr);
            _intra_frame_samples_out.write(intraFrame_out);

        }
        else
        {
            RTT::log(RTT::Warning) << "Viso2 Stereo Odometer failed!!" << RTT::endlog();
        }


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

    pose = Matrix::eye(4);
    poseOut.invalidate();
    poseOut.sourceFrame = "CameraFrame (t)";
    poseOut.targetFrame = "CameraFrame (0)";

    flag.reset();

    StereoPair pair;
    imagePair.set_capacity(1);
    imagePair.push_front(pair);

    ::base::samples::frame::Frame *intraFrame = new ::base::samples::frame::Frame();

    intraFrame_out.reset(intraFrame);
    intraFrame = NULL;

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

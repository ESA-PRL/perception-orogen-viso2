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
    imagePair[0].time = left_frame_sample->time; //time of the pair

    /** Correct distorsion in images left **/
    frameHelper.setCalibrationParameter(cameracalib.camLeft);

    /** The image need to be in gray scale and undistort **/
    //imagePair[0].left.init(*left_frame_sample, false);
    imagePair[0].left.frame_mode = base::samples::frame::MODE_RGB;
    imagePair[0].left.setDataDepth(left_frame_sample->getDataDepth());
    frameHelper.convert (*left_frame_sample, imagePair[0].left, 0, 0, frame_helper::INTER_LINEAR, true);
    //frameHelper.undistort (*left_frame_sample, imagePair[0].left);

    //imagePair[0].left = *left_frame_sample;

    RTT::log(RTT::Warning) << "[LeftFrameCallback] Frame arrived at: " <<left_frame_sample->time<< RTT::endlog();

    /* Set to true the new image **/
    flag.leftFrameSamples = true;

    return;
}

void StereoOdometer::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    uint8_t *l_image_data, *r_image_data;

    //imagePair[0].right = *right_frame_sample;

    RTT::log(RTT::Warning) << "[RightFrameCallback] Frame arrived at: " <<right_frame_sample->time<< RTT::endlog();


    if (flag.leftFrameSamples)
    {
        flag.rightFrameSamples = true;

        /** Correct distorsion in image right **/
        frameHelper.setCalibrationParameter(cameracalib.camRight);
        //imagePair[0].right.init (*right_frame_sample, false);
        imagePair[0].right.frame_mode = base::samples::frame::MODE_RGB;
        imagePair[0].right.setDataDepth(right_frame_sample->getDataDepth());
        frameHelper.convert (*right_frame_sample, imagePair[0].right, 0, 0, frame_helper::INTER_LINEAR, true);
        //frameHelper.undistort (*right_frame_sample, imagePair[0].right);

        base::Time frameDelta_t = imagePair[0].right.time-imagePair[0].left.time;
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

            /** Draw matches in the images usin the FrameHelper which internally uses openCV **/
            ::base::samples::frame::Frame *frame_ptr = intraFrame_out.write_access();

            this->drawMatches (imagePair[0].left, imagePair[0].right, viso->getMatches(), viso->getInlierIndices(), *frame_ptr);

            frame_ptr->time = imagePair[0].left.time;
            intraFrame_out.reset(frame_ptr);
            std::cout<<"frame_ptr: "<<frame_ptr<<"\n";
            std::cout<<"frame size: "<<frame_ptr->size.width*frame_ptr->size.height<<"\n";
            _intra_frame_samples_out.write(intraFrame_out);

            /** Create distance image **/
            this->createDistanceImage(imagePair[0].left, imagePair[0].right, viso->getMatches(), viso2param, distanceFrame_out);
            _distance_frame_samples_out.write(distanceFrame_out);
            _left_frame_samples_out.write(imagePair[0].left);

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

    /** Read the camera calib parameters **/
    cameracalib = _calib_parameters.value();

    /** Read the dedicated viso2  parameter configuration values **/
    viso2param.ransac_iters = _viso2_parameters.value().ransac_iters;
    viso2param.inlier_threshold = _viso2_parameters.value().inlier_threshold;
    viso2param.reweighting = _viso2_parameters.value().reweighting;
    viso2param.match = _viso2_parameters.value().match;
    viso2param.bucket = _viso2_parameters.value().bucket;

    /** Set the calib parameter in the viso2 type **/
    viso2param.base = cameracalib.extrinsic.tx/1000.00; //baseline in viso2 is in meters and comes in mm
    viso2param.calib.f = 0.5*(cameracalib.camLeft.fx + cameracalib.camRight.fx);
    viso2param.calib.cu = 0.5*(cameracalib.camLeft.cx + cameracalib.camRight.cx);
    viso2param.calib.cv = 0.5*(cameracalib.camLeft.cy + cameracalib.camRight.cy);

    /** Initialize variables **/
    viso.reset(new VisualOdometryStereo(viso2param));

    /** Viso2 Matrix class initialization **/
    pose = Matrix::eye(4);

    /** Ribid body state output initialization **/
    poseOut.invalidate();
    poseOut.sourceFrame = "CameraFrame_T";
    poseOut.targetFrame = "CameraFrame_0";

    /** Initialize flag **/
    flag.reset();

    /** Stereo working pair **/
    StereoPair pair;
    imagePair.set_capacity(1);
    imagePair.push_front(pair);

    ::base::samples::frame::Frame *intraFrame = new ::base::samples::frame::Frame();

    intraFrame_out.reset(intraFrame);
    intraFrame = NULL;

    /** Distance Image **/
    const size_t
	width = _calib_parameters.value().camLeft.width,
	height = _calib_parameters.value().camRight.height;

    // pre-allocate the memory for the output disparity map, so we don't
    // have to copy it. This means we have to assert that the width and
    // height is the same for input and resulting disparity images
    distanceFrame_out.setSize(width, height);
    distanceFrame_out.setIntrinsic(_calib_parameters.value().camLeft.fx,
                                   _calib_parameters.value().camLeft.fy,
                                   _calib_parameters.value().camLeft.cx,
                                   _calib_parameters.value().camLeft.cy);

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
void StereoOdometer::drawMatches(const base::samples::frame::Frame &image1,
                                    const base::samples::frame::Frame &image2,
                                    const std::vector<Matcher::p_match> &matches,
                                    const std::vector<int32_t>& inlier_indices,
                                    base::samples::frame::Frame &imageMatches)
{
    ::cv::Mat cvImage1 = frameHelper.convertToCvMat(image1);
    ::cv::Mat cvImage2 = frameHelper.convertToCvMat(image2);
    ::cv::Mat cvImgMatches;
    std::vector< cv::KeyPoint > keypoints1, keypoints2;
    std::vector< cv::DMatch > cvMatches;
    keypoints1.resize(inlier_indices.size());
    keypoints2.resize(inlier_indices.size());
    cvMatches.resize(inlier_indices.size());

    //std::cout<<"[drawMatches] number of features: "<<inlier_indices.size()<<"\n";

    for (size_t i = 0; i < inlier_indices.size(); ++i)
    {
        const Matcher::p_match& match = matches[inlier_indices[i]];
       // std::cout<<"[drawMatches] match1 "<< match.u1c << " " << match.v1c<<"\n";
       // std::cout<<"[drawMatches] match2 "<< match.u2c << " " << match.v2c<<"\n";
        cv::KeyPoint k1 (static_cast<float>(match.u1c), static_cast<float>(match.v1c), 1);
        cv::KeyPoint k2 (static_cast<float>(match.u2c), static_cast<float>(match.v2c), 1);
        float disparity = static_cast<float>(match.u1c) - static_cast<float>(match.u2c);
        cv::DMatch cvMatch (i, i, disparity);
        keypoints1[i] = k1;
        keypoints2[i] = k2;
        cvMatches[i] = cvMatch;
    }

    cv::drawMatches (cvImage1, keypoints1, cvImage2, keypoints2, cvMatches, cvImgMatches);
    frameHelper.copyMatToFrame(cvImgMatches, imageMatches);

    return;
}

void StereoOdometer::createDistanceImage(const base::samples::frame::Frame &image1,
                        const base::samples::frame::Frame &image2,
                        const std::vector<Matcher::p_match> &matches,
                        const VisualOdometryStereo::parameters &viso2param,
                        base::samples::DistanceImage &distImage)
{

    register int i;

    /** Firs set all the distance z values to NaN **/
    std::fill(distImage.data.begin(), distImage.data.end(), base::unknown < base::samples::DistanceImage::scalar >());
    for (std::vector<Matcher::p_match>::const_iterator it = matches.begin() ; it != matches.end(); ++it)
    {
        //std::cout<<"Index: "<<distImage.width*(it->v1c)+(it->u1c)<<"\n";
        //std::cout<<"v1c: "<<it->v1c<<"u1c: "<< it->u1c <<"disparity: "<<it->u1c - it->u2c<<"\n";
        if ((it->v1c < distImage.height) && (it->u1c < distImage.width))
            distImage.data[distImage.width*(it->v1c)+(it->u1c)] = viso2param.base*viso2param.calib.f/(it->u1c - it->u2c);

    }
}

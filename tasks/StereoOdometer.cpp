/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "StereoOdometer.hpp"

#define DEBUG_PRINTS 1

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
    imagePair[0].first.time = left_frame_sample->time; //time of the left image


    RTT::log(RTT::Warning) << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time<< RTT::endlog();

    /** The image need to be in gray scale and undistorted **/
    imagePair[0].first.frame_mode = base::samples::frame::MODE_GRAYSCALE;
    imagePair[0].first.setDataDepth(left_frame_sample->getDataDepth());
    frameHelperLeft.convert (*left_frame_sample, imagePair[0].first, 0, 0, frame_helper::INTER_LINEAR, true);

    /** Left color image **/
    leftColorImage.frame_mode = base::samples::frame::MODE_RGB;
    leftColorImage.setDataDepth(left_frame_sample->getDataDepth());
    frameHelperLeft.convert (*left_frame_sample, leftColorImage, 0, 0, frame_helper::INTER_LINEAR, true);

    /** Check the time difference between inertial sensors and joint samples **/
    base::Time diffTime = imagePair[0].first.time - imagePair[0].second.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_left_frame_period/2.0))
    {
        imagePair[0].time = imagePair[0].first.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISO2 LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->computeStereoOdometer();

    }

    return;
}

void StereoOdometer::right_frameCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{

    imagePair[0].second.time = right_frame_sample->time; //time stamp for the right image

    RTT::log(RTT::Warning) << "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time<< RTT::endlog();


    /** Correct distortion in image right **/
    imagePair[0].second.frame_mode = base::samples::frame::MODE_GRAYSCALE;
    imagePair[0].second.setDataDepth(right_frame_sample->getDataDepth());
    frameHelperRight.convert (*right_frame_sample, imagePair[0].second, 0, 0, frame_helper::INTER_LINEAR, true);


    /** Check the time difference between inertial sensors and joint samples **/
    base::Time diffTime = imagePair[0].second.time - imagePair[0].first.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_right_frame_period/2.0))
    {
        imagePair[0].time = imagePair[0].second.time;

        #ifdef DEBUG_PRINTS
        std::cout<< "[VISO2 RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
        #endif

        this->computeStereoOdometer();

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

    /** Read the camera calibration parameters **/
    cameracalib = _calib_parameters.value();

    /** Read the dedicated viso2  parameter configuration values **/
    viso2param.ransac_iters = _viso2_parameters.value().ransac_iters;
    viso2param.inlier_threshold = _viso2_parameters.value().inlier_threshold;
    viso2param.reweighting = _viso2_parameters.value().reweighting;
    viso2param.match = _viso2_parameters.value().match;
    viso2param.bucket = _viso2_parameters.value().bucket;

    /** Set the calibration parameters in the viso2 type **/
    viso2param.base = cameracalib.extrinsic.tx/1000.00; //baseline in viso2 is in meters and comes in mm
    viso2param.calib.f = 0.5*(cameracalib.camLeft.fx + cameracalib.camRight.fx);
    viso2param.calib.cu = 0.5*(cameracalib.camLeft.cx + cameracalib.camRight.cx);
    viso2param.calib.cv = 0.5*(cameracalib.camLeft.cy + cameracalib.camRight.cy);

    /** Re-projection Q matrix **/
    Q = Eigen::Matrix4d::Zero();
    Q(0,0) = Q(1,1) = 1.0;
    Q(3,2) = 1.0/viso2param.base;
    Q(0,3) = -cameracalib.camRight.cx;
    Q(1,3) = -cameracalib.camRight.cy;
    Q(2,3) = cameracalib.camRight.fx;
    Q(3,3) = (cameracalib.camRight.cx - cameracalib.camLeft.cx)/viso2param.base;

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 CONFIGURATION] Q re-projection matrix:\n "<<Q<<"\n";
    #endif


    /** Initialize variables **/
    viso.reset(new VisualOdometryStereo(viso2param));

    /** Frame Helper **/
    frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Viso2 Matrix class initialization **/
    pose = Matrix::eye(4);

    /** Ribid body state output initialization **/
    poseOut.invalidate();
    poseOut.sourceFrame = "CameraFrame_T";
    poseOut.targetFrame = "CameraFrame_0";

    /** Stereo working pair **/
    ::base::samples::frame::FramePair pair;
    imagePair.set_capacity(DEFAULT_CIRCULAR_BUFFER_SIZE);
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

void StereoOdometer::computeStereoOdometer()
{
    uint8_t *l_image_data, *r_image_data;

    /** Get the images to plain pointers **/
    l_image_data = imagePair[0].first.getImagePtr();
    r_image_data = imagePair[0].second.getImagePtr();
    int32_t dims[] = {imagePair[0].first.size.width, imagePair[0].first.size.height, imagePair[0].first.size.width};


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
        poseOut.time = imagePair[0].time;
        poseOut.orientation = attitude;
        poseOut.position = poseM.col(3).block<3,1>(0,0);
        std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
        std::cout << "Position:\n"<< poseOut.position<<"\n";

        _pose_samples_out.write(poseOut);

        /** Draw matches in the images using the FrameHelper which internally uses openCV **/
        ::base::samples::frame::Frame *frame_ptr = intraFrame_out.write_access();
        this->drawMatches (imagePair[0].first, imagePair[0].second, viso->getMatches(), viso->getInlierIndices(), *frame_ptr);

        frame_ptr->time = imagePair[0].time;
        intraFrame_out.reset(frame_ptr);
        std::cout<<"frame_ptr: "<<frame_ptr<<"\n";
        std::cout<<"frame size: "<<frame_ptr->size.width*frame_ptr->size.height<<"\n";
        _intra_frame_samples_out.write(intraFrame_out);

        /** Create the point cloud **/
        this->createPointCloud(leftColorImage, viso->getMatches(), viso->getInlierIndices(), Q, pointcloud_out);
        _point_cloud_samples_out.write(pointcloud_out);

    }
    else
    {
        RTT::log(RTT::Warning) << "[VISO2] Stereo Odometer failed!!" << RTT::endlog();
    }


    return;
}
void StereoOdometer::drawMatches(const base::samples::frame::Frame &image1,
                                    const base::samples::frame::Frame &image2,
                                    const std::vector<Matcher::p_match> &matches,
                                    const std::vector<int32_t>& inlier_indices,
                                    base::samples::frame::Frame &imageMatches)
{
    ::cv::Mat cvImage1 = frameHelperLeft.convertToCvMat(image1);
    ::cv::Mat cvImage2 = frameHelperRight.convertToCvMat(image2);
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
    frameHelperLeft.copyMatToFrame(cvImgMatches, imageMatches);

    return;
}

void StereoOdometer::createDistanceImage(const base::samples::frame::Frame &image1,
                        const base::samples::frame::Frame &image2,
                        const std::vector<Matcher::p_match> &matches,
                        const VisualOdometryStereo::parameters &viso2param,
                        base::samples::DistanceImage &distImage)
{

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

void StereoOdometer::createPointCloud(const base::samples::frame::Frame &image1,
                        const std::vector<Matcher::p_match> &matches,
                        const std::vector<int32_t>& inlier_indices,
                        const Eigen::Matrix4d &Q, 
                        ::base::samples::Pointcloud &pointcloud)
{
    cv::Mat cv_image1 = frame_helper::FrameHelper::convertToCvMat(image1);
    pointcloud.points.resize(inlier_indices.size());
    pointcloud.colors.resize(inlier_indices.size());

    for (register size_t i = 0; i < inlier_indices.size(); ++i)
    {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        ::base::Vector3d point (match.u1c + Q(0,3),  match.v1c + Q(1,3), Q(2,3));
        double disparity = match.u1c - match.u2c;
        double W  = Q(3,2)*disparity + Q(3,3);
        point = point * (1.0/W);
        pointcloud.points[i] = point;
        cv::Vec3b color = cv_image1.at<cv::Vec3b>(match.v1c, match.u1c);
        :: base::Vector4d color4d;
        color4d[0] = color[0];//R
        color4d[1] = color[1];//G
        color4d[2] = color[2];//B
        color4d[3] = 1.0;//Alpha
        pointcloud.colors[i] = color4d;
    }

}


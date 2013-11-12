/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "StereoOdometer.hpp"

#define DEBUG_PRINTS 1

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

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

    //RTT::log(RTT::Warning) << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toMicroseconds()<< RTT::endlog();
    std::cout << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toMicroseconds()<< std::endl;

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

    //RTT::log(RTT::Warning) << "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toMicroseconds()<< RTT::endlog();
    std::cout<< "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toMicroseconds()<<std::endl;

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

    /** Image plane uncertainty(variance) in pixel **/
    pxleftVar << pow(cameracalib.camLeft.pixel_error[0],2), 0.00,
              0.00, pow(cameracalib.camLeft.pixel_error[1]+viso2param.match.match_disp_tolerance, 2);

    pxrightVar << pow(cameracalib.camRight.pixel_error[0],2), 0.00,
              0.00, pow(cameracalib.camRight.pixel_error[1]+viso2param.match.match_disp_tolerance, 2);

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 CONFIGURATION] Left Frame Error matrix:\n "<<pxleftVar<<"\n";
    std::cout<< "[VISO2 CONFIGURATION] Right Frame Error matrix:\n "<<pxrightVar<<"\n";
    #endif

    /** Initialize variables **/
    viso.reset(new VisualOdometryStereo(viso2param));

    /** Frame Helper **/
    frameHelperLeft.setCalibrationParameter(cameracalib.camLeft);
    frameHelperRight.setCalibrationParameter(cameracalib.camRight);

    /** Initial pose initialization **/
    pose = Eigen::Affine3d::Identity();

    /** Rigid body state output initialization **/
    poseOut.invalidate();
    poseOut.sourceFrame = "CameraFrame_T";
    poseOut.targetFrame = "CameraFrame_0";
    poseOut.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    poseOut.position = Eigen::Vector3d::Zero();
    poseOut.velocity = Eigen::Vector3d::Zero();

    /** Stereo working pair **/
    ::base::samples::frame::FramePair pair;
    imagePair.set_capacity(DEFAULT_CIRCULAR_BUFFER_SIZE);
    imagePair.push_front(pair);

    ::base::samples::frame::Frame *intraFrame = new ::base::samples::frame::Frame();

    intraFrame_out.reset(intraFrame);
    intraFrame = NULL;

    /** Hash Table of indexes **/
    hashIdx.set_capacity(DEFAULT_CIRCULAR_BUFFER_SIZE);
    hashPointcloudPrev.clear();
    hashPointcloudCurr.clear();

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

    /** Delta pose **/
    base::samples::RigidBodyState deltaPoseOut;
    Eigen::Affine3d deltaPose;
    deltaPoseOut.invalidate();

    /** Get the images to plain pointers **/
    l_image_data = imagePair[0].first.getImagePtr();
    r_image_data = imagePair[0].second.getImagePtr();
    int32_t dims[] = {imagePair[0].first.size.width, imagePair[0].first.size.height, imagePair[0].first.size.width};

    /** Compute visual odometry **/
    bool success = viso->process (l_image_data, r_image_data, dims);

    /** On success get the delta transform and update the pose **/
    if (success)
    {
        Matrix deltaPoseViso2 =  Matrix::inv(viso->getMotion());

        /** Map the viso2 data to an Eigen matrix **/
        register int k = 0;
        double mdata[16];
        Eigen::Matrix4d deltaM;
        for (register int  i=0; i<4; i++)
            for (register int j=0; j<4; j++)
                    mdata[k++] = deltaPoseViso2.val[j][i];
        deltaM = Eigen::Map<const Eigen::Matrix4d> (&(mdata[0]), 4, 4);
        deltaPose.matrix() = deltaM;

        /** On success, update current pose **/
        pose = pose * deltaPose;

        /** Output some statistics **/
        double num_matches = viso->getNumberOfMatches();
        double num_inliers = viso->getNumberOfInliers();
        #ifdef DEBUG_PRINTS
        std::cout << ", Matches: " << num_matches;
        std::cout << ", Inliers: " << num_inliers;
        std::cout << ", Inliers Ratio: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
        std::cout << pose.matrix() << std::endl << std::endl;
        #endif

        /** Store in RigidBodyState to port out the pose **/
        Eigen::Quaternion<double> attitude(pose.rotation());
        poseOut.time = imagePair[0].time;
        poseOut.orientation = attitude;
        poseOut.position = pose.translation();
        #ifdef DEBUG_PRINTS
        std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
        Eigen::Vector3d euler; /** In Euler angles **/
        euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
        euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
        euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
        std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
        std::cout << "Position:\n"<< poseOut.position<<"\n";
        #endif

        /** Fill the delta pose out **/
        Eigen::Quaternion<double> deltaAttitude(deltaPose.rotation());
        deltaPoseOut.time = imagePair[0].time;
        deltaPoseOut.orientation = deltaAttitude;
        deltaPoseOut.position = deltaPose.translation();

    }
    else
    {
        RTT::log(RTT::Warning) << "[VISO2] Stereo Odometer failed!!" << RTT::endlog();
        deltaPoseOut.time = imagePair[0].time;
        std::cout<<"Matches size: "<<viso->getMatches().size()<<"\n";

    }

    /** Port out the delta pose **/
    _delta_pose_samples_out.write(deltaPoseOut);

    /** Port out the accumulated pose **/
    _pose_samples_out.write(poseOut);

    if (_output_debug.value())
    {
        /** Draw matches in the images using the FrameHelper which internally uses openCV **/
        ::base::samples::frame::Frame *frame_ptr = intraFrame_out.write_access();
        this->drawMatches (imagePair[0].first, imagePair[0].second, viso->getMatches(), viso->getInlierIndices(), *frame_ptr);

        frame_ptr->time = imagePair[0].time;
        intraFrame_out.reset(frame_ptr);
        #ifdef DEBUG_PRINTS
        std::cout<<"frame_ptr: "<<frame_ptr<<"\n";
        std::cout<<"frame size: "<<frame_ptr->size.width*frame_ptr->size.height<<"\n";
        #endif
        _intra_frame_samples_out.write(intraFrame_out);

    }


    /** Create the point cloud from the current pair **/
    this->createPointCloud(leftColorImage, viso->getMatches(),
                        viso->getInlierIndices(), Q, deltaPose, hashIdx, hashPointcloudPrev, hashPointcloudCurr);

    /** Re-arrange the point cloud and compute the uncertainty **/
    base::samples::Pointcloud pointcloud;
    this->postProcessPointCloud (hashIdx, hashPointcloudPrev, hashPointcloudCurr, pointcloud);

    /** Port out the information (current point cloud is at the vector front) **/
    _point_cloud_samples_out.write(pointcloud);
    //_point_cloud_variance_out.write(pointsVar);
    //_point_cloud_jacobian_out.write(featuresJacob);

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
                        const Eigen::Affine3d &deltaPose,
                        boost::circular_buffer< boost::unordered_map< int32_t, int32_t > >&hashIdx,
                        boost::unordered_map< int32_t, HashPoint > &hashPointcloudPrev,
                        boost::unordered_map< int32_t, HashPoint > &hashPointcloudCurr)
{

    boost::unordered_map< int32_t, int32_t > localHashIdx;

    /** Image for the colored points **/
    cv::Mat cv_image1 = frame_helper::FrameHelper::convertToCvMat(image1);

    /** Clear hash tables **/
    hashPointcloudPrev.clear();
    hashPointcloudCurr.clear();

    /** Uncertainty in the images (left and right) planes **/
    Eigen::Matrix4d pxVar;
    pxVar << pxleftVar, Eigen::Matrix2d::Zero(),
        Eigen::Matrix2d::Zero(), pxrightVar;

    /** Create the hast table with the point cloud **/
    for (register size_t i = 0; i <inlier_indices .size(); ++i)
    {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        HashPoint hashPoint;

        std::cout<<"****\n Match ["<<i<<"] Idx (current L): "<<match.i1c<<" (current R): "<<match.i2c <<"\n";
        std::cout<<"Match ["<<i<<"] Idx (previous L): "<<match.i1p<<" (previous R): "<<match.i2p <<"\n";

        /** 3D Point **/
        ::base::Vector3d point (match.u1c + Q(0,3),  match.v1c + Q(1,3), Q(2,3));
       // std::cout<<"point["<<i<<"] "<<point[0] <<" "<<point[1]<<" "<<point[2]<<"\n";
        double disparity = match.u1c - match.u2c;
        double W  = Q(3,2)*disparity + Q(3,3);
        point = point * (1.0/W);
        std::cout<<"3dpoint["<<i<<"] "<<point[0] <<" "<<point[1]<<" "<<point[2]<<"\n";
        hashPoint.point = point;

        /** Color **/
        cv::Vec3f color = cv_image1.at<cv::Vec3b>(match.v1c, match.u1c);
        ::base::Vector4d color4d;
        color4d[0] = color[0]/255.0;//R
        color4d[1] = color[1]/255.0;//G
        color4d[2] = color[2]/255.0;//B
        color4d[3] = 1.0;//Alpha
        hashPoint.color = color4d;

        /** Uncertainty information **/
        Eigen::Matrix<double, 3, 4> noiseJacobian; /** Jacobian Matrix for the triangulation noise model */
        double disparityPower = pow(disparity,2);

        noiseJacobian <<  -(viso2param.base*match.u2c)/disparityPower, 0, (viso2param.base*match.u1c)/disparityPower, 0.00,
                -(viso2param.base*match.v1c)/disparityPower, viso2param.base/disparity, (viso2param.base*match.v1c)/disparityPower, 0,
                -(viso2param.base*viso2param.calib.f)/disparityPower, 0,  (viso2param.base*viso2param.calib.f )/disparityPower, 0;

        hashPoint.variance = noiseJacobian * pxVar * noiseJacobian.transpose();
        std::cout<<"Point var:\n"<<hashPoint.variance <<"\n";

        /** Compute Jacobian **/
        hashPoint.jacobian = computeFeaturesJacobian (deltaPose, match);

        /** Add key to the hash Point cloud **/
        hashPointcloudPrev.insert(std::make_pair(match.i1p, hashPoint));
        hashPointcloudCurr.insert(std::make_pair(match.i1c, hashPoint));

        /** Feature index get the index for the previous left **/
        localHashIdx.insert(std::make_pair(match.i1p, match.i1c));
    }

    hashIdx.push_front(localHashIdx);

}


Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r )
{
    double theta = r.norm();
    if( fabs(theta) > 1e-5 )
	return Eigen::Quaterniond( Eigen::AngleAxisd( theta, r/theta ) );
    else
	return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q )
{
    Eigen::AngleAxisd aa( q );
    return aa.axis() * aa.angle();
}

inline double sign( double v )
{
    return v > 0 ? 1.0 : -1.0;
}

Eigen::Matrix<double,3,3> skew_symmetric( const Eigen::Vector3d& r )
{
    Eigen::Matrix3d res;
    res << 0, -r.z(), r.y(),
	r.z(), 0, -r.x(),
	-r.y(), r.x(), 0;
    return res;
}

Eigen::Matrix<double,3,3> drx_by_dr( const Eigen::Quaterniond& q, const Eigen::Vector3d& x )
{
    const Eigen::Vector3d r( q_to_r( q ) );
    const double theta = r.norm();
    const double alpha = 1.0 - theta*theta/6.0;
    const double beta = 0.5 - theta*theta/24.0;
    const double gamma = 1.0 / 3.0 - theta*theta/30.0;
    const double delta = -1.0 / 12.0 + theta*theta/180.0;

    return Eigen::Matrix3d(
	    -skew_symmetric(x)*(gamma*r*r.transpose()
		- beta*skew_symmetric(r)+alpha*Eigen::Matrix3d::Identity())
	    -skew_symmetric(r)*skew_symmetric(x)*(delta*r*r.transpose() 
		+ 2.0*beta*Eigen::Matrix3d::Identity()) );
}


base::Matrix3d StereoOdometer::computeFeaturesJacobian (const Eigen::Affine3d &deltaPose, const Matcher::p_match &match)
{
    base::Matrix3d J;

    /** Get the delta 6D vector **/
    Eigen::Quaternion<double> q(deltaPose.rotation());

    #ifdef DEBUG_PRINTS
    Eigen::Vector3d trans = deltaPose.translation();
    Eigen::Vector3d euler;
    euler[2] = q.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = q.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = q.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    std::cout<< "tx: "<<trans[0]<<" ty: "<<trans[1]<<" tz: "<<trans[2]<<"\n";
    std::cout<< "rx: "<<euler[0]<<" ry: "<<euler[1]<<" rz: "<<euler[2]<<"\n";
    #endif

    /** Compute derivative matrix **/
//    Eigen::Matrix<double, 6, 3> deltaF;
//    deltaF << Eigen::Matrix3d::Identity(), drx_by_dr(q, deltaPose.translation());

    /** Experimental **/
    J = deltaPose.rotation();


    return J;
}

void StereoOdometer::postProcessPointCloud (boost::circular_buffer< boost::unordered_map< int32_t, int32_t > >& hashIdx,
                                    boost::unordered_map< int32_t, HashPoint > &hashPointcloudPrev,
                                    boost::unordered_map< int32_t, HashPoint > &hashPointcloudCurr,
                                    base::samples::Pointcloud &pointcloud)
{


    std::cout<<"hashIdx[0] is: "<< hashIdx[0].size()<<"\n";
    std::cout<<"hashIdx[1] is: "<< hashIdx[1].size()<<"\n";

    if (hashIdx.size() == 1)
    {
        std::cout<<"REQUIRED AT LEAST 4 IMAGES\n";
        return;
    }

    if (hashIdx[1].size() == 0)
    {
        std::cout<<"FIRST TIME\n";
        boost::unordered_map< int32_t, int32_t > localHashIdx;

        for(boost::unordered_map<int32_t, int32_t>::iterator it = hashIdx[0].begin(); it != hashIdx[0].end(); ++it)
        {
           localHashIdx.insert(std::make_pair(it->first, it->first));
        }
        hashIdx[1] = localHashIdx;
    }

    std::cout<<"hashIdx[0] is: "<< hashIdx[0].size()<<"\n";
    std::cout<<"hashIdx[1] is: "<< hashIdx[1].size()<<"\n";


    pointcloud.points.resize(hashIdx[1].size());
    pointcloud.colors.resize(hashIdx[1].size());

    register size_t index = 0;
    for(boost::unordered_map<int32_t, int32_t>::iterator it = hashIdx[1].begin(); it != hashIdx[1].end(); ++it)
    {
        if (hashPointcloudPrev.find(it->second) != hashPointcloudPrev.end())
        {
            HashPoint point = hashPointcloudPrev.at(it->second);
            pointcloud.points[index] = point.point;
            pointcloud.colors[index] = point.color;

            std::cout<<"FOUND PREV["<<index<<"]: "<<it->first<<" -> "<<it->second<<" point: "<<point.point[0]<<" "<<point.point[1]<<" "<<point.point[2]<<"\n";

        }
        else
        {
            std::cout<<"NOT FOUND\n";
            pointcloud.points[index] = base::Vector3d::Zero();
            pointcloud.colors[index] = base::Vector4d::Zero();
        }
        index++;
    }

    return;
}

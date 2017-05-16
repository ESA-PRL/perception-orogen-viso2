/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "StereoOdometer.hpp"

//#define DEBUG_PRINTS 1

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

void StereoOdometer::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    imagePair[0].first.time = left_frame_sample->time; //time of the left image

  //  #ifdef DEBUG_PRINTS
    //RTT::log(RTT::Warning) << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toMicroseconds()<< RTT::endlog();
    //std::cout << "[VISO2 LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
 //   #endif

    /** Get the transformation (transformation) Tbody_left_camera which is body = Tbody_left_camera left_camera **/
    if (_body_frame.value().compare(_left_camera_viso2_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_left_camera_viso22body.get(ts, tf, false))
    {
        throw std::runtime_error("[VISO2] [FATAL ERROR]: transformation for transformer not found.");
        return;
    }

    /** The image need to be in gray scale and undistorted **/
    imagePair[0].first.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperLeft.convert (*left_frame_sample, imagePair[0].first, 0, 0, _resize_algorithm.value(), true);

    /** Left color image **/
    leftColorImage.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_RGB);
    frameHelperLeft.convert (*left_frame_sample, leftColorImage, 0, 0, _resize_algorithm.value(), true);

    /** Check the time difference between inertial sensors and joint samples **/
    base::Time diffTime = imagePair[0].first.time - imagePair[0].second.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_left_frame_period/2.0))
    {
        imagePair[0].time = imagePair[0].first.time;

      //  #ifdef DEBUG_PRINTS
       // std::cout<< "[VISO2 LEFT_FRAME] [ON] ("<<diffTime.toMicroseconds()<<")\n";
   //     #endif

        std::clock_t begin = std::clock();

        viso2::Viso2Info viso2_info = this->computeStereoOdometer(ts, tf);

        std::clock_t end = std::clock();
        viso2_info.compute_time = ::base::Time::fromSeconds(static_cast<double>(end - begin) / CLOCKS_PER_SEC);
        #ifdef DEBUG_PRINTS
        std::cout<<"Visual odometry in :"<< viso2_info.compute_time.toSeconds()<<"[seconds]\n";
        #endif

        _viso2_info.write(viso2_info);
    }

    return;
}

void StereoOdometer::right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample)
{
    Eigen::Affine3d tf; /** Transformer transformation **/
    imagePair[0].second.time = right_frame_sample->time; //time stamp for the right image

   // #ifdef DEBUG_PRINTS
    //RTT::log(RTT::Warning) << "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toMicroseconds()<< RTT::endlog();
    //std::cout<< "[VISO2 RIGHT_FRAME] Frame arrived at: " <<right_frame_sample->time.toString()<<std::endl;
 //   #endif

    /** Get the transformation (transformation) Tbody_left_camera which is body = Tbody_left_camera left_camera **/
    if (_body_frame.value().compare(_left_camera_viso2_frame.value()) == 0)
    {
        tf.setIdentity();
    }
    else if (!_left_camera_viso22body.get(ts, tf, false))
    {
        throw std::runtime_error("[VISO2] [FATAL ERROR]: transformation for transformer not found.");
        return;
    }

    /** Correct distortion in image right **/
    imagePair[0].second.init(right_frame_sample->size.width, right_frame_sample->size.height, right_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    frameHelperRight.convert (*right_frame_sample, imagePair[0].second, 0, 0, _resize_algorithm.value(), true);

    /** Check the time difference between inertial sensors and joint samples **/
    base::Time diffTime = imagePair[0].second.time - imagePair[0].first.time;

    /** If the difference in time is less than half of a period run the odometry **/
    if (diffTime.toSeconds() < (_right_frame_period/2.0))
    {
        imagePair[0].time = imagePair[0].second.time;

       // #ifdef DEBUG_PRINTS
      //  std::cout<< "[VISO2 RIGHT_FRAME] [ON] ("<<diffTime.toMicroseconds()<< " " << _right_frame_period << ")\n";
      //  #endif
        std::clock_t begin = std::clock();

        viso2::Viso2Info viso2_info = this->computeStereoOdometer(ts, tf);

        std::clock_t end = std::clock();
        viso2_info.compute_time = ::base::Time::fromSeconds(static_cast<double>(end - begin) / CLOCKS_PER_SEC);
        #ifdef DEBUG_PRINTS
        std::cout<<"Visual odometry in :"<< viso2_info.compute_time.toSeconds()<<"[seconds]\n";
        #endif

        _viso2_info.write(viso2_info);
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
    viso2param.base = cameracalib.extrinsic.tx; //baseline in meters
    viso2param.calib.f = 0.5*(cameracalib.camLeft.fx + cameracalib.camRight.fx);
    viso2param.calib.cu = 0.5*(cameracalib.camLeft.cx + cameracalib.camRight.cx);
    viso2param.calib.cv = 0.5*(cameracalib.camLeft.cy + cameracalib.camRight.cy);

    /** Re-projection Q matrix **/
    cameracalibCv.setCalibration(cameracalib);
    cameracalibCv.setImageSize(cv::Size(cameracalib.camLeft.width, cameracalib.camLeft.height));
    cameracalibCv.initCv();
    cv::cv2eigen(cameracalibCv.Q, Q);

    #ifdef DEBUG_PRINTS
    std::cout<< "[VISO2 CONFIGURATION] Q re-projection matrix:\n "<<Q<<"\n";
    #endif

    /** Image plane uncertainty(variance) in pixel **/
    //pxleftVar << pow(cameracalib.camLeft.pixel_error[0],2), 0.00,
    //          0.00, pow(cameracalib.camLeft.pixel_error[1]+viso2param.match.match_disp_tolerance, 2);

    //pxrightVar << pow(cameracalib.camRight.pixel_error[0],2), 0.00,
    //          0.00, pow(cameracalib.camRight.pixel_error[1]+viso2param.match.match_disp_tolerance, 2);

    pxleftVar = cameracalib.camLeft.getPixelCovariance();
    pxrightVar = cameracalib.camRight.getPixelCovariance();

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
    poseOut.sourceFrame = _visual_odometry_source_frame.value();
    poseOut.targetFrame = _visual_odometry_target_frame.value();
    poseOut.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
    poseOut.position = Eigen::Vector3d::Zero();
    poseOut.velocity = Eigen::Vector3d::Zero();

    /** Stereo working pair **/
    ::base::samples::frame::FramePair pair;
    imagePair.set_capacity(DEFAULT_CIRCULAR_BUFFER_SIZE);
    imagePair.push_front(pair);

    ::base::samples::frame::Frame *outframe = new ::base::samples::frame::Frame();

    frame_out.reset(outframe);
    outframe = NULL;

    /** Hash Table of indexes **/
    hashPointcloud.set_capacity(DEFAULT_CIRCULAR_BUFFER_SIZE);

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

viso2::Viso2Info StereoOdometer::computeStereoOdometer(const base::Time &ts, const Eigen::Affine3d &tf)
{
    uint8_t *l_image_data, *r_image_data;
    viso2::Viso2Info viso2_info;

    /** Time **/
    viso2_info.time = ts;
    viso2_info.num_matches = ::base::NaN<double>();

    /** Delta pose **/
    base::samples::RigidBodyState deltaPoseOut;
    Eigen::Affine3d deltaPose;
    deltaPoseOut.invalidate();
    deltaPoseOut.sourceFrame = _delta_visual_odometry_source_frame.value();
    deltaPoseOut.targetFrame = _delta_visual_odometry_target_frame.value();


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

        /** Transform the deltaPose in the body frame **/
        deltaPose = tf * deltaPose * tf.inverse();

        /** On success, update current pose **/
        pose = pose * deltaPose;

        /** Output some statistics **/
        viso2_info.num_matches = viso->getNumberOfMatches();
        viso2_info.num_inliers = viso->getNumberOfInliers();
        viso2_info.ratio_inliers = (100.0*viso2_info.num_inliers/viso2_info.num_matches);

        #ifdef DEBUG_PRINTS
        std::cout << ", Matches: " << viso2_info.num_matches;
        std::cout << ", Inliers: " << viso2_info.num_inliers;
        std::cout << ", Inliers Ratio: " << viso2_info.ratio_inliers << " %" << ", Current pose: " << std::endl;
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

        /** Port out the delta pose **/
        _delta_pose_samples_out.write(deltaPoseOut);

        /** Port out the accumulated pose **/
        _pose_samples_out.write(poseOut);

         if (_output_debug.value())
        {
            /** Draw matches in the images using the FrameHelper which internally uses openCV **/
            ::base::samples::frame::Frame *frame_ptr = frame_out.write_access();
            this->drawMatches (imagePair[0].first, imagePair[0].second, viso->getMatches(), viso->getInlierIndices(), *frame_ptr);

            frame_ptr->time = imagePair[0].time;
            frame_out.reset(frame_ptr);
            #ifdef DEBUG_PRINTS
            std::cout<<"frame_ptr: "<<frame_ptr<<"\n";
            std::cout<<"frame size: "<<frame_ptr->size.width*frame_ptr->size.height<<"\n";
            #endif
            _frame_samples_out.write(frame_out);

        }

        /** Create the point cloud from the current pair **/
        this->createPointCloud(tf, leftColorImage, viso->getMatches(),
                            viso->getInlierIndices(), Q, deltaPose, hashIdx, hashPointcloud);

        /** Re-arrange the point cloud and compute the uncertainty **/
        base::samples::Pointcloud pointcloud;
        std::vector<base::Matrix3d> pointsVar;
        std::vector<unsigned int> pointsIdx;
        base::MatrixXd deltaJacobCurr, deltaJacobPrev;
        this->postProcessPointCloud (hashIdx, hashPointcloud, pointcloud, pointsVar, pointsIdx, deltaJacobCurr, deltaJacobPrev);

        /** Port out the information **/
        pointcloud.time = imagePair[0].time;
        _point_cloud_samples_out.write(pointcloud);
        _point_cloud_uncertainty_out.write(pointsVar);
        _point_cloud_indexes_out.write(pointsIdx);
        _delta_pose_jacobians_k_m_out.write(deltaJacobCurr);
        _delta_pose_jacobians_k_out.write(deltaJacobPrev);

        #ifdef DEBUG_PRINTS
        std::cout<<"Jacobian Current is "<<deltaJacobCurr.rows()<<" x "<<deltaJacobCurr.cols()<<"\n";
        std::cout<<"Jacobian Previous is "<<deltaJacobPrev.rows()<<" x "<<deltaJacobPrev.cols()<<"\n";
        std::cout<<"Uncertainty is "<<pointsVar.size()<<"\n";
        #endif


    }
    else
    {
        RTT::log(RTT::Warning) << "[VISO2] Stereo Odometer failed!!" << RTT::endlog();
        deltaPoseOut.time = imagePair[0].time;
    }


    return viso2_info;
}

void StereoOdometer::drawMatches(const base::samples::frame::Frame &image1,
                                    const base::samples::frame::Frame &image2,
                                    const std::vector<Matcher::p_match> &matches,
                                    const std::vector<int32_t>& inlier_indices,
                                    base::samples::frame::Frame &imageOutput)
{
    ::cv::Mat cvImage1 = frameHelperLeft.convertToCvMat(image1);
    ::cv::Mat cvImage2 = frameHelperRight.convertToCvMat(image2);
    ::cv::Mat cvOutImg;
    std::vector< cv::KeyPoint > keypoints1, keypoints2;
    std::vector< cv::DMatch > cvMatches;
    keypoints1.resize(inlier_indices.size());
    keypoints2.resize(inlier_indices.size());
    cvMatches.resize(inlier_indices.size());

    //std::cout<<"[drawMatches] number of features: "<<inlier_indices.size()<<"\n";

    for (register std::size_t i = 0; i < inlier_indices.size(); ++i)
    {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        #ifdef DEBUG_PRINTS
       // std::cout<<"[drawMatches] match1 "<< match.u1c << " " << match.v1c<<"\n";
       // std::cout<<"[drawMatches] match2 "<< match.u2c << " " << match.v2c<<"\n";
        #endif
        cv::KeyPoint k1 (static_cast<float>(match.u1c), static_cast<float>(match.v1c), 1);
        cv::KeyPoint k2 (static_cast<float>(match.u2c), static_cast<float>(match.v2c), 1);
        float disparity = static_cast<float>(match.u1c) - static_cast<float>(match.u2c);
        cv::DMatch cvMatch (i, i, disparity);
        keypoints1[i] = k1;
        keypoints2[i] = k2;
        cvMatches[i] = cvMatch;
    }

    if (_image_ouput_type.get() == viso2::INTRA_MATCHES)
    {
        cv::drawMatches (cvImage1, keypoints1, cvImage2, keypoints2, cvMatches, cvOutImg);
    }
    else if (_image_ouput_type.get() == viso2::INTER_KEYPOINTS)
    {
        cv::drawKeypoints (cvImage1, keypoints1, cvOutImg, cv::Scalar(0, 255, 0));
    }

    frameHelperLeft.copyMatToFrame(cvOutImg, imageOutput);

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

void StereoOdometer::createPointCloud(const Eigen::Affine3d &tf,
                        const base::samples::frame::Frame &image1,
                        const std::vector<Matcher::p_match> &matches,
                        const std::vector<int32_t>& inlier_indices,
                        const Eigen::Matrix4d &Q,
                        const Eigen::Affine3d &deltaPose,
                        boost::unordered_map< int32_t, int32_t > &hashIdx,
                        boost::circular_buffer< std::map < int32_t, HashPoint, std::less<int32_t>,
                        Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > > &hashPointcloud)
{

    boost::unordered_map< int32_t, int32_t > localHashIdx;
    boost::unordered_map< int32_t, HashPoint > localHashPointcloud;

    /** Image for the colored points **/
    cv::Mat cv_image1 = frame_helper::FrameHelper::convertToCvMat(image1);

    /** Uncertainty in the images (left and right) planes **/
    Eigen::Matrix4d pxVar;
    pxVar << pxleftVar, Eigen::Matrix2d::Zero(),
        Eigen::Matrix2d::Zero(), pxrightVar;

    /** Create the hash table with the point cloud **/
    for (register std::size_t i = 0; i <inlier_indices.size(); ++i)
    {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        HashPoint hashPoint;

        #ifdef DEBUG_PRINTS
        std::cout<<"****\n Match ["<<i<<"] Idx (current L): "<<match.i1c<<" (current R): "<<match.i2c <<"\n";
        std::cout<<"Match ["<<i<<"] Idx (previous L): "<<match.i1p<<" (previous R): "<<match.i2p <<"\n";
        #endif

        /** 3D Point **/
        //std::cout<<"image_point["<<i<<"] "<<match.u1c <<" "<<match.v1c<<"\n";
        double disparity = match.u2c - match.u1c;
        base::Vector4d image_point (match.u1c, match.v1c, disparity, 1);
        base::Vector4d homogeneous_point = Q * image_point;
        base::Vector3d point (homogeneous_point(0)/homogeneous_point(3),
                                      homogeneous_point(1)/homogeneous_point(3),
                                      homogeneous_point(2)/homogeneous_point(3));
       // base::Vector2d point2d(point[0]/point[2], point[1]/point[2]);
       // std::cout<<"point["<<i<<"] "<<point[0] <<" "<<point[1]<<" "<<point[2]<<"\n";
       // std::cout<<"point2d["<<i<<"] "<<point2d[0] <<" "<<point2d[1]<<"\n";

        /** Point in the desired frame **/
        point = tf * point;

        /** Store it in the hash element **/
        hashPoint.point = point;

        /** Color **/
        cv::Vec3b color = cv_image1.at<cv::Vec3b>(match.v1c, match.u1c);
        base::Vector4d color4d;
        color4d[0] = color[0]/255.0;//R
        color4d[1] = color[1]/255.0;//G
        color4d[2] = color[2]/255.0;//B
        color4d[3] = 1.0;//Alpha
        hashPoint.color = color4d;

        /** Uncertainty information **/
        Eigen::Matrix<double, 3, 4> noiseJacobian; /** Jacobian Matrix for the triangulation noise model */
        double disparityPower = pow(disparity,2);

        noiseJacobian <<  -(viso2param.base*match.u2c)/disparityPower, 0.00, (viso2param.base*match.u1c)/disparityPower, 0.00,
                -(viso2param.base*match.v1c)/disparityPower, viso2param.base/disparity, (viso2param.base*match.v1c)/disparityPower, 0.00,
                -(viso2param.base*viso2param.calib.f)/disparityPower, 0.00,  (viso2param.base*viso2param.calib.f)/disparityPower, 0.00;

        hashPoint.cov = noiseJacobian * pxVar * noiseJacobian.transpose();

        /** Covariance in the desired frame **/
        hashPoint.cov = tf.rotation() * hashPoint.cov * tf.rotation().transpose();

        #ifdef DEBUG_PRINTS
        std::cout<<"Point:\n"<<hashPoint.point <<"\n";
        std::cout<<"Point var:\n"<<hashPoint.cov <<"\n";
        #endif

        /** Compute Jacobian **/
        hashPoint.jacobian = computeFeaturesJacobian (deltaPose, point);

        /** Add key to the hash Point cloud **/
        localHashPointcloud.insert(std::make_pair(match.i1c, hashPoint));

        /** Feature index **/
        localHashIdx.insert(std::make_pair(match.i1c, match.i1p));
    }

    /** Assign the index hash **/
    hashIdx = localHashIdx;

    /** Order the point cloud by match indexes **/
    register unsigned int index = 0;
    std::map < int32_t, HashPoint, std::less<int32_t>,
        Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > orderedPointcloud;
    std::map<int32_t, int32_t> orderedIdx(localHashIdx.begin(), localHashIdx.end());
    for(std::map<int32_t, int32_t>::iterator it = orderedIdx.begin(); it !=orderedIdx.end(); ++it)
    {
        HashPoint point = localHashPointcloud.at(it->first);

        point.idx = index;
        orderedPointcloud[it->first] = point;

        index++;
    }
    hashPointcloud.push_front(orderedPointcloud);

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


base::Vector3d StereoOdometer::computeFeaturesJacobian (const Eigen::Affine3d &deltaPose, const base::Vector3d &point)
{
    base::Vector3d Jcolumn;

    /** Experimental **/
    Jcolumn = point - deltaPose * point;

    #ifdef DEBUG_PRINTS
    std::cout<< "J column dx: "<<Jcolumn[0]<<" dy: "<<Jcolumn[1]<<" dz: "<<Jcolumn[2]<<"\n";
    #endif

    return Jcolumn;
}

void StereoOdometer::postProcessPointCloud (boost::unordered_map< int32_t, int32_t > & hashIdx,
                                    boost::circular_buffer< std::map < int32_t, HashPoint, std::less<int32_t>,
                                        Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > > &hashPointcloud,
                                    base::samples::Pointcloud &pointcloud,
                                    std::vector<base::Matrix3d> &pointsVar,
                                    std::vector<unsigned int> &pointsIdx,
                                    base::MatrixXd &deltaJacobCurr,
                                    base::MatrixXd &deltaJacobPrev)
{


    #ifdef DEBUG_PRINTS
    std::cout<<"hashIdx has size: "<< hashIdx.size()<<"\n";
    std::cout<<"hashPointcloud[0] has size: "<< hashPointcloud[0].size()<<"\n";
    #endif

    try
    {
        /** Resize variables to the number of elements in the current point cloud **/
        pointcloud.points.resize(hashPointcloud[0].size());
        pointcloud.colors.resize(hashPointcloud[0].size());
        pointsVar.resize(hashPointcloud[0].size());
        deltaJacobCurr.resize(3, hashPointcloud[0].size());
        deltaJacobCurr.setZero();
        deltaJacobPrev.resize(3, hashPointcloud[0].size());
        deltaJacobPrev.setZero();

        if (hashPointcloud.size() >= 2.0)
            pointsIdx.resize(hashPointcloud[1].size(), UINT_MAX);
   }
    catch (const std::bad_alloc&)
    {
        RTT::log(RTT::Warning)<<"[EXCEPTION] Catching bad_alloc exception."<<RTT::endlog();
        return;
    }

    register unsigned int index = 0;
    for(std::map<int32_t, HashPoint>::iterator it = hashPointcloud[0].begin(); it != hashPointcloud[0].end(); ++it)
    {

        /** The point to the point cloud **/
        HashPoint point = it->second;
        pointcloud.points[index] = point.point; // Coordinates
        pointcloud.colors[index] = point.color; // Color
        pointsVar[index] = point.cov; // Uncertainty

        /** Get the idx on the previous point cloud **/
        int32_t prevIdx = hashIdx.at(it->first);

        #ifdef DEBUG_PRINTS
        std::cout<<"IDX["<<index<<"]: "<<it->first<<" -> "<<prevIdx<<"\n";
        std::cout<<"POINT: "<<point.point[0]<<" "<<point.point[1]<<" "<<point.point[2]<<"\n";
        #endif


        if (hashPointcloud.size() >= 2.0)
        {
            std::map<int32_t, HashPoint>::iterator itPrev = hashPointcloud[1].find(prevIdx);

            /** Look in the previous point cloud **/
            if (itPrev != hashPointcloud[1].end())
            {
                /** Get the point at the previous point cloud **/
                HashPoint point_prev = hashPointcloud[1][prevIdx];

                #ifdef DEBUG_PRINTS
                std::cout<<"FOUND PREV: "<<point_prev.point[0]<<" "<<point_prev.point[1]<<" "<<point_prev.point[2]<<"\n";
                #endif

                /** Get the current Jacobian **/
                deltaJacobCurr.col(index) = point.jacobian;

                /** Get the previous Jacobian **/
                deltaJacobPrev.col(index) = point_prev.jacobian;

                /** Vector of indexes **/
               // std::cout<<"prev_idx "<< point_prev.idx<<" -> index "<< index <<"\n";
                pointsIdx[point_prev.idx] = index;
            }
            else
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"NOT FOUND\n";
                #endif

                /** Set the current Jacobian to Zero**/
                deltaJacobCurr.col(index).setZero();

                /** Set the previous Jacobian to Zero**/
                deltaJacobPrev.col(index).setZero();
            }
        }

        index++;
    }

    return;
}

/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VISO2_STEREOODOMETER_TASK_HPP
#define VISO2_STEREOODOMETER_TASK_HPP

#include "viso2/StereoOdometerBase.hpp"

/** Opencv for the conversion **/
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv/highgui.h>

/** Rock libraries **/
#include "frame_helper/FrameHelper.h" /** Rock lib for manipulate frames **/
#include "frame_helper/FrameHelperTypes.h" /** Types for FrameHelper **/
#include "frame_helper/Calibration.h" /** Rock type for camera calibration parameters **/
#include "frame_helper/CalibrationCv.h" /** Rock type for camera OpenCv calibration **/

/** Rock Types **/
#include <base/samples/RigidBodyState.hpp>

/** Standard **/
#include <vector>
#include <map>

/** Boost **/
#include <boost/shared_ptr.hpp> /** For shared pointers **/
#include <boost/circular_buffer.hpp> /** For circular buffers **/
#include <boost/unordered_map.hpp>

/** LibViso2 includes **/
#include <viso2/matrix.h>
#include <viso2/viso_stereo.h>
#include <ctime>

/** Eigen **/
#include<Eigen/StdVector>

namespace viso2 {


    struct HashPoint
    {
        size_t idx; /* Index in the point cloud **/
        base::Vector3d point; /** 3d coordinate **/
        base::Vector4d color; /** Color info **/
        base::Matrix3d cov; /** Covariance matrix of the 3d point**/
        base::Vector3d jacobian; /** Jacobian component of this feature point **/
    };


    /*! \class StereoOdometer 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','viso2::StereoOdometer')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class StereoOdometer : public StereoOdometerBase
    {
	friend class StereoOdometerBase;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        static const int DEFAULT_CIRCULAR_BUFFER_SIZE = 2;

    protected:

        /**************************/
        /*** Property Variables ***/
        /**************************/
        // set most important visual odometry parameters
        // for a full parameter list, look at: viso2/viso_stereo.h
        VisualOdometryStereo::parameters viso2param;

        //Intrinsic and extrinsic parameters for the pinhole camera model
        frame_helper::StereoCalibration cameracalib;

        /** Open Cv calibration for the perspective transformation Matrix Q **/
        frame_helper::StereoCalibrationCv cameracalibCv;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/
        boost::shared_ptr<VisualOdometryStereo> viso; /** Pointer to Viso2 object **/
        boost::circular_buffer<base::samples::frame::FramePair> imagePair; /** Left and right images **/
        frame_helper::FrameHelper frameHelperLeft, frameHelperRight; /** Frame helper **/
        Eigen::Matrix4d Q; /** Re-projection matrix **/
        ::base::samples::frame::Frame leftColorImage;/** coloring point clouds (if selected) */
        ::base::Matrix2d pxleftVar, pxrightVar; /** Error variance of image plane in pixel units **/
        boost::unordered_map< int32_t, int32_t > hashIdx; /** current to previous index **/
        boost::circular_buffer< std::map < int32_t, HashPoint, std::less<int32_t>,
            Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > > hashPointcloud; /** Circular buffer of hash Point Cloud **/

        /***************************/
        /** Output Port Variables **/
        /***************************/
        Eigen::Affine3d pose; /** Accumulated pose **/
        base::samples::RigidBodyState poseOut; /** Accumulated pose **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_out; /** Debug intra frame image **/

    protected:

        virtual void left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);
        virtual void right_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &right_frame_sample);

    public:
        /** TaskContext constructor for StereoOdometer
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        StereoOdometer(std::string const& name = "viso2::StereoOdometer");

        /** TaskContext constructor for StereoOdometer 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        StereoOdometer(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of StereoOdometer
         */
	~StereoOdometer();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Computes one step of stereo visual odometry
         */
        viso2::Viso2Info computeStereoOdometer(const base::Time &ts, const Eigen::Affine3d &tf);

        void drawMatches(const base::samples::frame::Frame &image1, const base::samples::frame::Frame &image2,
                        const std::vector<Matcher::p_match> &matches, const std::vector<int32_t>& inlier_indices, base::samples::frame::Frame &imageOutput);

        void createDistanceImage(const base::samples::frame::Frame &image1, const base::samples::frame::Frame &image2,
                        const std::vector<Matcher::p_match> &matches, const VisualOdometryStereo::parameters &viso2param,
                        base::samples::DistanceImage &distImage);

        void createPointCloud(const Eigen::Affine3d &tf,
                        const base::samples::frame::Frame &image1,
                        const std::vector<Matcher::p_match> &matches,
                        const std::vector<int32_t>& inlier_indices,
                        const Eigen::Matrix4d &Q,
                        const Eigen::Affine3d &deltaPose,
                        boost::unordered_map< int32_t, int32_t > &hashIdx,
                        boost::circular_buffer< std::map < int32_t, HashPoint, std::less<int32_t>,
                                    Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > > &hashPointcloud);

        base::Vector3d computeFeaturesJacobian (const Eigen::Affine3d &deltaPose,
                                            const base::Vector3d &point);

        void postProcessPointCloud (boost::unordered_map< int32_t, int32_t > & hashIdx,
                                    boost::circular_buffer< std::map < int32_t, HashPoint, std::less<int32_t>,
                                        Eigen::aligned_allocator< std::pair < const int32_t, HashPoint > > > > &hashPointcloud,
                                    base::samples::Pointcloud &pointcloud,
                                    std::vector<base::Matrix3d> &pointsVar,
                                    std::vector<size_t> &pointsIdx,
                                    base::MatrixXd &deltaJacobCurr,
                                    base::MatrixXd &deltaJacobPrev);

    };
}

#endif


#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer'
require 'vizkit'
require 'utilrb'
require 'eigen'

include Orocos
Orocos::CORBA.max_message_size = 90000000000000

if ARGV.size < 1 then 
    puts "usage: process_logs_viso.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

#Load typekit for rbs access in ruby
Orocos.load_typekit 'base'

PU_ST_LS_2_LS = 78.0/1000.0
BC_2_SWB_X = 329/1000.00
VICON_MARKER_2_BC_Y = 171.5/1000.00
#Sherpa transforms
qVicon2ViconMarker = Eigen::Quaternion.from_euler(Eigen::Vector3.new(0.00, 90.0*(Math::PI/180.00), 0.0), 2,1,0)
qViconMarker2BC = qVicon2ViconMarker * Eigen::Quaternion.from_euler(Eigen::Vector3.new((180.0-18.0)*(Math::PI/180.00), 0.00, 0.0), 2,1,0)
tViconMarker2BC = Eigen::Vector3.new(BC_2_SWB_X, -VICON_MARKER_2_BC_Y, 0.0 )
qWorld2Camera = Eigen::Quaternion.from_euler(Eigen::Vector3.new(-90.0*(Math::PI/180.00), 0.00, -90.0*(Math::PI/180.00)), 2,1,0)
qTiltCamera2Camera = Eigen::Quaternion.from_euler(Eigen::Vector3.new(0.00, 0.00, 30.0*(Math::PI/180.00)), 2,1,0)
tBC2CameraBase = Eigen::Vector3.new(BC_2_SWB_X, 0.0, 0.0 )

initialization = true
initPose = Types::Base::Samples::RigidBodyState.new


Orocos.run 'viso2::StereoOdometer' => 'visual_odometry' do

    # log all the output ports
    #Orocos.log_all_ports
    Orocos.conf.load_dir('../config/')

    # get the task
    visual_odometry = Orocos.name_service.get 'visual_odometry'
    Orocos.conf.apply(visual_odometry, ['default', 'bumblebee'], :override => true )

    # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )
    log_replay.use_sample_time = true

    #mapping the logs into the input ports (in the logs of 20130702 the right and left camera are changed!!!!)
    #log_replay.left_camera.frame.connect_to(visual_odometry.left_frame, :type => :buffer, :size => 100 )
    log_replay.stereo_camera_firewire.frame_right.connect_to(visual_odometry.left_frame, :type => :buffer, :size => 100 )
    #log_replay.right_camera.frame.connect_to(visual_odometry.right_frame, :type => :buffer, :size => 100 )
    log_replay.stereo_camera_firewire.frame_left.connect_to(visual_odometry.right_frame, :type => :buffer, :size => 100 )

    visual_odometry.configure
    visual_odometry.start

    # Trajectory of the ground truth
    #truthTrajectory = Vizkit.default_loader.TrajectoryVisualization
    #truthTrajectory.setColor(Eigen::Vector3.new(0, 255, 0)) #Green line
    #truthTrajectory.setPluginName("GroundTruthTrajectory")


    #Vicon marker visualization
    #rbsTruth = Vizkit.default_loader.RigidBodyStateVisualization
    #rbsTruth.setColor(Eigen::Vector3.new(0, 255, 0))#Green rbs
    #rbsTruth.setPluginName("GroundTruthPose")
    #rbsTruth.resetModel(0.4)

    #log_replay.Task.pose_samples.connect_to do |data,_|
    #    data.orientation = data.orientation * qViconMarker2BC
    #    data.position = data.position - (data.orientation * tViconMarker2BC)
    #    rbsTruth.updateRigidBodyState(data)
    #    truthTrajectory.updateTrajectory(data.position)
    #    if (initialization)
    #        initPose.position = data.position+(data.orientation * tBC2CameraBase)
    #        initPose.orientation = data.orientation
    #        initialization = false
    #    end
    #end

    # Trajectory of the ground truth
    #cameraTrajectory = Vizkit.default_loader.TrajectoryVisualization
    #cameraTrajectory.setColor(Eigen::Vector3.new(255, 0, 0)) #Red line
    #cameraTrajectory.setPluginName("CameraTrajectory")


    ##Vicon marker visualization
    #rbsCamera = Vizkit.default_loader.RigidBodyStateVisualization
    #rbsCamera.setColor(Eigen::Vector3.new(255, 0, 0))#Red rbs
    #rbsCamera.setPluginName("CameraPose")
    #rbsCamera.resetModel(0.4)

    #visual_odometry.pose_samples_out.connect_to do |data, _|
    #    #data.orientation = data.orientation * qTiltCamera2Camera
    #    data.orientation = (data.orientation * initPose.orientation)*qWorld2Camera
    #    data.position =  initPose.position + (data.orientation * data.position)
    #    rbsCamera.updateRigidBodyState(data)
    #    cameraTrajectory.updateTrajectory(data.position)
    #end

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

    Vizkit.exec
end


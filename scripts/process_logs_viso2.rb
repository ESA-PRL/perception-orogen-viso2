#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer/runtime'
require 'vizkit'
require 'utilrb'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs_viso.rb <data_log_directory>"
    exit
end

#Initializes the CORBA communication layer
Orocos.initialize

Orocos.run 'viso2::StereoOdometer' => 'visual_odometry' do

    # get the task
    visual_odometry = Orocos.name_service.get 'visual_odometry'

    # connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open( ARGV[0] )

    #mapping the logs into the input ports
    log_replay.image_preprocessing_left.out_frame.connect_to(visual_odometry.left_frame, :type => :buffer, :size => 100 )
    log_replay.image_preprocessing_right.out_frame.connect_to(visual_odometry.right_frame, :type => :buffer, :size => 100 )

    visual_odometry.configure
    visual_odometry.start

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1 #4

    Vizkit.exec
end


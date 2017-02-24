#include <kitti_player/Player.h>
#include <kitti_player/PlayerSupport.h>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/progress.hpp>
#include <boost/timer.hpp>

#include <sensor_msgs/distortion_models.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/highgui/highgui.hpp>

#if !defined(_MSC_VER)
  #include <sys/select.h>
#endif

#include <stdio.h>
#include <iostream>

#include <rosbag/bag.h>

using namespace std;

namespace fs = boost::filesystem;

void PlayerOptions::printRequiredDirectoryTree()
{
    cout << "kitti_player needs a directory tree like the following:" << endl;
    cout << "└── 2011_09_26_drive_0001_sync" << endl;
    cout << "    ├── image_00              " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │   └ timestamps.txt      " << endl;
    cout << "    ├── image_01              " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │   └ timestamps.txt      " << endl;
    cout << "    ├── image_02              " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │   └ timestamps.txt      " << endl;
    cout << "    ├── image_03              " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │   └ timestamps.txt      " << endl;
    cout << "    ├── oxts                  " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │   └ timestamps.txt      " << endl;
    cout << "    ├── velodyne_points       " << endl;
    cout << "    │   └── data              " << endl;
    cout << "    │     └ timestamps.txt    " << endl;
    cout << "    ├── calib_cam_to_cam.txt  " << endl;
    cout << "    ├── calib_imu_to_velo.txt " << endl;
    cout << "    └── calib_velo_to_cam.txt " << endl << endl;
}

Player::Player(ros::NodeHandle& n, ros::NodeHandle& pn, PlayerOptions options)
 : n_(n),
   pn_(pn),
   options_(options),
   image00_seq_(0),
   image01_seq_(0),
   image02_seq_(0),
   image03_seq_(0),
   velodyne_seq_(0),
   gps_seq_(0),
   imu_seq_(0),
   odom_seq_(0),
   tf_seq_(0),
   static_tf_seq_(0),
   has_tf_msg_(false),
   static_tf_published_(false),
   has_clock_msg_(false),
   step_(false),
   quit_(false),
   has_odom_origin_(false),
   terminal_modified_(false)
{

    dir_root_             = options_.path;
    dir_image00_          = (fs::path(options_.path) / fs::path("image_00/data")       ).string();
    dir_image01_          = (fs::path(options_.path) / fs::path("image_01/data")       ).string();
    dir_image02_          = (fs::path(options_.path) / fs::path("image_02/data")       ).string();
    dir_image03_          = (fs::path(options_.path) / fs::path("image_03/data")       ).string();
    dir_oxts_             = (fs::path(options_.path) / fs::path("oxts/data")           ).string();
    dir_velodyne_points_  = (fs::path(options_.path) / fs::path("velodyne_points/data")).string();

    dir_timestamp_image00_  = (fs::path(options_.path) / fs::path("image_00")       ).string();
    dir_timestamp_image01_  = (fs::path(options_.path) / fs::path("image_01")       ).string();
    dir_timestamp_image02_  = (fs::path(options_.path) / fs::path("image_02")       ).string();
    dir_timestamp_image03_  = (fs::path(options_.path) / fs::path("image_03")       ).string();
    dir_timestamp_oxts_     = (fs::path(options_.path) / fs::path("oxts")           ).string();
    dir_timestamp_velodyne_ = (fs::path(options_.path) / fs::path("velodyne_points")).string();

    ROS_INFO_STREAM ("Checking directories...");
    if(checkAllDirectories())
    {
        ROS_ERROR("Incorrect tree directory , use --help for details");
        quit_ = true;
        return;
    }
    ROS_INFO_STREAM (options_.path << "\t[OK]");

    total_entries_  = countNumEntries();
    entries_played_ = 0;

    // Initialize publishers
    image_transport::ImageTransport it(pn_);
    if(options_.all_data || options_.grayscale)
    {
        pub00_ = it.advertiseCamera("grayscale/left/image_rect", 1);
        pub01_ = it.advertiseCamera("grayscale/right/image_rect", 1);
    }
    if(options_.all_data || options_.color)
    {
        pub02_ = it.advertiseCamera("color/left/image_rect", 1);
        pub03_ = it.advertiseCamera("color/right/image_rect", 1);
    }
    if(options_.all_data || options_.velodyne)
        map_pub_ = pn_.advertise<sensor_msgs::PointCloud2> ("hdl64e", 1, true);
    if(options_.all_data || options_.gps)
        gps_pub_ = pn_.advertise<sensor_msgs::NavSatFix>  ("oxts/gps", 1, true);
    if(options_.all_data || options_.imu)
        imu_pub_ = pn_.advertise<sensor_msgs::Imu>  ("oxts/imu", 1, true);
    if(options_.all_data || options_.odometry)
        odom_pub_ = pn_.advertise<nav_msgs::Odometry>  ("odometry", 1, true);

    if(options_.clock)
        clock_pub_ = n_.advertise<rosgraph_msgs::Clock>  ("clock", 1, true);

    ROS_INFO_STREAM("Waiting 0.2 seconds after advertising topics...");
    ros::WallDuration(0.2).sleep();
    ROS_INFO_STREAM("Waiting 0.2 seconds after advertising topics...  done");

    if(options_.viewer)
    {
        ROS_INFO_STREAM("Opening CV viewer(s)");
        if(options_.color || options_.all_data)
        {
            ROS_DEBUG_STREAM("color||all " << options_.color << " " << options_.all_data);
            cv::namedWindow("CameraSimulator Color Viewer",CV_WINDOW_AUTOSIZE);
            string full_filename_image = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%010d") % 0 ) + ".png")).string();
            cv_image02_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        if(options_.grayscale || options_.all_data)
        {
            ROS_DEBUG_STREAM("grayscale||all " << options_.grayscale << " " << options_.all_data);
            cv::namedWindow("CameraSimulator Grayscale Viewer",CV_WINDOW_AUTOSIZE);
            string full_filename_image = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%010d") % 0 ) + ".png")).string();
            cv_image00_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        ROS_INFO_STREAM("Opening CV viewer(s)... OK");
    }

    // Load camera info
    if(options_.grayscale || options_.all_data)
    {
        ros_cameraInfoMsg_camera00_.header.frame_id = options_.frame_image00;
        ros_cameraInfoMsg_camera00_.height = 0;
        ros_cameraInfoMsg_camera00_.width  = 0;
        //ros_cameraInfoMsg_camera00_.D.resize(5);
        //ros_cameraInfoMsg_camera00_.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;
        ros_cameraInfoMsg_camera01_.header.frame_id = options_.frame_image01;
        ros_cameraInfoMsg_camera01_.height = 0;
        ros_cameraInfoMsg_camera01_.width  = 0;
        //ros_cameraInfoMsg_camera01_.D.resize(5);
        //ros_cameraInfoMsg_camera00_.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;
        if(!( PlayerSupport::getCamCalibration(dir_root_,"00",ros_cameraInfoMsg_camera00_.K.data(),ros_cameraInfoMsg_camera00_.D,ros_cameraInfoMsg_camera00_.R.data(),ros_cameraInfoMsg_camera00_.P.data()) &&
              PlayerSupport::getCamCalibration(dir_root_,"01",ros_cameraInfoMsg_camera01_.K.data(),ros_cameraInfoMsg_camera01_.D,ros_cameraInfoMsg_camera01_.R.data(),ros_cameraInfoMsg_camera01_.P.data())
            )
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA00/CAMERA01 calibration");
            quit_ = true;
            return;
        }
        //Assume same height/width for the camera pair
        string full_filename_image = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%010d") % 0 ) + ".png")).string();
        cv_image00_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera01_.height = ros_cameraInfoMsg_camera00_.height = cv_image00_.rows;// -1; TODO: CHECK -1?
        ros_cameraInfoMsg_camera01_.width  = ros_cameraInfoMsg_camera00_.width  = cv_image00_.cols;// -1;
    }
    if(options_.color || options_.all_data)
    {
        ros_cameraInfoMsg_camera02_.header.frame_id = options_.frame_image02;
        ros_cameraInfoMsg_camera02_.height = 0;
        ros_cameraInfoMsg_camera02_.width  = 0;
        //ros_cameraInfoMsg_camera02_.D.resize(5);
        //ros_cameraInfoMsg_camera02_.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;
        ros_cameraInfoMsg_camera03_.header.frame_id = options_.frame_image03;
        ros_cameraInfoMsg_camera03_.height = 0;
        ros_cameraInfoMsg_camera03_.width  = 0;
        //ros_cameraInfoMsg_camera03_.D.resize(5);
        //ros_cameraInfoMsg_camera03_.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;
        if(!(  PlayerSupport::getCamCalibration(dir_root_,"02",ros_cameraInfoMsg_camera02_.K.data(),ros_cameraInfoMsg_camera02_.D,ros_cameraInfoMsg_camera02_.R.data(),ros_cameraInfoMsg_camera02_.P.data()) &&
               PlayerSupport::getCamCalibration(dir_root_,"03",ros_cameraInfoMsg_camera03_.K.data(),ros_cameraInfoMsg_camera03_.D,ros_cameraInfoMsg_camera03_.R.data(),ros_cameraInfoMsg_camera03_.P.data())
            )
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA02/CAMERA03 calibration");
            quit_ = true;
            return;
        }
        //Assume same height/width for the camera pair
        string full_filename_image = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%010d") % 0 ) + ".png")).string();
        cv_image02_ = cv::imread(full_filename_image, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera03_.height = ros_cameraInfoMsg_camera02_.height = cv_image02_.rows;// -1;TODO: CHECK, qui potrebbe essere -1
        ros_cameraInfoMsg_camera03_.width  = ros_cameraInfoMsg_camera02_.width  = cv_image02_.cols;// -1;
    }
}

Player::~Player()
{
    if(options_.viewer)
    {
        ROS_INFO_STREAM(" Closing CV viewer(s)");
        if(options_.color || options_.all_data)
            cv::destroyWindow("CameraSimulator Color Viewer");
        if(options_.grayscale || options_.all_data)
            cv::destroyWindow("CameraSimulator Grayscale Viewer");
        ROS_INFO_STREAM(" Closing CV viewer(s)... OK");
    }

    restoreTerminal();
}

int Player::checkAllDirectories()
{
    // Return non-zero if directory tree is wrong
    if (
            (options_.all_data   && (   (!fs::exists(fs::path(dir_image00_))         || !fs::is_directory(fs::path(dir_image00_))) ||
                                        (!fs::exists(fs::path(dir_image01_))         || !fs::is_directory(fs::path(dir_image01_))) ||
                                        (!fs::exists(fs::path(dir_image02_))         || !fs::is_directory(fs::path(dir_image02_))) ||
                                        (!fs::exists(fs::path(dir_image03_))         || !fs::is_directory(fs::path(dir_image03_))) ||
                                        (!fs::exists(fs::path(dir_oxts_))            || !fs::is_directory(fs::path(dir_oxts_)))    ||
                                        (!fs::exists(fs::path(dir_velodyne_points_)) || !fs::is_directory(fs::path(dir_velodyne_points_)))
                                    )
            )
            ||
            (options_.color      && (   (!fs::exists(fs::path(dir_image02_)) || !fs::is_directory(fs::path(dir_image02_))) ||
                                        (!fs::exists(fs::path(dir_image03_)) || !fs::is_directory(fs::path(dir_image03_)))
                                    )
            )
            ||
            (options_.grayscale  && (   (!fs::exists(fs::path(dir_image00_)) || !fs::is_directory(fs::path(dir_image00_))) ||
                                        (!fs::exists(fs::path(dir_image01_)) || !fs::is_directory(fs::path(dir_image01_)))
                                    )
            )
            ||
            (options_.imu        && (   (!fs::exists(fs::path(dir_oxts_)) || !fs::is_directory(fs::path(dir_oxts_)))))
            ||
            (options_.gps        && (   (!fs::exists(fs::path(dir_oxts_)) || !fs::is_directory(fs::path(dir_oxts_)))))
            ||
            (options_.odometry   && (   (!fs::exists(fs::path(dir_oxts_)) || !fs::is_directory(fs::path(dir_oxts_)))))
            ||
            (options_.velodyne   && (   (!fs::exists(fs::path(dir_velodyne_points_)) || !fs::is_directory(fs::path(dir_velodyne_points_)))))
            ||
            (options_.timestamps && (   (!fs::exists(fs::path(dir_timestamp_image00_))  || !fs::is_directory(fs::path(dir_timestamp_image00_))) ||
                                        (!fs::exists(fs::path(dir_timestamp_image01_))  || !fs::is_directory(fs::path(dir_timestamp_image01_))) ||
                                        (!fs::exists(fs::path(dir_timestamp_image02_))  || !fs::is_directory(fs::path(dir_timestamp_image02_))) ||
                                        (!fs::exists(fs::path(dir_timestamp_image03_))  || !fs::is_directory(fs::path(dir_timestamp_image03_))) ||
                                        (!fs::exists(fs::path(dir_timestamp_oxts_))     || !fs::is_directory(fs::path(dir_timestamp_oxts_)))    ||
                                        (!fs::exists(fs::path(dir_timestamp_velodyne_)) || !fs::is_directory(fs::path(dir_timestamp_velodyne_)))
                                    )
            )
        )
    {
        return 1;
    }

    // If we get here we're OK
    return 0;
}

unsigned int Player::countNumEntries()
{
    // sync datasets have the same number of elements on each folder, so
    // we just need to count one of them.
    if (options_.all_data)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image02_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    if (options_.color)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image02_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    if (options_.grayscale)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_image00_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    if (options_.gps || options_.imu || options_.odometry)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_oxts_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }
    if (options_.velodyne)
    {
        return std::count_if(
            fs::directory_iterator(fs::path(dir_velodyne_points_)),
            fs::directory_iterator(),
            boost::bind( static_cast<bool(*)(const fs::path&)>(fs::is_regular_file), 
            boost::bind( &fs::directory_entry::path, _1 ) ) );
    }

    // We should never get here.
    ROS_ERROR("An error occured when counting number of entries.");
    return 0;
}

void Player::writeBag()
{
    rosbag::Bag bag;
    bag.open(options_.bagpath, rosbag::bagmode::Write);

    setupTerminal();
    boost::progress_display progress(total_entries_) ;
    while(entries_played_<total_entries_ && !quit_ && ros::ok())
    {
        processUserInput();

        if(!ros::ok() || quit_)  break;

        current_timestamp_ = getTimestampAt(entries_played_);

        // do it once
        if(options_.statictf && !static_tf_published_)
        {
            loadStaticTfs();
            bag.write("tf_static", current_timestamp_, static_tf_msg_);
            static_tf_published_ = true;
        }

        if(options_.odometry || options_.odomtf)
        {
            loadOdometryDataAt(entries_played_);
            if(options_.odometry)
                bag.write(ros::names::resolve("~odometry"), current_timestamp_, odom_msg_);
            if(options_.odomtf)
                bag.write("tf", current_timestamp_, tf_msg_);
        }

        if(options_.color || options_.all_data)
        {
            loadColorDataAt(entries_played_);
            bag.write(ros::names::resolve("~color/left/image_rect"), current_timestamp_, ros_msg02_);
            bag.write(ros::names::resolve("~color/left/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera02_);
            bag.write(ros::names::resolve("~color/right/image_rect"), current_timestamp_, ros_msg03_);
            bag.write(ros::names::resolve("~color/right/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera03_);
        }

        if(options_.grayscale || options_.all_data)
        {
            loadGrayscaleDataAt(entries_played_);
            bag.write(ros::names::resolve("~grayscale/left/image_rect"), current_timestamp_, ros_msg00_);
            bag.write(ros::names::resolve("~grayscale/left/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera00_);
            bag.write(ros::names::resolve("~grayscale/right/image_rect"), current_timestamp_, ros_msg01_);
            bag.write(ros::names::resolve("~grayscale/right/camera_info"), current_timestamp_, ros_cameraInfoMsg_camera01_);
        }

        if(options_.velodyne || options_.all_data)
        {
            loadVelodyneDataAt(entries_played_);
            bag.write(ros::names::resolve("~hdl64e"), current_timestamp_, points_msg_);
        }

        if(options_.gps || options_.all_data)
        {
            loadGpsDataAt(entries_played_);
            bag.write(ros::names::resolve("~oxts/gps"), current_timestamp_, gps_msg_);
        }
        
        if(options_.imu || options_.all_data)
        {
            loadImuDataAt(entries_played_);
            bag.write(ros::names::resolve("~oxts/imu"), current_timestamp_, imu_msg_);
        }

        ++progress;
        entries_played_++;
    }

    bag.close();
    restoreTerminal();
    quit_ = true;
}
    
void Player::publish()
{
    paused_ = options_.start_paused;
    ros::WallRate loop_rate(options_.frequency);

    if(options_.clock)
        clock_thread_ = boost::thread( &Player::clockThread, this );

    if(options_.odomtf)
        tf_thread_ = boost::thread( &Player::tfThread, this );

    setupTerminal();
    boost::progress_display progress(total_entries_) ;
    while(entries_played_<total_entries_ && !quit_ && ros::ok())
    {
        processUserInput();

        if(!ros::ok() || quit_)  break;
        if(paused_ && !step_)  continue;

        // variable used to display time spent on the loop
        boost::timer t;

        if(options_.timestamps || options_.clock)
            current_timestamp_ = getTimestampAt(entries_played_);
        else
            current_timestamp_ = ros::Time::now();

        // do it once
        if(options_.statictf && !static_tf_published_)
        {
            loadStaticTfs();
            std::for_each(static_tf_msg_.transforms.begin(),static_tf_msg_.transforms.end(),[this](geometry_msgs::TransformStamped & msg)
            {
                static_tf_br_.sendTransform(msg);
            } );
            static_tf_published_ = true;
        }

        if(options_.color || options_.all_data)
        {
            loadColorDataAt(entries_played_);

            if(options_.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Color Viewer",cv_image02_);
                //give some time to draw images
                cv::waitKey(5);
            }

            pub02_.publish(ros_msg02_,ros_cameraInfoMsg_camera02_);
            pub03_.publish(ros_msg03_,ros_cameraInfoMsg_camera03_);
        }

        if(options_.grayscale || options_.all_data)
        {
            loadGrayscaleDataAt(entries_played_);

            if(options_.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Grayscale Viewer",cv_image00_);
                //give some time to draw images
                cv::waitKey(5);
            }

            pub00_.publish(ros_msg00_,ros_cameraInfoMsg_camera00_);
            pub01_.publish(ros_msg01_,ros_cameraInfoMsg_camera01_);
        }

        if(options_.velodyne || options_.all_data)
        {
            loadVelodyneDataAt(entries_played_);
            map_pub_.publish(points_msg_);
        }

        if(options_.gps || options_.all_data)
        {
            loadGpsDataAt(entries_played_);
            gps_pub_.publish(gps_msg_);
        }
        
        if(options_.imu || options_.all_data)
        {
            loadImuDataAt(entries_played_);
            imu_pub_.publish(imu_msg_);
        }

        if(options_.odometry || options_.odomtf)
        {
            loadOdometryDataAt(entries_played_);
            if(options_.odometry)
                odom_pub_.publish(odom_msg_);
        }

        if(options_.clock)
        {
            // Update clock to be published in the clock thread
            clock_lock_.lock();
            clock_msg_.clock = current_timestamp_;
            clock_lock_.unlock();
            if(!has_clock_msg_) has_clock_msg_ = true;
        }

        ++progress;
        entries_played_++;
        ROS_DEBUG_STREAM("Loop took "<< t.elapsed() << " [s].");        
        loop_rate.sleep();
    }
    
    // clean up
    restoreTerminal();
    quit_ = true;
    if(options_.clock)
        clock_thread_.join();
    if(options_.odomtf)
        tf_thread_.join();
    ROS_INFO("Done.");
}

tf2::Transform Player::getOdomTfAt(unsigned int entry)
{
    tf2::Transform t;
    string full_filename_oxts = (fs::path(dir_oxts_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".txt")).string();
    if (!PlayerSupport::getOdomTf(full_filename_oxts,&t))
    {
        ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
        quit_ = true;
        return t;
    }

    if(!has_odom_origin_)
    {
        odom_origin_ = t;
        // set origin orientation to 0
        odom_origin_.setRotation(tf2::Quaternion(0,0,0,1));
        has_odom_origin_ = true;
    }

    t = odom_origin_.inverse() * t;
    return t;
}

void Player::tfThread()
{
    // Setup publishing rate
    ros::WallRate tf_rate = ros::WallRate(100.0);

    ROS_DEBUG_STREAM("Starting tf publish thread.");
    while(ros::ok() && !quit_)
    {
        if(has_tf_msg_)
        {
            tf_lock_.lock();
            tf_br_.sendTransform(tf_msg_.transforms);
            tf_lock_.unlock();
        }
        tf_rate.sleep();
    }
    ROS_DEBUG_STREAM("Finishing tf publish thread.");
}

void Player::loadOdometryDataAt(unsigned int entry)
{
    tf2::Transform t = getOdomTfAt(entry);

    if(options_.odomtf)
    {
        geometry_msgs::TransformStamped tf_stamped_msg;
        tf_stamped_msg.header.stamp = current_timestamp_;
        tf_stamped_msg.header.seq = tf_seq_++;
        tf_stamped_msg.header.frame_id = options_.frame_odom;
        tf_stamped_msg.child_frame_id = options_.frame_oxts;
        tf2::convert(t,tf_stamped_msg.transform);
        tf_lock_.lock();
        tf_msg_.transforms.clear();
        tf_msg_.transforms.push_back(tf_stamped_msg);
        tf_lock_.unlock();
        if(!has_tf_msg_) has_tf_msg_ = true;
    }

    // fill up the odometry msg
    odom_msg_.header.frame_id = options_.frame_odom;
    odom_msg_.header.stamp = current_timestamp_;
    odom_msg_.header.seq = odom_seq_++;

    odom_msg_.child_frame_id = options_.frame_oxts;

    odom_msg_.pose.pose.position.x = t.getOrigin().getX();
    odom_msg_.pose.pose.position.y = t.getOrigin().getY();
    odom_msg_.pose.pose.position.z = t.getOrigin().getZ();

    odom_msg_.pose.pose.orientation.x = t.getRotation().getX();
    odom_msg_.pose.pose.orientation.y = t.getRotation().getY();
    odom_msg_.pose.pose.orientation.z = t.getRotation().getZ();
    odom_msg_.pose.pose.orientation.w = t.getRotation().getW();
}

void Player::loadImuDataAt(unsigned int entry)
{
    std_msgs::Header header_support;
    header_support.frame_id = options_.frame_oxts;
    header_support.stamp = current_timestamp_;
    header_support.seq = imu_seq_++;

    string full_filename_oxts = (fs::path(dir_oxts_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".txt")).string();
    if (!PlayerSupport::getIMU(full_filename_oxts,&imu_msg_,&header_support))
    {
        ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
        quit_ = true;
        return;
    }
}

void Player::loadGpsDataAt(unsigned int entry)
{
    std_msgs::Header header_support;
    header_support.frame_id = options_.frame_oxts;
    header_support.stamp = current_timestamp_;
    header_support.seq = gps_seq_++;

    string full_filename_oxts = (fs::path(dir_oxts_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".txt")).string();
    if (!PlayerSupport::getGPS(full_filename_oxts,&gps_msg_,&header_support))
    {
        ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
        quit_ = true;
        return;
    }
}

void Player::loadVelodyneDataAt(unsigned int entry)
{
    string full_filename_velodyne = (fs::path(dir_velodyne_points_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".bin")).string();
    fstream input(full_filename_velodyne.c_str(), ios::in | ios::binary);
    if(!input.good())
    {
        ROS_ERROR_STREAM ( "Could not read file: " << full_filename_velodyne );
        quit_ = true;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> points;

    ROS_DEBUG_STREAM ("reading " << full_filename_velodyne);
    input.seekg(0, ios::beg);
    while(input.good() && !input.eof())
    {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points.push_back(point);
    }
    input.close();

    // convert from pcl to sensor_msg 
    pcl::toROSMsg(points, points_msg_);
    // fill up header
    points_msg_.header.frame_id = options_.frame_velodyne;
    points_msg_.header.stamp = current_timestamp_;
    points_msg_.header.seq = velodyne_seq_++;
}

void Player::loadColorDataAt(unsigned int entry)
{
    string full_filename_image02 = (fs::path(dir_image02_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".png")).string();
    string full_filename_image03 = (fs::path(dir_image03_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".png")).string();
    ROS_DEBUG_STREAM ( full_filename_image02 << endl << full_filename_image03 << endl << endl);

    cv_image02_ = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
    cv_image03_ = cv::imread(full_filename_image03, CV_LOAD_IMAGE_UNCHANGED);
    if ( (cv_image02_.data == NULL) || (cv_image03_.data == NULL) ){
        ROS_ERROR_STREAM("Error reading color images (02 & 03)");
        ROS_ERROR_STREAM(full_filename_image02 << endl << full_filename_image03);
        quit_ = true;
        return;
    }

    cv_bridge_img_.encoding = sensor_msgs::image_encodings::BGR8;

    ros_msg02_.header.stamp = ros_cameraInfoMsg_camera02_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image02;
    cv_bridge_img_.header.seq = image02_seq_++;
    cv_bridge_img_.image = cv_image02_;
    cv_bridge_img_.toImageMsg(ros_msg02_);

    ros_msg03_.header.stamp = ros_cameraInfoMsg_camera03_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image03;
    cv_bridge_img_.header.seq = image03_seq_++;
    cv_bridge_img_.image = cv_image03_;
    cv_bridge_img_.toImageMsg(ros_msg03_);
}

void Player::loadGrayscaleDataAt(unsigned int entry)
{
    string full_filename_image00 = (fs::path(dir_image00_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".png")).string();
    string full_filename_image01 = (fs::path(dir_image01_) / fs::path(boost::str(boost::format("%010d") % entry ) + ".png")).string();
    ROS_DEBUG_STREAM ( full_filename_image00 << endl << full_filename_image01 << endl << endl);

    cv_image00_ = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
    cv_image01_ = cv::imread(full_filename_image01, CV_LOAD_IMAGE_UNCHANGED);
    if ( (cv_image00_.data == NULL) || (cv_image01_.data == NULL) ){
        ROS_ERROR_STREAM("Error reading color images (00 & 01)");
        ROS_ERROR_STREAM(full_filename_image00 << endl << full_filename_image01);
        quit_ = true;
        return;
    }

    cv_bridge_img_.encoding = sensor_msgs::image_encodings::MONO8;

    ros_msg00_.header.stamp = ros_cameraInfoMsg_camera00_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image00;
    cv_bridge_img_.header.seq = image00_seq_++;
    cv_bridge_img_.image = cv_image00_;
    cv_bridge_img_.toImageMsg(ros_msg00_);

    ros_msg01_.header.stamp = ros_cameraInfoMsg_camera01_.header.stamp = cv_bridge_img_.header.stamp = current_timestamp_;
    cv_bridge_img_.header.frame_id = options_.frame_image01;
    cv_bridge_img_.header.seq = image01_seq_++;
    cv_bridge_img_.image = cv_image01_;
    cv_bridge_img_.toImageMsg(ros_msg01_);
}


ros::Time Player::getTimestampAt(unsigned int entry)
{
    // sync datasets have the same number timestamp for all elements, 
    // so we just need to read one of them.
    ros::Time stamp;
    if (options_.all_data)
    {
        if(PlayerSupport::getTimestamp(dir_timestamp_image02_, entry, stamp))
        {
            ROS_ERROR_STREAM("Error reading timestamp " << entry << " from image_02");
            quit_ = true;
        }
    }
    else if (options_.color)
    {
        if(PlayerSupport::getTimestamp(dir_timestamp_image02_, entry, stamp))
        {
            ROS_ERROR_STREAM("Error reading timestamp " << entry << " from image_02");
            quit_ = true;
        }
    }
    else if (options_.grayscale)
    {
        if(PlayerSupport::getTimestamp(dir_timestamp_image00_, entry, stamp))
        {
            ROS_ERROR_STREAM("Error reading timestamp " << entry << " from image_00");
            quit_ = true;
        }
    }
    else if (options_.gps || options_.imu || options_.odometry)
    {
        if(PlayerSupport::getTimestamp(dir_timestamp_oxts_, entry, stamp))
        {
            ROS_ERROR_STREAM("Error reading timestamp " << entry << " from oxts");
            quit_ = true;
        }
    }
    else if (options_.velodyne)
    {
        if(PlayerSupport::getTimestamp(dir_timestamp_velodyne_, entry, stamp))
        {
            ROS_ERROR_STREAM("Error reading timestamp " << entry << " from velodyne");
            quit_ = true;
        }
    }

    return stamp;
}

void Player::clockThread()
{
    // Setup publishing rate
    ros::WallRate clock_rate = ros::WallRate(100.0);

    ROS_DEBUG_STREAM("Starting clock publish thread.");
    while(ros::ok() && !quit_)
    {
        if(has_clock_msg_)
        {
            clock_lock_.lock();
            clock_pub_.publish(clock_msg_);
            clock_lock_.unlock();
        }
        clock_rate.sleep();
    }
    ROS_DEBUG_STREAM("Finishing clock publish thread.");
}

void Player::loadStaticTfs()
{
    static_tf_msg_.transforms.clear();

    std_msgs::Header header_support;
    // header_support.stamp = ros::Time::now();
    header_support.stamp = current_timestamp_;

    geometry_msgs::TransformStamped tf_stamped_msg;

    // velodyne
    header_support.frame_id = options_.frame_oxts;
    if(!PlayerSupport::getStaticTransform((fs::path(dir_root_)/fs::path("calib_imu_to_velo.txt")).string(), &tf_stamped_msg, &header_support))
    {
        ROS_WARN_STREAM("Static transform from IMU/VELODYNE could not be loaded");
    }
    else
    {
        tf_stamped_msg.child_frame_id = options_.frame_velodyne;
        tf_stamped_msg.header.seq = static_tf_seq_++;
        static_tf_msg_.transforms.push_back(tf_stamped_msg);
    }

    // image00
    header_support.frame_id = options_.frame_velodyne;
    if(!PlayerSupport::getStaticTransform((fs::path(dir_root_)/fs::path("calib_velo_to_cam.txt")).string(), &tf_stamped_msg, &header_support))
    {
        ROS_WARN_STREAM("Static transform from VELODYNE/CAMERA00 could not be loaded");
    }
    else
    {
        tf_stamped_msg.child_frame_id = options_.frame_image00;
        tf_stamped_msg.header.seq = static_tf_seq_++;
        static_tf_msg_.transforms.push_back(tf_stamped_msg);
    }

    // image01
    header_support.frame_id = options_.frame_image00;
    if(!PlayerSupport::getStaticTransform((fs::path(dir_root_)/fs::path("calib_cam_to_cam.txt")).string(), &tf_stamped_msg, &header_support, "01"))
    {
        ROS_WARN_STREAM("Static transform from CAMERA00/CAMERA01 could not be loaded");
    }
    else
    {
        tf_stamped_msg.child_frame_id = options_.frame_image01;
        tf_stamped_msg.header.seq = static_tf_seq_++;
        static_tf_msg_.transforms.push_back(tf_stamped_msg);
    }

    // image02
    header_support.frame_id = options_.frame_image00;
    if(!PlayerSupport::getStaticTransform((fs::path(dir_root_)/fs::path("calib_cam_to_cam.txt")).string(), &tf_stamped_msg, &header_support, "02"))
    {
        ROS_WARN_STREAM("Static transform from CAMERA00/CAMERA02 could not be loaded");
    }
    else
    {
        tf_stamped_msg.child_frame_id = options_.frame_image02;
        tf_stamped_msg.header.seq = static_tf_seq_++;
        static_tf_msg_.transforms.push_back(tf_stamped_msg);
    }

    // image03
    header_support.frame_id = options_.frame_image00;
    if(!PlayerSupport::getStaticTransform((fs::path(dir_root_)/fs::path("calib_cam_to_cam.txt")).string(), &tf_stamped_msg, &header_support, "03"))
    {
        ROS_WARN_STREAM("Static transform from CAMERA00/CAMERA03 could not be loaded");
    }
    else
    {
        tf_stamped_msg.child_frame_id = options_.frame_image03;
        tf_stamped_msg.header.seq = static_tf_seq_++;
        static_tf_msg_.transforms.push_back(tf_stamped_msg);
    }
}

void Player::processUserInput()
{
    bool charsleftorpaused = true;
    step_ = false;  quit_ = false;
    while (charsleftorpaused && ros::ok())
    {
        switch (readCharFromStdin()){
        case ' ':
            paused_ = !paused_;
            break;
        case 's':
            step_ = true;
            break;
        case 'q':
            quit_ = true;
            break;
        case EOF:
            charsleftorpaused = false;
        }
    }
}

void Player::setupTerminal() {
    if (terminal_modified_)
        return;

#if defined(_MSC_VER)
    input_handle = GetStdHandle(STD_INPUT_HANDLE);
    if (input_handle == INVALID_HANDLE_VALUE)
    {
        std::cout << "Failed to set up standard input handle." << std::endl;
        return;
    }
    if (! GetConsoleMode(input_handle, &stdin_set) )
    {
        std::cout << "Failed to save the console mode." << std::endl;
        return;
    }
    // don't actually need anything but the default, alternatively try this
    //DWORD event_mode = ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT;
    //if (! SetConsoleMode(input_handle, event_mode) )
    //{
    // std::cout << "Failed to set the console mode." << std::endl;
    // return;
    //}
    terminal_modified_ = true;
#else
    const int fd = fileno(stdin);
    termios flags;
    tcgetattr(fd, &orig_flags_);
    flags = orig_flags_;
    flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
    flags.c_lflag &= ~ECHO;        // disable echo
    flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;         // block if waiting for char
    tcsetattr(fd, TCSANOW, &flags);

    FD_ZERO(&stdin_fdset_);
    FD_SET(fd, &stdin_fdset_);
    maxfd_ = fd + 1;
    terminal_modified_ = true;
#endif
}

void Player::restoreTerminal() {
    if (!terminal_modified_)
        return;

#if defined(_MSC_VER)
    SetConsoleMode(input_handle, stdin_set);
#else
    const int fd = fileno(stdin);
    tcsetattr(fd, TCSANOW, &orig_flags_);
#endif
    terminal_modified_ = false;
}

int Player::readCharFromStdin() {
#ifdef __APPLE__
    fd_set testfd;
    FD_COPY(&stdin_fdset_, &testfd);
#elif !defined(_MSC_VER)
    fd_set testfd = stdin_fdset_;
#endif

#if defined(_MSC_VER)
    DWORD events = 0;
    INPUT_RECORD input_record[1];
    DWORD input_size = 1;
    BOOL b = GetNumberOfConsoleInputEvents(input_handle, &events);
    if (b && events > 0)
    {
        b = ReadConsoleInput(input_handle, input_record, input_size, &events);
        if (b)
        {
            for (unsigned int i = 0; i < events; ++i)
            {
                if (input_record[i].EventType & KEY_EVENT & input_record[i].Event.KeyEvent.bKeyDown)
                {
                    CHAR ch = input_record[i].Event.KeyEvent.uChar.AsciiChar;
                    return ch;
                }
            }
        }
    }
    return EOF;
#else
    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 0;
    if (select(maxfd_, &testfd, NULL, NULL, &tv) <= 0)
        return EOF;
    return getc(stdin);
#endif
}

#include <kitti_player/PlayerSupport.h>

#include <ros/ros.h>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <geodesy/utm.h>

#include <fstream>

using namespace std;

namespace fs = boost::filesystem;

namespace PlayerSupport
{

int getCamCalibration(string dir_root, string camera_name, double* K,std::vector<double> & D,double *R,double* P){
/*
 *   from: http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip
 *   calib_cam_to_cam.txt: Camera-to-camera calibration
 *   --------------------------------------------------
 *
 *     - S_xx: 1x2 size of image xx before rectification
 *     - K_xx: 3x3 calibration matrix of camera xx before rectification
 *     - D_xx: 1x5 distortion vector of camera xx before rectification
 *     - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
 *     - T_xx: 3x1 translation vector of camera xx (extrinsic)
 *     - S_rect_xx: 1x2 size of image xx after rectification
 *     - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
 *     - P_rect_xx: 3x4 projection matrix after rectification
*/

    //    double K[9];         // Calibration Matrix
    //    double D[5];         // Distortion Coefficients
    //    double R[9];         // Rectification Matrix
    //    double P[12];        // Projection Matrix Rectified (u,v,w) = P * R * (x,y,z,q)

    string calib_cam_to_cam=(fs::path(dir_root) / fs::path("calib_cam_to_cam.txt")).string();
    ifstream file_c2c(calib_cam_to_cam.c_str());
    if (!file_c2c.is_open())
        return false;

    ROS_INFO_STREAM("Reading camera" << camera_name.c_str() << " calibration from " << calib_cam_to_cam.c_str());

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";
    char index=0;
    tokenizer::iterator token_iterator;

    while (getline(file_c2c,line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line,sep);

        // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.
        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("K_")+camera_name+string(":"))).c_str())==0) //Calibration Matrix
        {
            index=0; //should be 9 at the end
            ROS_DEBUG_STREAM("K_" << camera_name.c_str());
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                K[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

        // EXPERIMENTAL: use with unrectified images

        //        token_iterator=tok.begin();
        //        if (strcmp((*token_iterator).c_str(),((string)(string("D_")+camera_name+string(":"))).c_str())==0) //Distortion Coefficients
        //        {
        //            index=0; //should be 5 at the end
        //            ROS_DEBUG_STREAM("D_" << camera_name);
        //            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
        //            {
        ////                std::cout << *token_iterator << '\n';
        //                D[index++]=boost::lexical_cast<double>(*token_iterator);
        //            }
        //        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("R_")+camera_name+string(":"))).c_str())==0) //Rectification Matrix
        {
            index=0; //should be 12 at the end
            ROS_DEBUG_STREAM("R_" << camera_name.c_str());
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                R[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("P_rect_")+camera_name+string(":"))).c_str())==0) //Projection Matrix Rectified
        {
            index=0; //should be 12 at the end
            ROS_DEBUG_STREAM("P_rect_" << camera_name.c_str());
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                P[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

    }
    ROS_INFO_STREAM("... ok");
    return true;
}

int getStaticTransform(string calib_filename, geometry_msgs::TransformStamped *ros_msgTf, std_msgs::Header *header, string camera_name)
{
    ifstream calib_file(calib_filename.c_str());
    if (!calib_file.is_open())
        return false;

    ROS_DEBUG_STREAM("Reading transformation" << (camera_name.empty()? string(""): string(" for camera ")+camera_name) << " from " << calib_filename.c_str());

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";
    char index=0;
    tokenizer::iterator token_iterator;

    tf2::Transform t;

    while (getline(calib_file,line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line,sep);

        // Move the iterator at the beginning of the tokenize vector and check for T/R matrices.
        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("T")+(camera_name.empty()?string(""):string("_")+camera_name)+string(":"))).c_str())==0) //Calibration Matrix
        {
            index=0; //should be 3 at the end
            ROS_DEBUG_STREAM("T" << (camera_name.empty()?string(""):string("_")).c_str() << camera_name.c_str());
            double T[3];
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                // std::cout << *token_iterator << '\n';
                T[index++]=boost::lexical_cast<double>(*token_iterator);
            }
            t.setOrigin(tf2::Vector3(T[0],T[1],T[2]));
        }

        // EXPERIMENTAL: use with unrectified images

        //        token_iterator=tok.begin();
        //        if (strcmp((*token_iterator).c_str(),((string)(string("D_")+camera_name+string(":"))).c_str())==0) //Distortion Coefficients
        //        {
        //            index=0; //should be 5 at the end
        //            ROS_DEBUG_STREAM("D_" << camera_name);
        //            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
        //            {
        ////                std::cout << *token_iterator << '\n';
        //                D[index++]=boost::lexical_cast<double>(*token_iterator);
        //            }
        //        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("R")+(camera_name.empty()?string(""):string("_")+camera_name)+string(":"))).c_str())==0) //Rectification Matrix
        {
            index=0; //should be 9 at the end
            ROS_DEBUG_STREAM("R" << (camera_name.empty()?string(""):string("_")).c_str() << camera_name.c_str());
            double R[9];
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                // std::cout << *token_iterator << '\n';
                R[index++]=boost::lexical_cast<double>(*token_iterator);
            }
            tf2::Matrix3x3 mat(R[0],R[1],R[2],
                              R[3],R[4],R[5],
                              R[6],R[7],R[8]);
            tf2::Quaternion quat;  mat.getRotation(quat);
            t.setRotation(quat);
        }

        // token_iterator=tok.begin();
        // if (strcmp((*token_iterator).c_str(),((string)(string("P_rect_")+camera_name+string(":"))).c_str())==0) //Projection Matrix Rectified
        // {
        //     index=0; //should be 12 at the end
        //     ROS_DEBUG_STREAM("P_rect_" << camera_name);
        //     for (token_iterator++; token_iterator != tok.end(); token_iterator++)
        //     {
        //         //std::cout << *token_iterator << '\n';
        //         P[index++]=boost::lexical_cast<double>(*token_iterator);
        //     }
        // }

    }

    // fill up tf message
    ros_msgTf->header.frame_id = header->frame_id;
    ros_msgTf->header.stamp = header->stamp;

    // Invert transform to match the tf tree structure: oxts -> velodyne -> cam00 -> cam{01,02,03}
    tf2::convert(t.inverse(),ros_msgTf->transform);

    ROS_DEBUG_STREAM("... ok");
    return true;
}

bool getTimestamp(string dir_timestamp, unsigned int entry_number, ros::Time &stamp)
{
	string str_support = (fs::path(dir_timestamp) / fs::path("timestamps.txt")).string();
	ifstream timestamps(str_support.c_str());
	if (!timestamps.is_open())
	{
	    ROS_ERROR_STREAM("Fail to open " << str_support.c_str());
	    return true;
	}
	timestamps.seekg(30*entry_number);
	getline(timestamps,str_support);
	stamp = parseTime(str_support);
	return false;
}

ros::Time parseTime(string timestamp)
{
    //Epoch time conversion
    //http://www.epochconverter.com/programming/functions-c.php

    ros::Time stamp;

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0,4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5,2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8,2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11,2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14,2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17,2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    stamp.sec  = timeSinceEpoch;
    stamp.nsec = boost::lexical_cast<int>(timestamp.substr(20, string::npos));

    return stamp;
}

int getGPS(string filename, sensor_msgs::NavSatFix *ros_msgGpsFix, std_msgs::Header *header)
{
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open()){
        ROS_ERROR_STREAM("Fail to open " << filename.c_str());
        return 0;
    }

    ROS_DEBUG_STREAM("Reading GPS data from oxts file: " << filename.c_str() );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";

    getline(file_oxts,line);
    tokenizer tok(line,sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgGpsFix->header.frame_id = header->frame_id;
    ros_msgGpsFix->header.stamp = header->stamp;

    ros_msgGpsFix->latitude  = boost::lexical_cast<double>(s[0]);
    ros_msgGpsFix->longitude = boost::lexical_cast<double>(s[1]);
    ros_msgGpsFix->altitude  = boost::lexical_cast<double>(s[2]);

    ros_msgGpsFix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    for (int i=0;i<9;i++)
        ros_msgGpsFix->position_covariance[i] = 0.0f;

    ros_msgGpsFix->position_covariance[0] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[4] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[8] = boost::lexical_cast<double>(s[23]);

    ros_msgGpsFix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    ros_msgGpsFix->status.status  = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

    return 1;
}

int getIMU(string filename, sensor_msgs::Imu *ros_msgImu, std_msgs::Header *header)
{
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open())
    {
        ROS_ERROR_STREAM("Fail to open " << filename.c_str());
        return 0;
    }

    ROS_DEBUG_STREAM("Reading IMU data from oxts file: " << filename.c_str() );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";

    getline(file_oxts,line);
    tokenizer tok(line,sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgImu->header.frame_id = header->frame_id;
    ros_msgImu->header.stamp = header->stamp;

    //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
    //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
    //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
    ros_msgImu->linear_acceleration.x = boost::lexical_cast<double>(s[11]);
    ros_msgImu->linear_acceleration.y = boost::lexical_cast<double>(s[12]);
    ros_msgImu->linear_acceleration.z = boost::lexical_cast<double>(s[13]);

    //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
    //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
    //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
    ros_msgImu->angular_velocity.x = boost::lexical_cast<double>(s[8]);
    ros_msgImu->angular_velocity.y = boost::lexical_cast<double>(s[9]);
    ros_msgImu->angular_velocity.z = boost::lexical_cast<double>(s[10]);

    //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
    tf2::Quaternion q; q.setRPY( boost::lexical_cast<double>(s[3]),
                                 boost::lexical_cast<double>(s[4]),
                                 boost::lexical_cast<double>(s[5])
                                 );
    ros_msgImu->orientation.x = q.getX();
    ros_msgImu->orientation.y = q.getY();
    ros_msgImu->orientation.z = q.getZ();
    ros_msgImu->orientation.w = q.getW();

    return 1;
}

int getOdomTf(string filename, tf2::Transform *tf2_tf)
{
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open())
    {
        ROS_ERROR_STREAM("Fail to open " << filename.c_str());
        return 0;
    }

    ROS_DEBUG_STREAM("Reading odom data from oxts file: " << filename.c_str() );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";

    getline(file_oxts,line);
    tokenizer tok(line,sep);
    vector<string> s(tok.begin(), tok.end());

    //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
    tf2::Quaternion q; q.setRPY(   boost::lexical_cast<double>(s[3]),
                                   boost::lexical_cast<double>(s[4]),
                                   boost::lexical_cast<double>(s[5])
                                   );
    //    - lat:     latitude of the oxts-unit (deg)
    //    - lon:     longitude of the oxts-unit (deg)
    //    - alt:     altitude of the oxts-unit (m)
    geodesy::UTMPoint utmP(geodesy::toMsg(   boost::lexical_cast<double>(s[0]),
                                             boost::lexical_cast<double>(s[1]),
                                             boost::lexical_cast<double>(s[2])
                                             ));

    // set a transform from utm and orientation
    tf2_tf->setRotation(q);
    tf2_tf->setOrigin(tf2::Vector3(utmP.easting, utmP.northing, utmP.altitude));
}


} // namespace PlayerSupport
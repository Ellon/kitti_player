#include <kitti_player_visual_odometry/PlayerSupportVisualOdometry.h>





namespace fs = boost::filesystem;

namespace PlayerSupportVisualOdometry
{

int getCamCalibration(string dir_calib, string camera_name, double* K,std::vector<double> & D,double *R,double* P){
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

    string calib=(fs::path(dir_calib) / "calib.txt").string();
    ifstream file_c(calib.c_str());
    if (!file_c.is_open())
        return false;

    ROS_INFO_STREAM("Reading camera" << camera_name << " calibration from " << calib);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    string line="";
    int index=0;
    tokenizer::iterator token_iterator;

    while (getline(file_c,line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line,sep);

        // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.
        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("P")+camera_name+string(":"))).c_str())==0) //Calibration Matrix
        {
            index=0; //should be 12 at the end
            ROS_DEBUG_STREAM("P" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {

                //std::cout << *token_iterator << '\n';
                P[index++]=boost::lexical_cast<double>(*token_iterator);

            }

        }



    }

    //We don't have R in calib.txt so => identity
    for(int i=0;i<9;i++){
        if(i==0 || i==4 || i==8)
            R[i]=1.0;
        else
            R[i]=0.0;
    }

    //We don't have K in calib.txt so => Rotation Matrix of P
    for(int i=0;i<9;i++){
        K[i]=P[i+(i/3)];
    }

    //We don't have D in calib.txt so => all parameters to 0
    for(int i=0;i<5;i++)
        D.push_back(0);

    ROS_INFO_STREAM("... ok");
    return true;
}



bool getTimestamp(string dir_timestamp,vector<ros::Time> &times)
{
    string str_support;
    str_support = (fs::path(dir_timestamp) / "times.txt").string();
    ifstream timestamps(str_support.c_str());
    double stamp;
    if (!timestamps.is_open())
    {
        ROS_ERROR_STREAM("Fail to open times.txt :" << str_support.c_str());
        return false;
    }
    while(timestamps >> stamp){
        ros::Time t(stamp);
        times.push_back(t);
    }

    timestamps.close();
    return true;
}



/**
 * @brief get_poses
 * @param dir_root
 * @param rotations      - Vector of rotation matrix(3,3)
 * @param pts            - Vector of translation vector
 * @return 1: file found, 0: file not found
 */
bool get_poses(string dir_root,string sequence,vector<Eigen::Matrix3d> &rotations,vector<geometry_msgs::Point> &pts){

    string poseFile=sequence+".txt";

    string infile =(fs::path(dir_root) / fs::path("poses") /poseFile).string();
    fstream input(infile.c_str());
    string line;
    if(!input.good())
    {
        ROS_ERROR_STREAM ( "Could not read file: " << infile.c_str());
        return false;
    }
    //Transform each Isometric matrix stored in the file into a rotation Matrix and a translation vector
    while (getline (input,line))
    {
        stringstream ss(line);
        Eigen::Matrix3d rotation;
        geometry_msgs::Point pt;
        string entry;
        int i=0;

        while (ss >> entry){
            double value=stod(entry);

            switch(i){
                case 3: pt.x=value;
                        break;
                case 7: pt.y=value;
                        break;
                case 11: pt.z=value;
                        break;
                default:rotation(i/4,i%4)= value;
                        break;
            }
            i++;
        }

        rotations.push_back(rotation);
        pts.push_back(pt);
    }
    input.close();
    return true;
}



} // namespace PlayerSupportVisualOdometry

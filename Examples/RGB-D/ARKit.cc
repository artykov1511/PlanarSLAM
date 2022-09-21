#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
#include<Mesh.h>
#include<MapPlane.h>
#include <opencv2/core/eigen.hpp> 

using namespace std;

void LoadTimestamps(const string &strSequencePath, vector<double> &vTimestamps);
void LoadCamPose(const string& , cv::Mat&);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./Planar_SLAM path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<double> vTimestamps;
    string strSequencePath = string(argv[3]);
    LoadTimestamps(strSequencePath, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vTimestamps.size();
   

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    Planar_SLAM::System SLAM(argv[1], argv[2], Planar_SLAM::System::RGBD, true);
    Planar_SLAM::Config::SetParameterFile(argv[2]);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
	
    // Feed each image to the system
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        cout<<"PlanarSLAM Printer: This is the "<<ni<<"th image"<<endl;
        // a RGB-D pair
        imRGB = cv::imread(string(argv[3])+ "color_480x640/" + to_string(ni) + "_color" + ".jpg" ,CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+ "depth_480x640/" + to_string(ni) + ".png",CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        cv::Mat pose;
        string pose_pth = string(argv[3]) + "pose/" + to_string(ni) + ".txt";
        LoadCamPose(pose_pth, pose);

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << to_string(ni) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe, pose);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << "Track time for frame " + to_string(ni) + " is " + to_string(ttrack) << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e5);
    }
    char bStop;

//    cerr << "PlanarSLAM Printer: Please type 'x', if you want to shutdown windows." << endl;
//
//    while (bStop != 'x'){
//        bStop = getchar();
//    }
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "PlanarSLAM Printer:" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveMesh("MeshMap.ply");

    return 0;
}

void LoadTimestamps(const string &strSequencePath, vector<double> &vTimestamps)
{
    ifstream fSequence;
    string timesFile = strSequencePath + "times.txt"; 
    fSequence.open(timesFile.c_str());
    while(!fSequence.eof())
    {
        string s;
        getline(fSequence,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
            
        }
    }
}


void LoadCamPose(const string& strPoseFile, cv::Mat& pose)
{
    ifstream fCamPose;
    fCamPose.open(strPoseFile.c_str());
    vector<float> v;
    while(!fCamPose.eof())
    {
        string s;
        getline(fCamPose, s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            float p;
            ss >> p;

            v.push_back(p);
        }
    }

    Eigen::Matrix<float, 4, 4> pose_eigen = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(v.data(), 4, 4); // camera to world frame
    pose_eigen = pose_eigen.inverse().eval(); // world to camera
    // Convert eigen to cv::Mat
    cv::eigen2cv(pose_eigen, pose);
    
    cout << "Pose: " << pose << endl;
}
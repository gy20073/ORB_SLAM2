/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
// for checking whether file exists
#include <sys/stat.h>
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include <dirent.h>
#include<vector>
#include <string>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./mono_nexar path_to_vocabulary path_to_settings path_to_sequence use_viewer mask_relative" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, stoi(argv[4]) );

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);

        std::string mask_relative = argv[5];
        cv::Mat mask;
        if(mask_relative != "none"){
            // reading the mask in as well
            std::string image_path = vstrImageFilenames[ni];
            // from the relative path to mask path
            auto splits = split(image_path, '/');
            while(splits.back().empty())
                splits.pop_back();

            std::string mask_path;
            for(int i=0; i<splits.size()-1; ++i)
                mask_path += splits[i] + "/";
            auto file_splits = split(splits.back(), '.');
            mask_path += mask_relative + "/" + file_splits[0] + ".png";

             mask= cv::imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);
            // end of reading the mask
        }


        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe, mask);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

inline bool exists_test3 (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

vector<string> get_all_files(string root_dir){
    vector<string> ans;

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (root_dir.c_str())) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
        ans.push_back(root_dir + "/" + ent->d_name);
      }
      closedir (dir);
    } else {
        // bad dir
        printf("bad dir\n");
    }
    sort(ans.begin(), ans.end());
    return ans;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    const double interval = 1.0 / 30;

    /*
    // loop over all images
    int image_id = 0;
    while (1){
    	image_id += 1;
    	
    	stringstream ss;
        ss << setfill('0') << setw(4) << image_id;
        string image_name = strPathToSequence + "/" + ss.str() + ".jpg";
        if (exists_test3(image_name)){
        	vstrImageFilenames.push_back(image_name);
        }else{
        	break;
        }
    }
    */
    vector<string> all = get_all_files(strPathToSequence);

    for(int i=0; i<all.size(); ++i){
        string name = all[i];
        transform(name.begin(), name.end(), name.begin(), ::tolower);
        int len = name.size();
        if(name.substr(len-3)=="png" || name.substr(len-3)=="jpg" || name.substr(len-4)=="jpeg"){
            vstrImageFilenames.push_back(all[i]);
        }
    }
    int image_id = vstrImageFilenames.size();
    
    for(int i=0; i<image_id; ++i){
    	vTimestamps.push_back(i*interval);
    }
}

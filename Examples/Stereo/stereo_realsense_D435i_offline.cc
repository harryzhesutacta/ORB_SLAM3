/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <dirent.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if (argc < 5 || argc > 6)
    {
        cerr << endl
             << "Usage: ./stereo_realsense_D435i_offline path_to_vocabulary path_to_settings path_to_left_images_folder path_to_right_images_folder (trajectory_file_name)" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;

    string pathLeft = string(argv[3]);
    string pathRight = string(argv[4]);
    
    string trajectory_file_name = "CameraTrajectory.txt";
    if (argc == 6) {
        trajectory_file_name = string(argv[5]);
    }

    LoadImages(pathLeft, pathRight, vstrImageLeft, vstrImageRight, vTimestamps);

    if (vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images found in provided paths." << endl;
        return 1;
    }

    if (vstrImageLeft.size() != vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }

    const int nImages = vstrImageLeft.size();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imLeft, imRight;

    // Main loop
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

        if (imLeft.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << vstrImageLeft[ni] << endl;
            return 1;
        }

        if (imRight.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << vstrImageRight[ni] << endl;
            return 1;
        }

        double tframe = vTimestamps[ni];

        if (imageScale != 1.f)
        {
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI(trajectory_file_name);
    cout << "Trajectory saved to: " << trajectory_file_name << endl;

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    // Structure to hold image data for sorting
    struct ImageData {
        double timestamp;
        string leftPath;
        string rightPath;
        int counter;
    };
    
    vector<ImageData> imageData;
    
    // Read left images directory
    DIR* dir = opendir(strPathLeft.c_str());
    if (dir == nullptr) {
        cerr << "Cannot open left images directory: " << strPathLeft << endl;
        return;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        string filename = entry->d_name;
        
        // Skip . and .. directories
        if (filename == "." || filename == "..") continue;
        
        // Check if it's a PNG file
        if (filename.find(".png") == string::npos) continue;
        
        // Extract timestamp and counter from filename: timestamp_counter.png
        size_t underscorePos = filename.find('_');
        size_t dotPos = filename.find(".png");
        
        if (underscorePos != string::npos && dotPos != string::npos) {
            string timestampStr = filename.substr(0, underscorePos);
            string counterStr = filename.substr(underscorePos + 1, dotPos - underscorePos - 1);
            
            try {
                double timestamp = stod(timestampStr);
                int counter = stoi(counterStr);
                
                // Create corresponding right image path
                string rightImagePath = strPathRight + "/" + filename;
                string leftImagePath = strPathLeft + "/" + filename;
                
                // Check if right image exists
                ifstream rightFile(rightImagePath);
                if (rightFile.good()) {
                    ImageData data;
                    data.timestamp = timestamp;
                    data.leftPath = leftImagePath;
                    data.rightPath = rightImagePath;
                    data.counter = counter;
                    imageData.push_back(data);
                }
                rightFile.close();
            } catch (const exception& e) {
                cerr << "Error parsing filename: " << filename << " - " << e.what() << endl;
                continue;
            }
        }
    }
    closedir(dir);
    
    // Sort by timestamp to ensure correct order
    sort(imageData.begin(), imageData.end(), 
         [](const ImageData& a, const ImageData& b) {
             return a.timestamp < b.timestamp;
         });
    
    // Fill output vectors
    vTimeStamps.reserve(imageData.size());
    vstrImageLeft.reserve(imageData.size());
    vstrImageRight.reserve(imageData.size());
    
    for (const auto& data : imageData) {
        vTimeStamps.push_back(data.timestamp);
        vstrImageLeft.push_back(data.leftPath);
        vstrImageRight.push_back(data.rightPath);
    }
    
    cout << "Loaded " << imageData.size() << " stereo image pairs" << endl;
    if (!imageData.empty()) {
        cout << "Timestamp range: " << imageData.front().timestamp 
             << " to " << imageData.back().timestamp << endl;
    }
}

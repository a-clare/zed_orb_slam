///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**********************************************************************************
 ** This sample demonstrates how to capture a live 3D reconstruction of a scene  **
 ** as a fused point cloud and display the result in an OpenGL window.           **
 **********************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"

#include <opencv2/opencv.hpp>

#include <System.h>

// Using std and sl namespaces
using namespace std;
using namespace sl;

void parse_args(int argc, char **argv,InitParameters& param);

void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");

int main(int argc, char **argv) {

    if (argc < 4) {
        std::cout << "Not enough input arguments" << std::endl;
        std::cout << "Usage: " << std::endl;
        std::cout << "\tzed_orb_slam /path/to/svo /path/to/orb/vocab /path/to/orb/config" << std::endl;
        return 1;
    }

    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed    
    // parse_args should ignore the ORB slam arguments (only looks for SVO)
    parse_args(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);

    if (returned_state != ERROR_CODE::SUCCESS) {// Quit if an error occurred
        print("Open Camera", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    auto camera_infos = zed.getCameraInformation();
    std::cout << "ZED SVO Info: \n" <<
        "\tWidth: " << camera_infos.camera_configuration.resolution.width <<
        "\tHeight: " << camera_infos.camera_configuration.resolution.height <<
        "\tFPS: " << camera_infos.camera_configuration.fps << std::endl;

    std::cout << "argc: " << argc << std::endl;
    std::cout << "argv[2]: " << argv[2] << std::endl;
    std::cout << "argv[3]: " << argv[3] << std::endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // vocab should be arg[2] and settings argv[3], this is different than standard ORB SLAM
    ORB_SLAM3::System SLAM(argv[2],argv[3],ORB_SLAM3::System::STEREO,true);

    // Setup and start positional tracking
    Pose pose;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.enable_area_memory = false;
    returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positional tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    // Set spatial mapping parameters
    SpatialMappingParameters spatial_mapping_parameters;
    // Request a Point Cloud
    spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
    // Set mapping range, it will set the resolution accordingly (a higher range, a lower resolution)
    spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RANGE::LONG);
    // Request partial updates only (only the lastest updated chunks need to be re-draw)
    spatial_mapping_parameters.use_chunk_only = true;
    // Start the spatial mapping
    zed.enableSpatialMapping(spatial_mapping_parameters);

    // Timestamp of the last fused point cloud requested
    chrono::high_resolution_clock::time_point ts_last;

    // Setup runtime parameters
    RuntimeParameters runtime_parameters;
    // Use low depth confidence avoid introducing noise in the constructed model
    runtime_parameters.confidence_threshold = 50;

    auto camera_resolution = camera_infos.camera_configuration.resolution;

    // Allocate the memory/images for ZED and ORB (OpenCV)
    sl::Mat left_zed_image(camera_resolution, sl::MAT_TYPE::U8_C4);
    sl::Mat right_zed_image(camera_resolution, sl::MAT_TYPE::U8_C4);

    // If you download or have used zed-examples from github, there is an example
    // called svo_recording/export. There is a function in there called slMat2cvMat,
    // this is where the logic/code to convert ZED -> OpenCV was taken from
    cv::Mat left_orb_image(
            left_zed_image.getHeight(),
            left_zed_image.getWidth(),
            CV_8UC4,
            left_zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::Mat right_orb_image(
            right_zed_image.getHeight(),
            right_zed_image.getWidth(),
            CV_8UC4,
            right_zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));

    // Going to keep procesing until we reach the end of the SVO file
    bool end_of_svo = false;

    // Simple counter for number of images retreived. Used for debugging
    std::uint32_t image_cnt = 0;

    while (!end_of_svo) {
        // Try and grab a new image
        sl::ERROR_CODE error = zed.grab(runtime_parameters);

        if (error == ERROR_CODE::SUCCESS) {
            // Retrieve the images image
            zed.retrieveImage(left_zed_image, VIEW::LEFT);
            zed.retrieveImage(right_zed_image, VIEW::RIGHT);
            image_cnt += 1;
            std::cout << "Num images: " << image_cnt << std::endl;
        }
        else if (error == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
            std::cout << "Reached end of SVO" << std::endl;
            std::cout << "Num images: " << image_cnt << std::endl;
            end_of_svo = true;
        }
        else {
            std::cout << "Unexpected SVO/ZED error " << error << std::endl;
            end_of_svo = true;
        }
    }

    // Free allocated memory before closing the camera
    left_zed_image.free();
    right_zed_image.free();
    // Close the ZED
    zed.close();

    return 0;
}

void parse_args(int argc, char **argv,InitParameters& param)
{
    if (argc > 1 && string(argv[1]).find(".svo")!=string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        param.svo_real_time_mode=true;

        cout<<"[Sample] Using SVO File input: "<<argv[1]<<endl;
    } else if (argc > 1 && string(argv[1]).find(".svo")==string::npos) {
        string arg = string(argv[1]);
        unsigned int a,b,c,d,port;
        if (sscanf(arg.c_str(),"%u.%u.%u.%u:%d", &a, &b, &c, &d,&port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a)+"."+to_string(b)+"."+to_string(c)+"."+to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()),port);
            cout<<"[Sample] Using Stream input, IP : "<<ip_adress<<", port : "<<port<<endl;
        }
        else  if (sscanf(arg.c_str(),"%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(String(argv[1]));
            cout<<"[Sample] Using Stream input, IP : "<<argv[1]<<endl;
        }
        else if (arg.find("HD2K")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout<<"[Sample] Using Camera in resolution HD2K"<<endl;
        } else if (arg.find("HD1080")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout<<"[Sample] Using Camera in resolution HD1080"<<endl;
        } else if (arg.find("HD720")!=string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout<<"[Sample] Using Camera in resolution HD720"<<endl;
        } else if (arg.find("VGA")!=string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout<<"[Sample] Using Camera in resolution VGA"<<endl;
        }
    } else {
        // Default
    }
}

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
    cout <<"[Sample]";
    if (err_code != sl::ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout<<" ";
    cout << msg_prefix << " ";
    if (err_code != sl::ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[]) try
{
    float maxRange = 3.0f;
    if (argc == 2) {
        maxRange = std::stof(argv[1]);
    }

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    float _x;
    float _y;
    float _z;
    float _azi;
    size_t roiPtCnt = 0;
    size_t t = 0;
    while(1)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto vertices = points.get_vertices();
        _x = 0.0f;
        _y = 0.0f;
        _z = 0.0f;
        _azi = 0.0f;
        roiPtCnt = 0;
        for (int i = 0; i < points.size(); ++i)
        {
            if (vertices[i].z                            // the object is in front of realsense
                && vertices[i].y >=-0.1f                 // the object is above realsense horizon
                && vertices[i].y <  0.1f                 // the object occupies 0-0.1m reason in front of realsense
                && vertices[i].z * vertices[i].z         // the euclidean distance is within 1m
                 + vertices[i].x * vertices[i].x < maxRange * maxRange
                )
            {
                _z += vertices[i].z;
                _x += vertices[i].x;
                ++roiPtCnt;
            }
        }
        printf("itr time: %d", (int)t++);
        if (roiPtCnt) {
            _z /= roiPtCnt; // get mean value of _z
            _x /= roiPtCnt; // get mean value of _x
            _azi = std::atan2f(_x, _z) * 180 / 3.141592654f;
            printf("    x: %.2f    z: %.2f    azimuth: %.1f", _x, _z, _azi);
        }
        printf("\n");
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

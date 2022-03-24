#pragma once
#include <ros/ros.h>

#include <map>
#include <vector>
#include <string>

namespace autol_pointcloud
{
    struct LaserCorrection
    {
        /** parameters in db.xml */
        float rot_correction;
        float vert_correction;
        float dist_correction;
        bool two_pt_correction_available;
        float dist_correction_x;
        float dist_correction_y;
        float vert_offset_correction;
        float horiz_offset_correction;
        int max_intensity;
        int min_intensity;
        float focal_distance;
        float focal_slope;

        /** cached values calculated when the calibration file is read */
        float cos_rot_correction;              ///< cosine of rot_correction
        float sin_rot_correction;              ///< sine of rot_correction
        float cos_vert_correction;             ///< cosine of vert_correction
        float sin_vert_correction;             ///< sine of vert_correction

        int laser_ring;                        ///< ring number for this laser
    };

    struct SlamOffset
    {
        float roll;
        float pitch;
        float yaw;
        float x_offset;
        float y_offset;
        float z_offset;
    };


    class Calibration
    {
    public:
        // float distance_resolution_m;
        // std::map<int, LaserCorrection> laser_corrections_map;
        // std::vector<LaserCorrection> laser_corrections;
        // int num_lasers;
        // bool initialized;
        // bool ros_info;

        bool initialized;
        int node_type_;

        int num_lidars;
        std::vector<SlamOffset> lidar_slamoffset_corrections;
    public:
        Calibration();
        //void Read(std::string file);
        void ReadSlamOffset(std::string file);


    private:

    };
}

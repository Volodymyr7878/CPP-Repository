#include <iostream>
#include <fstream>
#include <cmath>

#define ENABLE_LOG 0
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

const long ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3f;
const float wheelbase_m  = 1.0f;


bool first_line = true;
float prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
float x = 0, y = 0, theta = 0;


int main(int argc, char** argv) {
    // The program expects exactly one argument: a path to telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    // Open the input file
    std::ifstream Infile(argv[1]);
    if(!Infile.is_open())
    {
        DEBUG("Error: Could not open the input file");
        return 1;
    }


    long timestamp_ms, fl_ticks, fr_ticks, bl_ticks, br_ticks;

    while(Infile >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks)
    {

        float d_fl, d_fr, d_bl, d_br;
        float d_left, d_right;
        float distance_per_tick, dL, dR;
        float d, dtheta;

        if (first_line) 
        {
            first_line = false;
            prev_fl = fl_ticks;
            prev_fr = fr_ticks;
            prev_bl = bl_ticks;
            prev_br = br_ticks;
            continue;  // Skip first row, it's all zeros
        }

        //Step 1: Compute the change in ticks for each wheel since the last sample
        d_fl = fl_ticks - prev_fl;

        d_fr = fr_ticks - prev_fr;

        d_bl = bl_ticks - prev_bl;

        d_br = br_ticks - prev_br;

        LOG("d_fl: " << d_fl << ", d_fr: " << d_fr << ", d_bl: " << d_bl << ", d_br: " << d_br);


        //Step 2: Convert ticks to distance traveled by each wheel
        d_left = (d_fl + d_bl) / 2;

        d_right = (d_fr + d_br) / 2;

        LOG("d_left: " << d_left << ", d_right: " << d_right);

        //Step 3: Compute the distance traveled by the robot and the change in orientation
        distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

        dL = d_left * distance_per_tick;

        dR = d_right * distance_per_tick;

        LOG("dL: " << dL << ", dR: " << dR);

        //Step 4: Update the robot's pose (x, y, theta)
        d = (dL + dR) / 2;              // пройдена вiдстань центру

        dtheta = (dR - dL) / wheelbase_m;    // змiна орiєнтацiї

        LOG("d: " << d << ", dtheta: " << dtheta);

        //Step 4: Update the robot's pose (x, y, theta)
        x += d * cos(theta + dtheta / 2);

        y += d * sin(theta + dtheta / 2);

        theta += dtheta;

        LOG("Updated pose: x: " << x << ", y: " << y << ", theta: " << theta);

        std::cout << timestamp_ms << " " << x << " " << y << " " << theta << std::endl;
        // Output the timestamp and updated values of x, y, and theta to a text file
        //std::ofstream Outfile("odometry_output.txt");
        //Outfile << timestamp_ms << " " << x << " " << y << " " << theta << std::endl;

        prev_fl = fl_ticks;
        prev_fr = fr_ticks;
        prev_bl = bl_ticks;
        prev_br = br_ticks;


    }

    Infile.close(); 

    return 0;
}

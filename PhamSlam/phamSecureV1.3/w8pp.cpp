/*
 *  RPLIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// ******************************* //
#include "phamslam_global.h"
#include "phamslam_file.h"
#include <algorithm>
#include <vector>
#include <tuple>
// ******************************* //

#include "Include/rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>



//For CG1112 Starts
const int DEF_MARGIN = 20;
const int DISP_RING_ABS_DIST  = 100;
const float DISP_FULL_DIST    = 16000;
const float DISP_DEFAULT_DIST = 8000;
const float DISP_MIN_DIST     = 1000;
const float PI   = (float)3.14159265;
//For CG1112 Ends

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

// *********************** //
// Pham
// typedef std::tuple<float,float,float,float> data_node_T;
// std::vector<data_node_T> data_list;
// uint32_t data_list_size;
float dist_0_degree;
float dist_90_degree;
float dist_180_degree;
float dist_270_degree;
float previous_dist_0_degree;
float previous_dist_180_degree;
float moving_dist_pixel;
float min_dist;
float min_angle;
float previous_min_angle;
float command_angle = 0;
float comparing_angle;
float min_angle_diff;
std::vector<float> min_angle_set;
bool has_been_scanned = 0;
// *********************** //

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR 2D Plot for RPLIDAR.\n"
            "Version: " RPLIDAR_SDK_VERSION "\n"
            "Usage:\n"
            "%s <com port> [baudrate]\n"
            "The default baudrate is 115200. Please refer to the datasheet for details.\n"
            , argv[0]);
}

void undo_lidar_var() {
    FILE* backupvarFile = fopen("backup_var.dat","r");
    fscanf(backupvarFile,"%f %f %f %d",&previous_min_angle,&previous_dist_0_degree,&previous_dist_180_degree,&has_moved);
    fclose(backupvarFile);
}

void update_lidar_backup_var() {
    FILE* backupvarFile = fopen("backup_var.dat","w");
    fprintf(backupvarFile,"%f %f %d",previous_min_angle,previous_dist_0_degree,previous_dist_180_degree,has_moved);
    fclose(backupvarFile);
}

void updateBetterMap(){
    FILE* tempMapFile;      // "lidar_temp_reading.dat"
    FILE* betterMapFile;    // "lidar_better_map_reading.dat"
    // FILE* previousMapFile;  // "lidar_back_up_map_reading.dat"
    int preX,preY;
    float newX,newY;
    int brightness;
    if (allow_scanning == 0)
        return;
    if (has_been_scanned==0) {
        tempMapFile = fopen("lidar_temp_reading.dat","r");
        betterMapFile = fopen("lidar_better_map_reading.dat","a");
        while(fscanf(tempMapFile,"%d %d %d",&preX,&preY,&brightness)>=0) { 
            fprintf(betterMapFile, "%d\t%d\t%d\n",preX,preY,brightness);
        }
        fclose(tempMapFile);
        fclose(betterMapFile);
    } else {
        float rad = (-1) * (delta_angle*PI/180.0);
        // previousMapFile = fopen("lidar_back_up_map_reading.dat","w");
        tempMapFile = fopen("lidar_temp_reading.dat","a");
        betterMapFile = fopen("lidar_better_map_reading.dat","r");
        while(fscanf(betterMapFile,"%d %d %d",&preX,&preY,&brightness)>=0){
            // fprintf(previousMapFile,"%d\t%d\t%d\n",preX,preY,brightness);

            newX = preX*cos(rad) + preY*sin(rad);
            newY = preY*cos(rad) - preX*sin(rad) - moving_dist_pixel;
            fprintf(tempMapFile,"%d\t%d\t%d\n",(int) newX,(int) newY,brightness);
        }
        // fclose(previousMapFile);
        fclose(tempMapFile);
        fclose(betterMapFile);
        tempMapFile = fopen("lidar_temp_reading.dat","r");
        betterMapFile = fopen("lidar_better_map_reading.dat","w"); 
        while(fscanf(tempMapFile,"%d %d %d",&preX,&preY,&brightness)>=0) { 
            fprintf(betterMapFile, "%d\t%d\t%d\n",preX,preY,brightness);
        }
        fclose(tempMapFile);
        fclose(betterMapFile);
    }
    has_moved = 0;
    command_angle = 0;
}

u_result capture_and_display(RPlidarDriver * drv, const char* fname)
{


    u_result ans;

    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    float angle;
    float dist;
    _u8   quality;

    FILE* outputFile;

    // ********************************** //
    bool drawing_map = 0;
    // FILE* outputMapFile;
    FILE* tempMapFile;

    switch(current_cmd) {
        case 'l':
            command_angle -= current_cmd_val;
            comparing_angle = previous_min_angle - command_angle;
            if (comparing_angle > 180)
                comparing_angle -= 360;
        case 'r':
            command_angle += current_cmd_val;
            comparing_angle = previous_min_angle - command_angle;
            if (comparing_angle <= -180)
                comparing_angle += 360;
    }
    // ********************************** //

    printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {

        drv->ascendScanData(nodes, count);

        outputFile = fopen(fname, "w");

        // **************************************** //
        // Pham
        if (allow_scanning == 1) {    
            drawing_map = 1;
            // outputMapFile = fopen("lidar_map_reading.dat", "a");
            tempMapFile = fopen("lidar_temp_reading.dat", "w");
        }
        // data_list.clear();
        // data_list_size = 0;
        dist_0_degree = 0;
        dist_90_degree = 0;
        dist_180_degree = 0;
        dist_270_degree = 0;
        min_dist = 0;
        min_angle = 0;
        min_angle_set.clear();
        float angle_scanned_0 = 20;
        float angle_scanned_90 = 110;
        float angle_scanned_180 = -160;
        float angle_scanned_270 = -70;
        // **************************************** //

        for (int pos = 0; pos < (int)count ; ++pos) {
            quality = (nodes[pos].sync_quality>>RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
            dist = nodes[pos].distance_q2/4.0f;

            //Print Debug info on screen
            // printf("Debug [theta: %03.2f Dist: %08.2f]\n", angle, dist);

            // ********************************************** //
            // Pham
            angle -= 180;
            if (dist == 0)
                continue;
            if (std::min(fabs(angle-180),fabs(angle+180))<std::min(fabs(angle_scanned_180-180),fabs(angle_scanned_180+180))) {
                angle_scanned_180 = angle;
                dist_180_degree = dist;
            }
            if (fabs(angle+90)<fabs(angle_scanned_270+90)) {
                angle_scanned_270 = angle;
                dist_270_degree = dist;
            }
            if (fabs(angle)<fabs(angle_scanned_0)) {
                angle_scanned_0 = angle;
                dist_0_degree = dist;
            }
            if (fabs(angle-90)<fabs(angle_scanned_90-90)) {
                angle_scanned_90 = angle;
                dist_90_degree = dist;
            }
            if ((min_dist==0)||(dist<min_dist)) {
                min_dist = dist;
                min_angle = angle;
                min_angle_set.clear();
                min_angle_diff = std::min(fabs(min_angle-comparing_angle),360-fabs(min_angle-comparing_angle));
            }
            if (dist==min_dist) {
                min_angle_set.push_back(angle);
                float local_angle_diff = std::min(fabs(angle-comparing_angle),360-fabs(angle-comparing_angle));
                if (local_angle_diff < min_angle_diff) {
                    min_angle = angle;
                    min_angle_diff = local_angle_diff;
                }
            }
            // ********************************************** //

            //Calculate the scale of pixel vs distance, 
            //     i.e. 1 pixel = ?? meter
            float distScale = 300/DISP_DEFAULT_DIST;  //300 pixel / default_dist
            float distPixel = dist*distScale;

            //Convert angle to radian
            float rad = (float)(angle*PI/180.0);

            //assume a 320 x 320 pixels display, then center point 
            // (location of the RPLidar unit) is at (x=160, y=160) 
            float centerPtX = 0;
            float centerPtY = 0;


            //TODO: Figure out the transformation from angle+distance
            // to (X,Y) coordinate
            float endptX = centerPtX+distPixel*sin(rad); //change this
            float endptY = centerPtY+distPixel*cos(rad); //change this
            // data_list.push_back({endptX,endptY,distPixel,rad});
            // data_list_size += 1;

            // ******************************** //
            float current_ptX = (float) (current_X*10.0*distScale);
            float current_ptY = (float) (current_Y*10.0*distScale);
            float current_rad = (float) ((angle+current_angle)*PI/180.0);
            float mapptX = current_ptX + distPixel*sin(current_rad);
            float mapptY = current_ptY + distPixel*cos(current_rad);
            // ******************************** //

            //Quality of the data is represented by brightness
            //Note: Not used for our studio
            int brightness = (quality<<1) + 128;
            if (brightness>255) brightness=255;

            //Print the data into a output data file
            fprintf(outputFile, "%d\t%d\t%d\n",(int)endptX,(int)endptY, brightness);

            // ******************************** //
            if (drawing_map == 1)
                // fprintf(outputMapFile, "%d\t%d\t%d\n", (int)mapptX,(int)mapptY, brightness);
                fprintf(tempMapFile, "%d\t%d\t%d\n",(int)endptX,(int)endptY, brightness);
            // ******************************** //

        }

        fclose(outputFile);

        // ***************************** //
        // Pham
        if (drawing_map == 1) {
            // fclose(outputMapFile);
            fclose(tempMapFile);
            // allow_scanning = 0;
            drawing_map = 0;
        }
        
        float angle_val;
        float min_angle_sum = 0;
        int number_of_min = min_angle_set.size();
        int valid_min_size = 0;

        for (int i=0; i<number_of_min; i++) {
            angle_val = min_angle_set[i];
            if (fabs(angle_val-min_angle) < 10) {
                min_angle_sum += angle_val;
                valid_min_size += 1;
            } else if (fabs(angle_val-360-min_angle) < 10) {
                min_angle_sum += (angle_val-360);
                valid_min_size += 1;
            } else if (fabs(angle_val+360-min_angle) < 10) {
                min_angle_sum += (angle_val+360);
                valid_min_size += 1;
            } else
                printf("Possible other min angle: %f\n", angle_val);
        }
        if (valid_min_size != 0)
            min_angle = min_angle_sum/valid_min_size;
        if (min_angle < -180)
            min_angle += 360;
        if (min_angle > 180)
            min_angle -= 360;
        min_angle_diff = previous_min_angle - min_angle;
        if (min_angle_diff <= -180)
            min_angle_diff += 360;
        if (min_angle_diff > 180)
            min_angle_diff -= 360;
        
        if (has_moved == 0) {
            delta_angle = min_angle_diff;
            delta_dist = 0;
        } else if (has_moved == 1) {
            delta_angle = 0;
            if (previous_dist_0_degree < previous_dist_180_degree)
                delta_dist = previous_dist_0_degree - dist_0_degree;
            else
                delta_dist = dist_180_degree - previous_dist_180_degree;
        }
        moving_dist_pixel = delta_dist * (300/DISP_DEFAULT_DIST);
        updateBetterMap();
        if (allow_scanning == 1 || has_been_scanned == 0) {
            // back_up_previous_min_angle = previous_min_angle;
            // back_up_previous_dist_180_degree = previous_dist_180_degree;
            previous_min_angle = min_angle;
            previous_dist_180_degree = dist_180_degree;
            previous_dist_0_degree = dist_0_degree;
            update_lidar_backup_var();
        }
        // printf("Current X    : %f\n", current_X);
        // printf("Current Y    : %f\n", current_Y);
        // printf("Current angle: %f\n", current_angle);
        printf("0   degree : %f\n", dist_0_degree/10);
        printf("90  degree : %f\n", dist_90_degree/10);
        printf("180 degree : %f\n", dist_180_degree/10);
        printf("270 degree : %f\n", dist_270_degree/10);
        printf("Min dist   : %f\n", min_dist/10);
        printf("Min angle  : %f\n", min_angle);
        printf("Delta angle: %f\n", delta_angle);
        printf("Delta dist : %f\n", delta_dist/10);
        has_been_scanned = 1;
        // ***************************** //

        printf("** Scan Complete! **\n\n");

    } else {
        printf("error code: %x\n", ans);
    }

    return ans;
}

// **************************************************** //
// Pham
const char * opt_com_path = NULL;
_u32         opt_com_baudrate = 115200;
u_result     op_result;
RPlidarDriver * drv;
rplidar_response_device_health_t healthinfo;
rplidar_response_device_info_t devinfo;

void clear_map() {
    FILE* clearFile;
    clearFile = fopen("lidar_map_reading.dat", "w");
    fclose(clearFile);
    clearFile = fopen("lidar_better_map_reading.dat", "w");
    fclose(clearFile);
    has_been_scanned = 0;
    initializeLidar(g_argc,g_argv);
    scanningLidarData();
    shutdownLidar();
    has_moved = 0;
}

void initializeLidar(int argc, const char * argv[]) {
    // ************************** //
    FILE* clearFile;
    clearFile = fopen("lidar_map_reading.dat", "w");
    fclose(clearFile);
    // ************************** //
    if (argc < 2) {
        print_usage(argc, argv);
        return;
    }
    opt_com_path = argv[1];
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);
    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
}

void shutdownLidar() {
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
}

void scanningLidarData() {

    // try to connect
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        return;
    }

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            // you can check the detailed failure reason
            fprintf(stderr, "Error, operation time out.\n");
        } else {
            fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
            // other unexpected result
        }
        return;
    }

    /*
    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

       printf("\n"
       "Version: " RPLIDAR_SDK_VERSION "\n"
       "Firmware Ver: %d.%02d\n"
       "Hardware Rev: %d\n"
       , devinfo.firmware_version>>8
       , devinfo.firmware_version & 0xFF
       , (int)devinfo.hardware_version);
     */

    // check the device health
    ////////////////////////////////////////
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : ");
        switch (healthinfo.status) {
            case RPLIDAR_STATUS_OK:
                printf("OK.");
                break;
            case RPLIDAR_STATUS_WARNING:
                printf("Warning.");
                break;
            case RPLIDAR_STATUS_ERROR:
                printf("Error.");
                break;
        }
        printf(" (errorcode: %d)\n", healthinfo.error_code);

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return;
    }


    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
        // enable the following code if you want rplidar to be reboot by software
        drv->reset();
        return;
    }

    drv->startMotor();

    // take only one 360 deg scan and display the result as a histogram
    ////////////////////////////////////////////////////////////////////////////////
    if (IS_FAIL(drv->startScan( /* true */ ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
    {
        fprintf(stderr, "Error, cannot start the scan operation.\n");
        return;
    }

    if (IS_FAIL(capture_and_display(drv, "lidar_map_reading.dat"))) {
        fprintf(stderr, "Error, cannot grab scan data.\n");
        return;

    }

}
// **************************************************** //

/*
   int main(int argc, const char * argv[]) {
   const char * opt_com_path = NULL;
   _u32         opt_com_baudrate = 115200;
   u_result     op_result;

// ************************** //
FILE* clearFile;
clearFile = fopen("lidar_map_reading.dat", "w");
fclose(clearFile);
// ************************** //

if (argc < 2) {
print_usage(argc, argv);
return -1;
}
opt_com_path = argv[1];
if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

// create the driver instance
RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

if (!drv) {
fprintf(stderr, "insufficent memory, exit\n");
exit(-2);
}

rplidar_response_device_health_t healthinfo;
rplidar_response_device_info_t devinfo;
do {
// try to connect
if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
, opt_com_path);
break;
}

// retrieving the device info
////////////////////////////////////////
op_result = drv->getDeviceInfo(devinfo);

if (IS_FAIL(op_result)) {
if (op_result == RESULT_OPERATION_TIMEOUT) {
// you can check the detailed failure reason
fprintf(stderr, "Error, operation time out.\n");
} else {
fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
// other unexpected result
}
break;
}

// print out the device serial number, firmware and hardware version number..
printf("RPLIDAR S/N: ");
for (int pos = 0; pos < 16 ;++pos) {
printf("%02X", devinfo.serialnum[pos]);
}

printf("\n"
"Version: " RPLIDAR_SDK_VERSION "\n"
"Firmware Ver: %d.%02d\n"
"Hardware Rev: %d\n"
, devinfo.firmware_version>>8
, devinfo.firmware_version & 0xFF
, (int)devinfo.hardware_version);


// check the device health
////////////////////////////////////////
op_result = drv->getHealth(healthinfo);
if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
printf("RPLidar health status : ");
switch (healthinfo.status) {
    case RPLIDAR_STATUS_OK:
        printf("OK.");
        break;
    case RPLIDAR_STATUS_WARNING:
        printf("Warning.");
        break;
    case RPLIDAR_STATUS_ERROR:
        printf("Error.");
        break;
}
printf(" (errorcode: %d)\n", healthinfo.error_code);

} else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    break;
}


if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
    fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
    // enable the following code if you want rplidar to be reboot by software
    // drv->reset();
    break;
}

drv->startMotor();

// take only one 360 deg scan and display the result as a histogram
////////////////////////////////////////////////////////////////////////////////
if (IS_FAIL(drv->startScan( // true // ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
{
    fprintf(stderr, "Error, cannot start the scan operation.\n");
    break;
}

if (IS_FAIL(capture_and_display(drv, "lidar_reading.dat"))) {
    fprintf(stderr, "Error, cannot grab scan data.\n");
    break;

}

} while(1);

drv->stop();
drv->stopMotor();

RPlidarDriver::DisposeDriver(drv);
return 0;
}
*/

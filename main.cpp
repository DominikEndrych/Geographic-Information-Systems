#include <stdio.h>
#include <stdlib.h>

#include <cmath>

#include <opencv2/opencv.hpp>

#include "defines.h"
#include "utils.h"

#include "stdafx.h"

//#include <proj_api.h>

#define STEP1_WIN_NAME "Heightmap"
#define STEP2_WIN_NAME "Edges"
#define ZOOM           1


struct MouseProbe {
    cv::Mat & heightmap_8uc1_img_;
    cv::Mat & heightmap_show_8uc3_img_;
    cv::Mat & edgemap_8uc1_img_;

    MouseProbe( cv::Mat & heightmap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, cv::Mat & edgemap_8uc1_img )
     : heightmap_8uc1_img_( heightmap_8uc1_img ), heightmap_show_8uc3_img_( heightmap_show_8uc3_img ), edgemap_8uc1_img_( edgemap_8uc1_img )
    {
    }
};

// variables

// function declarations
void flood_fill( cv::Mat & src_img, cv::Mat & dst_img, const int x, const int y );


/**
 * Mouse clicking callback.
 */
void mouse_probe_handler( int event, int x, int y, int flags, void* param ) {
    MouseProbe *probe = (MouseProbe*)param;

    switch ( event ) {

    case cv::EVENT_LBUTTONDOWN:
        printf( "Clicked LEFT at: [ %d, %d ]\n", x, y );
        flood_fill( probe->edgemap_8uc1_img_, probe->heightmap_show_8uc3_img_, x, y );
        break;

    case cv::EVENT_RBUTTONDOWN:
        printf( "Clicked RIGHT at: [ %d, %d ]\n", x, y );
        break;
    }
}


void create_windows( const int width, const int height ) {
    cv::namedWindow( STEP1_WIN_NAME, 0 );
    cv::namedWindow( STEP2_WIN_NAME, 0 );

    cv::resizeWindow( STEP1_WIN_NAME, width*ZOOM, height*ZOOM );
    cv::resizeWindow( STEP2_WIN_NAME, width*ZOOM, height*ZOOM );

} // create_windows


/**
 * Perform flood fill from the specified point (x, y) for the neighborhood points if they contain the same value,
 * as the one that came in argument 'value'. Function recursicely call itself for its 4-neighborhood.
 * 
 * edgemap_8uc1_img - image, in which we perform flood filling
 * heightmap_show_8uc3_img - image, in which we display the filling
 * value - value, for which we'll perform flood filling
 */
void fill_step( cv::Mat & edgemap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, const int x, const int y, const uchar value ) {
    int width, height;
    width = edgemap_8uc1_img.cols;
    height = edgemap_8uc1_img.rows;

    // Return if wrong color is found or out of bounds
    if (y >= height || y < 0 ||
        x >= width || x < 0 ||
        edgemap_8uc1_img.at<uchar>(y,x) != value ||
        heightmap_show_8uc3_img.at<cv::Vec3b>(y,x) == cv::Vec3b(0,0,255)) return;

    // Red
    heightmap_show_8uc3_img.at<cv::Vec3b>(y, x)[0] = 0;
    heightmap_show_8uc3_img.at<cv::Vec3b>(y, x)[1] = 0;
    heightmap_show_8uc3_img.at<cv::Vec3b>(y, x)[2] = 255;

    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x + 1, y, value);
    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x - 1, y, value);
    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x, y + 1, value);
    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x, y - 1, value);

} //fill_step


/**
 * Perform flood fill from the specified point (x, y). The function remembers the value at the coordinate (x, y)
 * and fill the neighborhood using 'fill_step' function so long as the value in the neighborhood points are the same.
 * Execute the fill on a temporary image to prevent the original image from being repainted.

 * edgemap_8uc1_img - image, in which we perform flood filling
 * heightmap_show_8uc3_img - image, in which we display the filling
 */
void flood_fill( cv::Mat & edgemap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, const int x, const int y ) {
    cv::Mat tmp_edgemap_8uc1_img;
    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x, y, edgemap_8uc1_img.at<uchar>(y, x));       // Perform fill_step

} //flood_fill


/**
 * Find the minimum and maximum coordinates in the file.
 * Note that the file is the S-JTSK coordinate system.
 */
void get_min_max( const char *filename, float *a_min_x, float *a_max_x, float *a_min_y, float *a_max_y, float *a_min_z, float *a_max_z, int lidar_type ) {
    FILE *f = NULL;
    float x, y, z;
    float min_x, min_y, min_z, max_x, max_y, max_z;
    int l_type;

    // My code - ex1
    f = fopen(filename, "rb");   // Open file in binary mode
    if (f != NULL)
    {
        //printf("I am reading!");
        //printf("\n");

        float fBuffer[3];       // Buffer for first three floats
        int iBuffer[1];         // Buffer for l_type

        max_x = -FLT_MAX;
        max_y = -FLT_MAX;
        max_z = -FLT_MAX;
        min_x = FLT_MAX;
        min_y = FLT_MAX;
        min_z = FLT_MAX;

        int n = 0;
        
        // Read the whole file
        while (!feof(f))
        {
            n++;
            //printf("Reding \n");
            fread(fBuffer, sizeof(float), 3, f);    // Reading first three floats
            fread(iBuffer, sizeof(int), 1, f);      // Reding l_type

            x = fBuffer[0];
            y = fBuffer[1];
            z = fBuffer[2];
            l_type = iBuffer[0];
            
            if (l_type == lidar_type || lidar_type < 0)
            {
                if (x > max_x) max_x = x;
                if (y > max_y) max_y = y;
                if (z > max_z) max_z = z;
                if (x < min_x) min_x = x;
                if (y < min_y) min_y = y;
                if (z < min_z) min_z = z;
            }
            

        }
        fclose(f);      // Close the file

        *a_max_x = max_x;
        *a_max_y = max_y;
        *a_max_z = max_z;
        *a_min_x = min_x;
        *a_min_y = min_y;
        *a_min_z = min_z;
    }

}

int ConvertRange(int OldValue, int OldMin, int OldMax, int NewMin, int NewMax)
{
    int OldRange = (OldMax - OldMin);
    int NewRange = (NewMax - NewMin);
    int NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin;
    return NewValue;
}

/**
 * Fill the image by data from lidar.
 * All lidar points are stored in a an array that has the dimensions of the image. Then the pixel is assigned
 * a value as an average value range from at the corresponding array element. However, with this simple data access, you will lose data precission.
 * filename - file with binarny data
 * img - input image
 */
void fill_image( const char *filename, cv::Mat & heightmap_8uc1_img, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int lidar_type ) {
    FILE *f = NULL;
    int delta_x, delta_y, delta_z;
    float fx, fy, fz;
    int x, y, l_type;
    int stride;
    int num_points = 1;
    float range = 0.0f;
    float **sum_height = NULL;
    int **sum_height_count = NULL;

    if (lidar_type == 1 || lidar_type == 2 || lidar_type == 3)
    {
        get_min_max(filename, &min_x, &max_x, &min_y, &max_y, &min_z, &max_z, lidar_type);
    }

    // zjistime sirku a vysku obrazu
    delta_x = round( max_x - min_x + 0.5f );
    delta_y = round( max_y - min_y + 0.5f );
    delta_z = round( max_z - min_z + 0.5f );

    stride = delta_x;

    // 1:
    // We allocate helper arrays, in which we store values from the lidar
    // and the number of these values for each pixel
    //sum_height = new float[delta_y * delta_x];
    //sum_height_count = new int[delta_y * delta_x];
    sum_height = new float* [delta_y];
    sum_height_count = new int* [delta_y];

    for (int i = 0; i < delta_y; i++)
    {
        sum_height[i] = new float[delta_x];
        sum_height_count[i] = new int[delta_x];
    }

    for (int i = 0; i < delta_y; i++)
    {
        for (int j = 0; j < delta_x; j++)
        {
            sum_height[i][j] = 0;
            sum_height_count[i][j] = 0;
        }
    }

    /*
    OldRange = (OldMax - OldMin)
    NewRange = (NewMax - NewMin)
    NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    */


    // 2:
    // go through the file and assign values to the field
    // beware that in S-JTSK the beginning of the co-ordinate system is at the bottom left,
    // while in the picture it is top left

    // My code - ex1
    f = fopen(filename, "rb");   // Open file in binary mode
    if (f != NULL)
    {
        //printf("I am reading!");
        //printf("\n");

        float fBuffer[3];       // Buffer for first three floats
        int iBuffer[1];         // Buffer for l_type

        // Read the whole file
        while (!feof(f))
        {
            //printf("Reding \n");
            fread(fBuffer, sizeof(float), 3, f);    // Reading first three floats
            fread(iBuffer, sizeof(int), 1, f);      // Reding l_type

            fx = fBuffer[0];
            fy = fBuffer[1];
            fz = fBuffer[2];
            l_type = iBuffer[0];        // l_type

            // Assign to helper field
            y = ConvertRange(fy, min_y, max_y, 0, delta_y - 1);
            x = ConvertRange(fx, min_x, max_x, 0, delta_x - 1);
            //fz = ConvertRange(fz, min_z, max_z, 0, 255);

            if (l_type == lidar_type || lidar_type < 0)
            {
                sum_height[y][x] = sum_height[y][x] + fz;
                sum_height_count[y][x] = sum_height_count[y][x] + 1;     // Increment counter
            }

        }
        fclose(f);      // Close the file
        printf("File closed \n");
    }

    // 3:
    // assign values from the helper field into the image
    float value;
    float new_value;
    int count;
    int normalized_value;
    for (y = 0; y < heightmap_8uc1_img.rows; y++) {
        for (x = 0; x < heightmap_8uc1_img.cols; x++) {
            count = sum_height_count[(heightmap_8uc1_img.rows-1) - y][x];
            value = sum_height[(heightmap_8uc1_img.rows - 1) - y][x];
            new_value = value / count;
            if (count != 0)
            {
                normalized_value = ConvertRange(new_value, min_z, max_z, 0, 255);
            }
                
            else normalized_value = 0;
            //normalized_value = ConvertRange(new_value, min_z, max_z, 0, 255);
            heightmap_8uc1_img.at<uchar>(y, x) = normalized_value;
        }
    }
    //printf("Image created");
}


void make_edges( const cv::Mat & src_8uc1_img, cv::Mat & edgemap_8uc1_img ) {
    cv::Canny( src_8uc1_img, edgemap_8uc1_img, 1, 80 );
}


/**
 * Transforms the image so it contains only two values.
 * Threshold may be set experimentally.
 */
void binarize_image( cv::Mat & src_8uc1_img ) {
    int x, y;
    uchar value;
    for (y = 0; y < src_8uc1_img.rows; y++) {
        for (x = 0 + 1; x < src_8uc1_img.cols; x++) {
            value = src_8uc1_img.at<uchar>(y, x);                   // Get value of pixel

            if (value > 128) src_8uc1_img.at<uchar>(y, x) = 255;    // White
            else src_8uc1_img.at<uchar>(y, x) = 0;                  // Black
        }
    }
}


void dilate_and_erode_edgemap(cv::Mat& edgemap_8uc1_img) {
    // Copy
    cv::Mat edgemap_copy = edgemap_8uc1_img.clone();
    int newValue;

    //Dilatation
    for (int y = 0+1; y < edgemap_copy.rows-1; y++) {
        for (int x = 0+1; x < edgemap_copy.cols-1; x++) {

            if (edgemap_copy.at<uchar>(y, x) == 255) newValue = 255;
            else newValue = 0;
            if (newValue != 255) continue;

            // Change 3x3 area around pixel
            for (int i_y = y - 1; i_y <= y + 1; i_y++)
            {
                for (int i_x = x - 1; i_x <= x + 1; i_x++)
                {
                    edgemap_8uc1_img.at<uchar>(i_y, i_x) = 255;
                }
            }
        }
    }

    //cv::imshow("puvodni", edgemap_copy);
    edgemap_copy = edgemap_8uc1_img.clone();

    //Erosion
    for (int y = 0 + 1; y < edgemap_copy.rows - 1; y++) {
        for (int x = 0 + 1; x < edgemap_copy.cols - 1; x++) {
            
            // Check if pixel has 255 value
            if (edgemap_copy.at<uchar>(y, x) == 255)
            {
                newValue = 255;
                // Scan 3x3 pixel area
                for (int i_y = y - 1; i_y <= y + 1; i_y++)
                {
                    for (int i_x = x - 1; i_x <= x + 1; i_x++)
                    {
                        if (edgemap_copy.at<uchar>(i_y, i_x) != 255)
                        {
                            newValue = 0;
                            edgemap_8uc1_img.at<uchar>(y, x) = newValue;    // Change pixel
                        }
                    }
                    if (newValue == 0) break;
                }
            }
            
        }
    }

}


void process_lidar( const char *txt_filename, const char *bin_filename, const char *img_filename ) {
    float min_x, max_x, min_y, max_y, min_z, max_z;
    float delta_x, delta_y, delta_z;
    MouseProbe *mouse_probe;

    cv::Mat heightmap_8uc1_img;      // image of source of lidar data
    cv::Mat heightmap_show_8uc3_img; // image to detected areas
    cv::Mat edgemap_8uc1_img;        // image for edges

    get_min_max( bin_filename, &min_x, &max_x, &min_y, &max_y, &min_z, &max_z, -1);

    printf( "min x: %f, max x: %f\n", min_x, max_x );
    printf( "min y: %f, max y: %f\n", min_y, max_y );
    printf( "min z: %f, max z: %f\n", min_z, max_z );

    delta_x = max_x - min_x;
    delta_y = max_y - min_y;
    delta_z = max_z - min_z;

    printf( "delta x: %f\n", delta_x );
    printf( "delta y: %f\n", delta_y );
    printf( "delta z: %f\n", delta_z );

    // create images according to data from the source file
    
    heightmap_8uc1_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC1 );
    
    heightmap_show_8uc3_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC3 );
    
    edgemap_8uc1_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC3 );

    create_windows( heightmap_8uc1_img.cols, heightmap_8uc1_img.rows );
    mouse_probe = new MouseProbe( heightmap_8uc1_img, heightmap_show_8uc3_img, edgemap_8uc1_img );

    cv::setMouseCallback( STEP1_WIN_NAME, mouse_probe_handler, mouse_probe );
    cv::setMouseCallback( STEP2_WIN_NAME, mouse_probe_handler, mouse_probe );

    printf( "Image w=%d, h=%d\n", heightmap_8uc1_img.cols, heightmap_8uc1_img.rows );
    
    
    cv::Mat lidar_map_0 = cv::Mat(cvSize(cvRound(delta_x + 0.5f), cvRound(delta_y + 0.5f)), CV_8UC1);;
    cv::Mat lidar_map_1 = cv::Mat(cvSize(cvRound(delta_x + 0.5f), cvRound(delta_y + 0.5f)), CV_8UC1);
    cv::Mat lidar_map_2 = cv::Mat(cvSize(cvRound(delta_x + 0.5f), cvRound(delta_y + 0.5f)), CV_8UC1);

    fill_image(bin_filename, lidar_map_0, min_x, max_x, min_y, max_y, min_z, max_z, 0);
    fill_image(bin_filename, lidar_map_1, min_x, max_x, min_y, max_y, min_z, max_z, 1);
    fill_image(bin_filename, lidar_map_2, min_x, max_x, min_y, max_y, min_z, max_z, 2);

    cv::imshow("Lidar 0", lidar_map_0);
    cv::imshow("Lidar 1", lidar_map_1);
    cv::imshow("Lidar 2", lidar_map_2);


    // fill the image with data from lidar scanning
    get_min_max(bin_filename, &min_x, &max_x, &min_y, &max_y, &min_z, &max_z, -1);
    fill_image( bin_filename, heightmap_8uc1_img, min_x, max_x, min_y, max_y, min_z, max_z, -1 );
    cv::cvtColor( heightmap_8uc1_img, heightmap_show_8uc3_img, CV_GRAY2RGB );
    cv::imwrite("heightmap.png", heightmap_show_8uc3_img);

    // create edge map from the height image
    make_edges( heightmap_8uc1_img, edgemap_8uc1_img );

    // binarize image, so we can easily process it in the next step
    binarize_image( edgemap_8uc1_img );
    printf("Binarize done \n");
    
    // implement image dilatation and erosion
    dilate_and_erode_edgemap( edgemap_8uc1_img );

    cv::imwrite( img_filename, heightmap_8uc1_img );

    // wait here for user input using (mouse clicking)
    
    while ( 1 ) {
        cv::imshow( STEP1_WIN_NAME, heightmap_show_8uc3_img );
        cv::imshow( STEP2_WIN_NAME, edgemap_8uc1_img );
        int key = cv::waitKey( 10 );
        if ( key == 'q' ) {
            break;
        }
    }
}


int main( int argc, char *argv[] ) {
    char *txt_file, *bin_file, *img_file;

    printf("argc: %d\n", argc );
    
    if ( argc < 4 ) {
        printf( "Not enough command line parameters.\n" );
        exit( 1 );
    }

    txt_file = argv[ 1 ];
    bin_file = argv[ 2 ];
    img_file = argv[ 3 ];

    process_lidar( txt_file, bin_file, img_file );

    return 0;
}

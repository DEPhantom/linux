#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const int frame_smallest_size = 250;

int find_frame( Mat image, int first_x, int first_y ) {
  int count = first_x;
  bool is_pink = true;
  while ( count < image.rows && is_pink ) {
    Vec3b color = image.at<Vec3b>( count, first_y ); // height
    if ( color[0] > 200 && color[1] < 50 && color[2] > 200 ) {
      ;
    } // if()
    else {
      is_pink = false;
    } // else

    count++;

  } // while()

  if ( count-first_x > frame_smallest_size ) { // is pink frame 
    count = first_y;
    is_pink = true;
  } // if()
  else {
    return -1;
  } // else

  while ( count < image.cols && is_pink ) {
    Vec3b color = image.at<Vec3b>( first_x, count ); // width
    if ( color[0] > 200 && color[1] < 50 && color[2] > 200 ) {
      ;
    } // if()
    else {
      is_pink = false;
    } // else

    count++;

  } // while()

  return count - first_y;

} // find_frame()

int main(int argc, char* argv[] ) {

  Mat image;
  image = imread( "predictions.jpg", IMREAD_COLOR );
  cout << image.rows << endl; // height
  cout << image.cols << endl; // width
  // cout << image.at<Vec3b>( 2512 , 1736) << endl; // (height,width)
  int ans = 0;
  int first_pink_x = 0;
  int first_pink_y = 0;
  int x = 0;
  int y = 0;
  while( x < image.rows && ans <= frame_smallest_size ) {
    while( y < image.cols && ans <= frame_smallest_size ) {
        // get pixel
        // Vec3b & color = image.at<Vec3b>(y,x); change color
        Vec3b color = image.at<Vec3b>( x, y );

        // find first pink
        if ( color[0] > 200 && color[1] < 50  && color[2] > 200 ) {
          ans = find_frame( image, x, y ); 
          cout << y << " " << x << " " << ans << endl;
          if ( ans > frame_smallest_size ) {
            first_pink_x = x;
            first_pink_y = y; 
          } // if()

        } // if()

        // set pixel
        // color[0] = 13 // blue
        // color[1] = 13 // green
        // color[2] = 13 // red
        //image.at<Vec3b>(Point(x,y)) = color;
        //if you copy value
        y++;
    } // while()
   
    y = 0;
    x++;

  } // while()

  if ( !image.data ) {
    cout << "No image data n" << endl;
    return -1;
  }

  if ( ans > frame_smallest_size ) {
    cout << "point: "<< ans/2+first_pink_y << endl;
  } // if()
  else {
    -99
  } // else

  return 0;

} // main()

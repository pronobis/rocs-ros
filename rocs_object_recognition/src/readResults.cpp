#include <iostream>
#include <string>
#include <fstream>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <ros/ros.h>


#ifdef WIN32
#include <io.h>
#else
#include <dirent.h>
#endif

using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
  float score_threshold;
  string STRING;
  string image_path;
  double score_obtained, seq_number;
  double odom_x, odom_y, odom_t;
  double x1,x2,y1,y2;

  if (argc < 3)
  {
    cout << "Usage: ./readResultsPosition file_path score_threshold" << endl;
    return 0;
  }

  cout << "This program read latentSVM detector results from file." << endl <<
          endl <<
          "Keys:" << endl <<
          "'n' - to go to the next image;" << endl <<
          "'esc' - to quit." << endl <<
          endl;


  string file_name = argv[1];
  score_threshold = atof(argv[2]);

  //Open input file
  ifstream is;
  is.open (file_name.data(), ios::binary );

  while(!is.eof()) // To get you all the lines.
  {
    getline(is,image_path); // Saves the line in STRING.

    if(image_path.length()==0)
      break;


    is >> x1; //x1
    is >> x2; //x2
    is >> y1; //y1
    is >> y2; //y2

    is >> score_obtained; //Score
    is >> seq_number; //Sequencie number
    is >> odom_x; //odom x
    is >> odom_y; //odom y
    is >> odom_t; //odom t

    getline(is,STRING); //Empty
    getline(is,STRING); //Empty

    if(score_obtained<score_threshold)
      continue;

    cout<< "Image filename: " << image_path << endl; // Prints our STRING.
    cout << "Score " << score_obtained << endl;
    cout << "Seq. Number " << seq_number << endl;
    cout << "Odom x " << odom_x << endl;
    cout << "Odom y " << odom_y << endl;
    cout << "Odom t " << odom_t << endl;

    std::ostringstream strs;
    strs << score_obtained;
    std::string score = strs.str();


    Mat image = imread( image_path );
    if( image.empty() )
    {
      ROS_ERROR("Image empty");
      continue;
    }

    //Show the detection and the score
    Rect rect(x1, y1, x2-x1, y2-y1);
    rectangle( image, rect,  cvScalar(255,0,0), 3 );
    putText( image, score, Point(rect.x+4,rect.y-10), FONT_ITALIC, 0.55, cvScalar(255,0,0), 2 );
    imshow( "result", image );

        for(;;)
        {
            int c = waitKey();
            if( (char)c == 'n')
                break;
            else if( (char)c == '\x1b' )
                exit(0);
        }




  }

  is.close();

  return 0;

}

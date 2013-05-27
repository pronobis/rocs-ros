#include <iostream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#ifdef WIN32
#include <io.h>
#else
#include <dirent.h>
#endif

#ifdef HAVE_CVCONFIG_H
#include <cvconfig.h>
#endif

#ifdef HAVE_TBB
#include "tbb/task_scheduler_init.h"
#endif

using namespace std;
using namespace cv;

float score_level;

void help()
  {
      cout << "This program demonstrated the use of the latentSVM detector." << endl <<
              "It reads in a trained object models and then uses them to detect the object in an images" << endl <<
               endl <<
              "Call:" << endl <<
               endl <<
              "Keys:" << endl <<
              "'n' - to go to the next image;" << endl <<
              "'esc' - to quit." << endl <<
              endl;
  }

static void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads )
{
    vector<LatentSvmDetector::ObjectDetection> detections;

    TickMeter tm;

#ifdef HAVE_TBB
    tbb::task_scheduler_init init(tbb::task_scheduler_init::deferred);
    if (numThreads > 0)
    {
        init.initialize(numThreads);
        cout << "Number of threads: " << numThreads << endl;
    }
    else
    {
        cout << "Number of threads is not correct for TBB version"  << endl;
        return;
    }
#endif



    tm.start();
    detector.detect( image, detections, overlapThreshold, numThreads);
    tm.stop();

    cout << "Detection time = " << tm.getTimeSec() << " sec" << endl;

#ifdef HAVE_TBB
    init.terminate();
#endif

    const vector<string> classNames = detector.getClassNames();
    CV_Assert( colors.size() == classNames.size() );

    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        cout << "confidence level " << classNames[od.classID] <<": " << od.score << endl;

        if(od.score>score_level)
        {
          rectangle( image, od.rect, colors[od.classID], 3 );
          // put name of object over the all rectangles
          putText( image, classNames[od.classID], Point(od.rect.x+4,od.rect.y+13), FONT_HERSHEY_SIMPLEX, 0.55, colors[od.classID], 2 );

          // put score value
          std::ostringstream strs;
          strs << od.score;
          std::string score = strs.str();

          putText( image, score, Point(od.rect.x+od.rect.width-100,od.rect.y+od.rect.height-10), FONT_ITALIC, 0.55, cvScalar(255,0,0), 2 );
        }
    }
}

static void readDirectory( const string& directoryName, vector<string>& filenames, bool addDirectoryName=true )
{
    filenames.clear();

    DIR* dir = opendir( directoryName.c_str() );
    if( dir != NULL )
    {
        struct dirent* dent;
        while( (dent = readdir(dir)) != NULL )
        {
            if( addDirectoryName )
                filenames.push_back( directoryName + "/" + string(dent->d_name) );
            else
                filenames.push_back( string(dent->d_name) );
        }
    }

    sort( filenames.begin(), filenames.end() );
}

int main(int argc, char* argv[])
{
    help();

    string images_folder, models_folder;
    float overlapThreshold = 0.2f;
    int numThreads = -1;

    if (argc < 4)
    {
      cout << "Usage: ./object_detector images_folder models_folder score_level" << endl;
      return 0;
    }

    images_folder = argv[1];
    models_folder = argv[2];
    score_level = (float)atof(argv[3]);

    vector<string> images_filenames, models_filenames;
    readDirectory( images_folder, images_filenames );
    readDirectory( models_folder, models_filenames );

    LatentSvmDetector detector( models_filenames );
    if( detector.empty() )
    {
        cout << "Models cann't be loaded" << endl;
        exit(-1);
    }

    const vector<string>& classNames = detector.getClassNames();
    cout << "Loaded " << classNames.size() << " models:" << endl;
    for( size_t i = 0; i < classNames.size(); i++ )
    {
        cout << i << ") " << classNames[i] << "; ";
    }
    cout << endl;

    cout << "overlapThreshold = " << overlapThreshold << endl;


    vector<Scalar> colors;
    generateColors( colors, detector.getClassNames().size() );


    for( size_t i = 0; i < images_filenames.size(); i++ )
    {
        Mat image = imread( images_filenames[i] );
        if( image.empty() )  continue;

        cout << "Process image " << images_filenames[i] << endl;
        detectAndDrawObjects( image, detector, colors, overlapThreshold, numThreads );

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

    return 0;
}

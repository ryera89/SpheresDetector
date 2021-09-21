#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>

#include "SpheresDetectionUtils.h"

/*Spheres detector in 3D scanned images using PCL library*/

using namespace std;


int main()
{
    float halfCroppingSize;
    double minRadius;
    double maxRadius;
    char answer = 'y';
    bool changeDetectionParameters = true;
    string pointCloudFileName;
    string pickedPointsFileName;

    pcl::visualization::CloudViewer viewer( "Kugelmodell" );

    do
    {
        cout << "Introduce the Polygon file \".ply\" name (including the path): ";
        cin >> pointCloudFileName;

        //point cloud
        pcl::PointCloud<Point_t>::Ptr cloud( new pcl::PointCloud<Point_t> );
        if (pcl::io::loadPLYFile( pointCloudFileName, *cloud ) != -1) //success loading the .ply file
        {

            viewer.showCloud( cloud, "Kugelmodell" ); //show cloud in viewer

            vector<Eigen::Vector4f> points; //store the approximate coordinates of the spheres center

            bool pickedPointFileOpened = false; //flag in case of error opening the .pp file

            while ( !pickedPointFileOpened )
            {
                cout << "Introduce the Picked Points file \".pp\" name (including the path): ";
                cin >> pickedPointsFileName;

                pickedPointFileOpened = ReadPickedPointsFile( pickedPointsFileName, points );

                if ( !pickedPointFileOpened ) cerr << "Error loading .pp file introduce a valid file name \n";
            }


            while ( changeDetectionParameters )
            {
                //I use the value of 4 for the cropping size in my tests
                cout << "Introduce the Submeshes half cropping size: ";
                cin >> halfCroppingSize;

                //Crop cloud boxes with center at the given points
                std::vector<pcl::PointCloud<Point_t>::Ptr> vCroppedClouds = CroppedClouds(points,cloud,halfCroppingSize);

                //Show results on viewer - cropped cloud in green
                for (size_t i = 0; i < vCroppedClouds.size(); ++i)
                {
                    string cloudId = "CroppedCloud" + to_string(i);
                    viewer.showCloud( vCroppedClouds[i], cloudId );
                }

                //I use the values of 1 or 2 for the min radius in my tests
                cout << "Introduce the min radius: ";
                cin >> minRadius;
                //I use the value of 3 or 4 for the max radius in my tests
                cout << "Introduce the max radius (should be greater than min radius): ";
                cin >> maxRadius;

                //Try to detect spheres in the cropped cloud and show in visualizer
                for (size_t i = 0; i < vCroppedClouds.size(); ++i)
                {
                    //Store center of the sphere and radius
                    SphereData sphereData;

                    if ( DetectSphere( vCroppedClouds[i], minRadius, maxRadius, sphereData ) )
                    {
                        //Sphere Detection Success. Print output

                        cout << "********<>********<>********<>********<>********<>********<>********\n";
                        cout << "Sphere " + to_string(i) << "\n";
                        cout << "Sphere Center: " << "[ x = " << sphereData.m_coordinates.x()
                             << ", y = " << sphereData.m_coordinates.y() << ", z = " << sphereData.m_coordinates.z() << "] \n"
                             << "Sphere Radius = " << sphereData.m_radius << "\n";
                        //Show results on viewer - cropped cloud in green and detected sphere in red
                        string cloudId = "CroppedCloud" + to_string(i);
                        viewer.showCloud( vCroppedClouds[i], cloudId );

                    }
                    else
                    {
                        //Sphere detection fail
                        cerr << "Spheres Not Detected. Change crooping size, min radius and/or max radius parameters: \n";
                        continue;
                    }
                }

                cout << "********<>********<>********<>********<>********<>********<>********\n";
                cout << "Do you wish to change detection parameters crooping size, \n"
                        " min radius and/or max radius and perform another sphere detection (yes = y):  ";
                cin >> answer;
                if (answer != 'y') changeDetectionParameters = false;

            }



            cout << "Do you wish to analize another .ply file (yes = y): ";
            cin >> answer;
        }
        else
        {
            cerr << "Error loading .ply file. Introduce a valid file name \n";
        }

    } while( answer == 'y' );


//    pcl::PointCloud<Point_t>::Ptr cloud( new pcl::PointCloud<Point_t> );
//    pcl::io::loadPLYFile( "/media/ryeram/work/Development/C++/InputFiles/Kugelmodell_edges_nico.ply", *cloud );

//    pcl::visualization::CloudViewer viewer( "Kugelmodell" );
//    viewer.showCloud( cloud, "Kugelmodell" );

//    //reading Picked Points file
//    vector<Eigen::Vector4f> points;
//    string fileName = "/media/ryeram/work/Development/C++/InputFiles/Kugelmodell_edges_nico_picked_points.pp";
//    ReadPickedPointsFile( fileName,points );

    //********<>********<>********<>********<>********<>********<>********<>********<>********<>********<>********<>********

    /*PCLVisualizer does not work I dont know why. I can't see if the normals are correctly computed*/

    //pcl::visualization::PCLVisualizer::Ptr viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer1->setBackgroundColor (0, 0, 0);
    //viewer1->addPointCloud<Point_t> (cloud, "Cloud");
    //viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer1->addCoordinateSystem (1.0);
    //viewer1->initCameraParameters ();
    //return (viewer);

    //********<>********<>********<>********<>********<>********<>********<>********<>********<>********<>********<>********

//    std::vector<pcl::PointCloud<Point_t>::Ptr> vCroppedClouds = CroppedClouds(points,cloud,5.0f);

//    for (size_t i = 0; i < vCroppedClouds.size(); ++i)
//    {
//        string name = "CroppedCloud" + to_string(i);
//        viewer.showCloud(vCroppedClouds[i],name);
//    }

//    std::vector<pcl::PointCloud<NPoint_t>::Ptr> vNormals = ComputeNormals(cloud, vCroppedClouds);

//    for (size_t i = 0; i < vCroppedClouds.size(); ++i)
//    {
//        if (DetectSphere(vCroppedClouds[i]))
//        {
//            cout << "SphereFound Cloud Point model \n";
//        }

//        string name = "CroppedCloud" + to_string(i);
//        viewer.showCloud(vCroppedClouds[i],name);
//    }




//    while (!viewer.wasStopped()){}

    return 0;
}

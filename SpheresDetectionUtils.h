#ifndef SPHERESDETECTIONUTILS_H
#define SPHERESDETECTIONUTILS_H

#include <vector>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <string>
#include <regex>
#include <fstream>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>


using Point_t = pcl::PointXYZRGBA;
using NPoint_t = pcl::Normal;


struct SphereData
{
    Eigen::Vector3d m_coordinates; //coordinates os the sphere center
    double m_radius; //sphere radius
};

//Extract the "axis" coordinate of a point from a line of picked point file .pp
double ExtractCoordinate(const std::string& line,char axis)
{
    std::regex regAxis;
    axis = tolower(axis);
    switch (axis) {
    case 'x':
        regAxis = "x=\"";
        break;
    case 'y':
        regAxis = "y=\"";
        break;
    case 'z':
        regAxis = "z=\"";
        break;
    default:
        std::cerr << "Axis label invalid returning -1 \n";
        return -1;
    }

    std::regex regApostrophe{ "\"" };
    std::smatch m1;
    std::smatch m2;
    std::string suffix;

    if (regex_search(line,m1,regAxis))
    {
        suffix = m1.suffix();
        if (regex_search(suffix, m2, regApostrophe))
        //std::cout << axis << " = " << m2.prefix() << "\n";
        return stod(m2.prefix());
    }
    std::cerr << "Coordinate of axis " << axis << " not found, returning -1 \n";
    return -1;
}

bool ReadPickedPointsFile(const std::string& fileName,std::vector<Eigen::Vector4f>& points)
{
    points.clear();

    std::cout << fileName << "\n";
    std::ifstream fileStream(fileName, std::ios_base::in);
    if (fileStream.is_open())
    {
        std::string line;
        std::regex regPoint( "point" );
        std::smatch pointMatch;
        while ( getline( fileStream, line ) )
        {
            if ( regex_search( line, pointMatch, regPoint ) )
            {
                Eigen::Vector4f point( 0.f,0.f,0.f,1.f );
                point.x() = ExtractCoordinate( pointMatch.suffix(),'x' );
                point.y() = ExtractCoordinate( pointMatch.suffix(),'y' );
                point.z() = ExtractCoordinate( pointMatch.suffix(),'z' );

                points.push_back(point);
            }
        }
    }
    else
    {
        std::cerr << "Filed to open Picked Points file: " + fileName << "\n";
        return false;
    }
    return true;
}
//return a vector of cropped points clouds pointers
std::vector<pcl::PointCloud<Point_t>::Ptr> CroppedClouds(const std::vector<Eigen::Vector4f>& vPoints,
                                                         pcl::PointCloud<Point_t>::Ptr cloud,float croppingHalfSize)
{
    std::vector<pcl::PointCloud<Point_t>::Ptr> vCroppedClouds;

    pcl::CropBox<Point_t> CropBoxFilter;
    CropBoxFilter.setInputCloud(cloud);

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        Eigen::Vector4f minPoint = vPoints[i];
        minPoint.x() -= croppingHalfSize;
        minPoint.y() -= croppingHalfSize;
        minPoint.z() -= croppingHalfSize;

        Eigen::Vector4f maxPoint = vPoints[i];
        maxPoint.x() += croppingHalfSize;
        maxPoint.y() += croppingHalfSize;
        maxPoint.z() += croppingHalfSize;

        CropBoxFilter.setMin(minPoint);
        CropBoxFilter.setMax(maxPoint);

        pcl::PointCloud<Point_t>::Ptr croppedCloud( new pcl::PointCloud<Point_t> );
        CropBoxFilter.filter( *croppedCloud );
        //std::vector<int> inliersIndices;
        //CropBoxFilter.filter(inliersIndices);

        //Changing the color of the points of the cropped cloud to green
        for (size_t i = 0; i < croppedCloud->points.size(); ++i)
        {
            //cloud->points[inliersIndices[i]].r = 0;
            //cloud->points[inliersIndices[i]].g = 255;
            //cloud->points[inliersIndices[i]].b = 0;
            croppedCloud->points[i].r = 0;
            croppedCloud->points[i].g = 255;
            croppedCloud->points[i].b = 0;
        }

        vCroppedClouds.push_back(croppedCloud);

    }
    return vCroppedClouds;
}

std::vector<pcl::PointCloud<NPoint_t>::Ptr> ComputeNormals(pcl::PointCloud<Point_t>::Ptr cloud,
                                                              std::vector<pcl::PointCloud<Point_t>::Ptr>& vCroppedClouds)
{
    std::vector<pcl::PointCloud<NPoint_t>::Ptr> vNormals;

    for (size_t i = 0; i < vCroppedClouds.size(); ++i)
    {
        pcl::NormalEstimation<Point_t, NPoint_t> normalEstimation;
        normalEstimation.setInputCloud( vCroppedClouds[i] );

        normalEstimation.setSearchSurface( cloud );

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given surface dataset.
        pcl::search::KdTree<Point_t>::Ptr tree( new pcl::search::KdTree<Point_t>());
        normalEstimation.setSearchMethod( tree );

        // Output datasets
        pcl::PointCloud<NPoint_t>::Ptr croppedCloudsNormal (new pcl::PointCloud<NPoint_t>);

        // Use all neighbors in a sphere of radius 3cm
        normalEstimation.setRadiusSearch (0.03);

        // Compute the features
        normalEstimation.compute (*croppedCloudsNormal);

        std::cout << croppedCloudsNormal->size() << " ------ " << vCroppedClouds[i]->size() << "\n";

        vNormals.push_back(croppedCloudsNormal);
    }

    return vNormals;
}

bool DetectSphere(pcl::PointCloud<Point_t>::Ptr cloud,double minRadius,double maxRadius, SphereData& sphereData)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<Point_t> segmentation;

    segmentation.setInputCloud(cloud);
    //segmentation.setInputNormals(cloud);
    //segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    segmentation.setModelType(pcl::SACMODEL_SPHERE); //Model
    segmentation.setMethodType(pcl::SAC_RANSAC); //Algorithm
    segmentation.setDistanceThreshold(0.1); //I found that the value 0.1 gives a good result for this problem
    segmentation.setOptimizeCoefficients(true);
    segmentation.setRadiusLimits(minRadius, maxRadius);
    segmentation.setEpsAngle(15 / (180/3.141592654));
    segmentation.setMaxIterations(1000000);

    pcl::PointIndices inliersIndices;

    segmentation.segment(inliersIndices,*coefficients);

    if (inliersIndices.indices.size() == 0)
    {
        std::cout << "Sphere Not Detected: \n";
        return false;
    }
    else
    {
        //Extract Sphere Data
        sphereData.m_coordinates.x() = coefficients->values[0];
        sphereData.m_coordinates.y() = coefficients->values[1];
        sphereData.m_coordinates.z() = coefficients->values[2];
        sphereData.m_radius = coefficients->values[3];

        for (size_t i = 0; i < inliersIndices.indices.size(); ++i)
        {
            cloud->points[inliersIndices.indices[i]].r = 255;
            cloud->points[inliersIndices.indices[i]].g = 0;
            cloud->points[inliersIndices.indices[i]].b = 0;
        }

        return true;
    }

}
//This function uses the ransac without the segmentation in tests the sphere detection performance following this procedure is inferior.
//bool DetectSphere1(pcl::PointCloud<Point_t>::Ptr cloud)
//{
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//    // created RandomSampleConsensus object and compute the appropriated model
//      pcl::SampleConsensusModelSphere<Point_t>::Ptr
//        model_s(new pcl::SampleConsensusModelSphere<Point_t> (cloud));

//       std::vector<int> inliersIndices;
//       pcl::RandomSampleConsensus<Point_t> ransac (model_s);
//       ransac.setDistanceThreshold (0.01);
//       ransac.computeModel();
//       ransac.getInliers(inliersIndices);

////    pcl::SACSegmentation<Point_t> segmentation;

////    segmentation.setInputCloud(cloud);
////    //segmentation.setInputNormals(cloud);
////    //segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
////    segmentation.setModelType(pcl::SACMODEL_SPHERE);
////    segmentation.setMethodType(pcl::SAC_RANSAC);
////    segmentation.setDistanceThreshold(0.01);
////    segmentation.setOptimizeCoefficients(true);
////    segmentation.setRadiusLimits(0.1, 6.0);
////    segmentation.setEpsAngle(15 / (180/3.141592654));
////    segmentation.setMaxIterations(1000000);



//    //segmentation.segment(inliersIndices,*coefficients);

//    if (inliersIndices.size() == 0)
//    {
//        std::cout << "Sphere Not Detected: \n";
//        return false;
//    }
//    else
//    {
//        for (size_t i = 0; i < coefficients->values.size(); ++i)
//        {
//            std::cout << "coeff: " << i+1 << coefficients->values[i] << "\n";
//        }
//        std::cout << "inliersIndices Found: " << inliersIndices.size() << "\n";
//        for (size_t i = 0; i < inliersIndices.size(); ++i)
//        {
//            cloud->points[inliersIndices[i]].r = 0;
//            cloud->points[inliersIndices[i]].g = 0;
//            cloud->points[inliersIndices[i]].b = 255;
//        }

//        return true;
//    }

//}

//Sphere Detection using normals not working
bool DetectSphere(pcl::PointCloud<Point_t>::Ptr cloud, pcl::PointCloud<NPoint_t>::Ptr normals)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<Point_t,NPoint_t> segmentation;

    segmentation.setInputCloud(cloud);
    segmentation.setInputNormals(normals);
    segmentation.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    //segmentation.setModelType(pcl::SACMODEL_SPHERE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.1);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setRadiusLimits(0.1, 6.0);
    segmentation.setEpsAngle(15 / (180/3.141592654));
    segmentation.setMaxIterations(100000);

    pcl::PointIndices inliersIndices;

    segmentation.segment(inliersIndices,*coefficients);

    if (inliersIndices.indices.size() == 0)
    {
        std::cout << "Sphere Not Detected: \n";
        return false;
    }
    else
    {
        std::cout << "Sphere Found Cloud Point with normals model" << std::endl;

        for (size_t i = 0; i < coefficients->values.size(); ++i)
        {
            std::cout << "coeff: " << i+1 << coefficients->values[i] << "\n";
        }
        std::cout << "inliersIndices Found: " << inliersIndices.indices.size() << "\n";
        for (size_t i = 0; i < inliersIndices.indices.size(); ++i)
        {
            cloud->points[inliersIndices.indices[i]].r = 0;
            cloud->points[inliersIndices.indices[i]].g = 0;
            cloud->points[inliersIndices.indices[i]].b = 255;
        }

        return true;
    }

}

#endif // SPHERESDETECTIONUTILS_H

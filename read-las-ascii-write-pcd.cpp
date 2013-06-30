#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/compute_average_spacing.h>
#include <boost/tuple/tuple.hpp>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;

// Data type := index, followed by the point, followed by three integers that
// define the Red Green Blue color of the point.
typedef boost::tuple<int, Point> IndexedPoint;

int 
main (int argc, char** argv)
{
    std::vector<IndexedPoint> points_CGAL;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::ifstream infile("145E-clean-returns-spaces.xyz");  // Input file for CGAL
    std::ifstream infile_pcl("145E-clean-returns.xyz");     // Input file for PCL
    std::string line = "";
    std::string delims = ",";
    std::vector<std::string> vector_las;

    //
    if (!infile || !CGAL::read_xyz_points (infile, std::back_inserter(points_CGAL), CGAL::Nth_of_tuple_property_map<1,IndexedPoint>()))
    {
      std::cerr << "Error: cannot read file data/sphere_20k.xyz" << std::endl;
    }

    //
    for(unsigned int i = 0; i < points_CGAL.size(); i++)
    {
      points_CGAL[i].get<0>() = i;   // set index value of tuple to i
    }

    // Computes average spacing.
    const unsigned int nb_neighbors = 6; // 1 ring
    FT average_spacing = CGAL::compute_average_spacing(
                          points_CGAL.begin(), points_CGAL.end(),
                          CGAL::Nth_of_tuple_property_map<1,IndexedPoint>(),
                          nb_neighbors);
    std::cout << "Average spacing: " << average_spacing << std::endl;

    // Read each line
    while (std::getline(infile_pcl, line))
    {
        boost::trim(line);
        boost::split(vector_las, line, boost::is_any_of(delims), boost::token_compress_on);
        cloud.push_back (pcl::PointXYZ (float (atof (vector_las[0].c_str ())), 
                                        float (atof (vector_las[1].c_str ())), 
                                        float (atof (vector_las[2].c_str ()))));
    }
    // Populate cloud parameters
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);
    // Write output file
    pcl::io::savePCDFileASCII ("145E-clean-returns.pcd", cloud);
    // Load input file into a PointCloud<T> with appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mls (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load 145E.pcd 
    pcl::io::loadPCDFile ("145E-clean-returns.pcd", *cloud_mls);
    // TODO
    for (size_t i = 0; i < cloud_mls->points.size (); ++i)
        std::cout << "    " << cloud_mls->points[i].x
                  << " "    << cloud_mls->points[i].y
                  << " "    << cloud_mls->points[i].z << std::endl;

    // Create KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Initialize object
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    // Compute normals
    mls.setComputeNormals (true);
    // Set parameters
    mls.setInputCloud (cloud_mls);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (average_spacing);
    // Reconstruct
    mls.process (mls_points);
    // Save output
    pcl::io::savePCDFile ("145E-clean-returns-mls.pcd", mls_points);
}
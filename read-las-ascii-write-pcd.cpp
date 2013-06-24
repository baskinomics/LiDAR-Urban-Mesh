#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::ifstream infile("145E.xyz");
    int las_num_points;
    std::string line = "";
    std::string delims = ",";
    std::vector<std::string> vector_las;

    while (std::getline(infile, line))
    {
        boost::trim (line);
        boost::split(vector_las, line, boost::is_any_of(delims), boost::token_compress_on);
        cloud.push_back (pcl::PointXYZ (float (atof (vector_las[0].c_str ())), 
                                        float (atof (vector_las[1].c_str ())), 
                                        float (atof (vector_las[2].c_str ()))));
    }
    
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    pcl::io::savePCDFileASCII ("145E.pcd", cloud);
}
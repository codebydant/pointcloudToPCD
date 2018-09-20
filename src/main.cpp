/*********************************
           HEADERS
**********************************/
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply/ply.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/range_image/range_image.h>
#include <iostream>
#include <fstream>
#include <string>

void printUsage (const char* progName){
  std::cout << "\nUsage: " << progName << " <file.ply> or <file.txt> or <file.xyz>"  << std::endl <<
               "[q] to exit" << std::endl;
}

using namespace std;

int main(int argc, char **argv){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;
  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  if(argc < 2 or argc > 2){
      printUsage (argv[0]);
      return -1;
  }

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){
                  printUsage (argv[0]);
                  return -1;
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
      file_is_ply = true;
  }else{
      printUsage (argv[0]);
      return -1;
  }

  if(file_is_pcd){
 
      pcl::console::print_info("\nFound pcd file.\n");
      return -1;
    }else if(file_is_ply){
      if(pcl::io::loadPolygonFile(argv[filenames[0]], cl) < 0){
          std::cout << "Error loading point cloud " << argv[filenames[0]] << "\n";
          printUsage (argv[0]);
          return -1;
        }else{
          pcl::io::loadPLYFile(argv[filenames[0]], *cloud);
          //pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          pcl::console::print_info("\nFound ply file.\n");
          pcl::console::print_info ("[done, ");
          pcl::console::print_value ("%g", tt.toc ());
          pcl::console::print_info (" ms : ");
          pcl::console::print_value ("%d", cloud->size ());
          pcl::console::print_info (" points]\n");
          
        }
    }else if(file_is_txt or file_is_xyz){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      double x_,y_,z_;
      while(file >> x_ >> y_ >> z_){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;
          cloud->points.push_back(pt);
      }

      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");

    }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;  

  pcl::console::print_info ("saved point cloud.\n");
  pcl::io::savePCDFileBinary("prueba.pcd",*cloud);

  return 0;
}


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <output_bt_file> [resolution_in_meters]" << std::endl;
        std::cerr << "Example: " << argv[0] << " input.pcd output.bt 0.05" << std::endl;
        return 1;
    }

    std::string pcd_filename = argv[1];
    std::string bt_filename = argv[2];
    double resolution = (argc >= 4) ? std::atof(argv[3]) : 0.05; // Default 5cm resolution

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Loading point cloud from: " << pcd_filename << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename, *cloud) == -1) {
        std::cerr << "Error: Could not read PCD file: " << pcd_filename << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded " << cloud->points.size() << " points." << std::endl;

    // Create OctoMap OcTree
    std::cout << "Creating OctoMap with resolution: " << resolution << " m..." << std::endl;
    octomap::OcTree tree(resolution);

    // Insert point cloud into OcTree
    octomap::Pointcloud octomap_cloud;
    for (const auto& point : cloud->points) {
        // Skip invalid points (NaNs/Infs)
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            octomap_cloud.push_back(point.x, point.y, point.z);
        }
    }

    // Insert scan with origin at (0,0,0)
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);
    std::cout << "Inserting points into OcTree..." << std::endl;
    size_t total_points = octomap_cloud.size();
    size_t processed = 0;

    for (octomap::Pointcloud::const_iterator it = octomap_cloud.begin(); it != octomap_cloud.end(); ++it) {
        // Insert individual ray from sensor origin to point
        tree.insertRay(sensor_origin, *it);
    
        processed++;
        if (processed % 500000 == 0 || processed == total_points) {
            float percent = (static_cast<float>(processed) / static_cast<float>(total_points)) * 100.0f;
            std::cout << "\rProgress: " << processed << " / " << total_points 
                    << " points (" << static_cast<int>(percent) << "%)" << std::flush;
        }
    }
    std::cout << std::endl;

    // Compress map representation
    tree.updateInnerOccupancy();

    // Save binary OctoMap
    std::cout << "Writing OctoMap to: " << bt_filename << std::endl;
    if (tree.writeBinary(bt_filename)) {
        std::cout << "Success! Saved OctoMap with " << tree.size() << " nodes." << std::endl;
    } else {
        std::cerr << "Error writing to destination file: " << bt_filename << std::endl;
        return -1;
    }

    return 0;
}

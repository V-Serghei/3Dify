#include "PointCloudTo3DModel.h"
#include <sstream>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <jni.h>
#include <fstream>

std::string PointCloudTo3DModel::processPointCloud(const std::vector<std::vector<float>>& pointArrays) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Combine point arrays into a single point cloud
    for (const auto& array : pointArrays) {
        for (size_t i = 0; i < array.size(); i += 3) {
            pcl::PointXYZ point;
            point.x = array[i];
            point.y = array[i + 1];
            point.z = array[i + 2];
            cloud->points.push_back(point);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    // Filter and smooth the point cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);

    // Triangulate the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Save the OBJ file
    std::string filePath = "/data/data/com.example.nativelib/files/model.obj";
    pcl::io::saveOBJFile(filePath, triangles);

    return filePath;  // Return the path to the saved file
}

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz, jobjectArray pointArrays) {
    std::vector<std::vector<float>> pointVectors;

    // Convert jobjectArray to std::vector<std::vector<float>>
    jsize arrayLength = env->GetArrayLength(pointArrays);
    for (jsize i = 0; i < arrayLength; ++i) {
        jfloatArray floatArray = (jfloatArray)env->GetObjectArrayElement(pointArrays, i);
        jfloat* floatArrayElements = env->GetFloatArrayElements(floatArray, nullptr);
        jsize floatArrayLength = env->GetArrayLength(floatArray);

        std::vector<float> points;
        for (jsize j = 0; j < floatArrayLength; ++j) {
            points.push_back(floatArrayElements[j]);
        }

        env->ReleaseFloatArrayElements(floatArray, floatArrayElements, 0);
        pointVectors.push_back(points);
    }

    std::string filePath = PointCloudTo3DModel::processPointCloud(pointVectors);
    return env->NewStringUTF(filePath.c_str());  // Return the path to the saved file as jstring
}

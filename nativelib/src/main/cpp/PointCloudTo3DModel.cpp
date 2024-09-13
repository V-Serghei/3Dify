#include <jni.h>
#include <vector>
#include <string>
#include <pcl/io/obj_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <pcl/surface/poisson.h>

#include <fcntl.h>
#include <unistd.h>
#include "PointCloudTo3DModel.h"


bool PointCloudTo3DModel::processPointCloud(const std::vector<std::vector<float>>& pointArrays, const std::string& filePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(10);
    poisson.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    if (pcl::io::saveOBJFile(filePath, mesh) == 0) {
        return true;
    } else {
        std::cerr << "Error saving OBJ file to " << filePath << std::endl;
        return false;
    }
}

//bool PointCloudTo3DModel::processPointCloud(const std::vector<std::vector<float>>& pointArrays, const std::string& filePath) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (const auto& array : pointArrays) {
//        for (size_t i = 0; i < array.size(); i += 3) {
//            pcl::PointXYZ point;
//            point.x = array[i];
//            point.y = array[i + 1];
//            point.z = array[i + 2];
//            cloud->points.push_back(point);
//        }
//    }
//
//    cloud->width = static_cast<uint32_t>(cloud->points.size());
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.2f, 0.2f, 0.2f);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    sor.filter(*cloud_filtered);
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    ne.setInputCloud(cloud_filtered);
//    ne.setSearchMethod(tree);
//    ne.setKSearch(20);
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    ne.compute(*normals);
////    normals->push_back(pcl::Normal(0, 0, -1));  // Нижняя грань
////    normals->push_back(pcl::Normal(0, 0, -1));  // Нижняя грань
////    normals->push_back(pcl::Normal(0, 0, -1));  // Нижняя грань
////    normals->push_back(pcl::Normal(0, 0, -1));  // Нижняя грань
////
////    normals->push_back(pcl::Normal(0, 0, 1));   // Верхняя грань
////    normals->push_back(pcl::Normal(0, 0, 1));   // Верхняя грань
////    normals->push_back(pcl::Normal(0, 0, 1));   // Верхняя грань
////    normals->push_back(pcl::Normal(0, 0, 1));   // Верхняя грань
//
//    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
//
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud(cloud_with_normals);
//
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//    gp3.setSearchRadius(0.05);
//    gp3.setMu(2.0);
//    gp3.setMaximumNearestNeighbors(200);
//    gp3.setMaximumSurfaceAngle(M_PI / 2);
//    gp3.setMinimumAngle(M_PI / 36);
//    gp3.setMaximumAngle(2 * M_PI / 3);
//    gp3.setNormalConsistency(false);
//
//    gp3.setInputCloud(cloud_with_normals);
//    gp3.setSearchMethod(tree2);
//    gp3.reconstruct(triangles);
//
//
//    if (pcl::io::saveOBJFile(filePath, triangles) == 0) {
//        return true;
//    } else {
//        std::cerr << "Error saving OBJ file to " << filePath << std::endl;
//        return false;
//    }
//}


extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloudToUri(JNIEnv *env, jobject thiz,
                                                                 jobjectArray point_arrays, jint fileDescriptor) {
    std::vector<std::vector<float>> pointArrays;

    int arrayLength = env->GetArrayLength(point_arrays);
    for (int i = 0; i < arrayLength; i++) {
        jfloatArray pointArray = (jfloatArray) env->GetObjectArrayElement(point_arrays, i);
        jfloat *points = env->GetFloatArrayElements(pointArray, nullptr);
        int length = env->GetArrayLength(pointArray);

        std::vector<float> pointData(points, points + length);
        pointArrays.push_back(pointData);

        env->ReleaseFloatArrayElements(pointArray, points, JNI_ABORT);
    }

    int fd = fileDescriptor;
    FILE* file = fdopen(fd, "w");

    if (file == nullptr) {
        std::cerr << "Не удалось открыть файл через дескриптор" << std::endl;
        return JNI_FALSE;
    }

    bool result = PointCloudTo3DModel::processPointCloud(pointArrays, "/proc/self/fd/" + std::to_string(fd));

    fclose(file);

    return result ? JNI_TRUE : JNI_FALSE;
}


//extern "C"
//JNIEXPORT jboolean JNICALL
//Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz,
//                                                            jobjectArray point_arrays, jstring filePath) {
//    std::vector<std::vector<float>> pointArrays;
//
//    int arrayLength = env->GetArrayLength(point_arrays);
//    for (int i = 0; i < arrayLength; i++) {
//        jfloatArray pointArray = (jfloatArray) env->GetObjectArrayElement(point_arrays, i);
//        jfloat *points = env->GetFloatArrayElements(pointArray, nullptr);
//        int length = env->GetArrayLength(pointArray);
//
//        std::vector<float> pointData(points, points + length);
//        pointArrays.push_back(pointData);
//
//        env->ReleaseFloatArrayElements(pointArray, points, JNI_ABORT);
//    }
//
//    const char *filePathChars = env->GetStringUTFChars(filePath, nullptr);
//    std::string filePathStr(filePathChars);
//    env->ReleaseStringUTFChars(filePath, filePathChars);
//
//    bool result = PointCloudTo3DModel::processPointCloud(pointArrays, filePathStr);
//
//    return result ? JNI_TRUE : JNI_FALSE;
//}

//std::string PointCloudTo3DModel::processPointCloud(const std::vector<std::vector<float>>& pointArrays) {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (const auto& array : pointArrays) {
//        for (size_t i = 0; i < array.size(); i += 3) {
//            pcl::PointXYZ point;
//            point.x = array[i];
//            point.y = array[i + 1];
//            point.z = array[i + 2];
//            cloud->points.push_back(point);
//        }
//    }
//
//    cloud->width = static_cast<uint32_t>(cloud->points.size());
//    cloud->height = 1;
//    cloud->is_dense = true;
//
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.2f, 0.2f, 0.2f);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    sor.filter(*cloud_filtered);
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    ne.setInputCloud(cloud_filtered);
//    ne.setSearchMethod(tree);
//    ne.setKSearch(20);
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    ne.compute(*normals);
//
//    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
//
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud(cloud_with_normals);
//
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//    gp3.setSearchRadius(0.025);
//    gp3.setMu(2.5);
//    gp3.setMaximumNearestNeighbors(100);
//    gp3.setMaximumSurfaceAngle(M_PI / 4);
//    gp3.setMinimumAngle(M_PI / 18);
//    gp3.setMaximumAngle(2 * M_PI / 3);
//    gp3.setNormalConsistency(false);
//
//    gp3.setInputCloud(cloud_with_normals);
//    gp3.setSearchMethod(tree2);
//    gp3.reconstruct(triangles);
//
//    char tmpFileName[L_tmpnam];
//    std::tmpnam(tmpFileName);
//    std::string filePath(tmpFileName);
//

//    if (pcl::io::saveOBJFile(filePath, triangles) == 0) {
//        return filePath;
//    } else {
//        std::cerr << "Error saving OBJ file" << std::endl;
//        std::remove(filePath.c_str());
//        return "";
//    }
//}
//
//extern "C"
//JNIEXPORT jstring JNICALL
//Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz,
//                                                            jobjectArray point_arrays) {
//    std::vector<std::vector<float>> pointArrays;
//
//    int arrayLength = env->GetArrayLength(point_arrays);
//    for (int i = 0; i < arrayLength; i++) {
//        jfloatArray pointArray = (jfloatArray) env->GetObjectArrayElement(point_arrays, i);
//        jfloat *points = env->GetFloatArrayElements(pointArray, nullptr);
//        int length = env->GetArrayLength(pointArray);
//
//        std::vector<float> pointData(points, points + length);
//        pointArrays.push_back(pointData);
//
//        env->ReleaseFloatArrayElements(pointArray, points, JNI_ABORT);
//    }
//
//    std::string resultPath = PointCloudTo3DModel::processPointCloud(pointArrays);
//
//    if (resultPath.empty()) {
//        return env->NewStringUTF("Ошибка при сохранении файла.");
//    }
//    return env->NewStringUTF(resultPath.c_str());
//}

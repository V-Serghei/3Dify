#ifndef POINTCLOUDTO3DMODEL_H
#define POINTCLOUDTO3DMODEL_H

#include <jni.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>

class PointCloudTo3DModel {
public:
    static std::string processPointCloud(const std::vector<std::vector<float>>& pointArrays);
};

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz, jobjectArray pointArrays);

#endif // POINTCLOUDTO3DMODEL_H

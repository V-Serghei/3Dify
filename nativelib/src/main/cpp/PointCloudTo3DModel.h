#ifndef INC_3DIFY_POINTCLOUDTO3DMODEL_H
#define INC_3DIFY_POINTCLOUDTO3DMODEL_H

#include <jni.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudTo3DModel {
public:


    static std::string processPointCloud();
};

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz);

#endif // INC_3DIFY_POINTCLOUDTO3DMODEL_H
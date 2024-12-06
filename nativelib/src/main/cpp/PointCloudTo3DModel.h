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
    static bool processPointCloud(const std::vector<std::vector<float>>& pointArrays, const std::string& filePath) ;


    };

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloudToUri(JNIEnv *env, jobject thiz,
                                                                 jobjectArray point_arrays, jint fileDescriptor);



#endif // POINTCLOUDTO3DMODEL_H

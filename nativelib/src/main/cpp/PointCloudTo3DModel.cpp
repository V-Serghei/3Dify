

#include "PointCloudTo3DModel.h"

std::string PointCloudTo3DModel::processPointCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (float i = -1.0; i <= 1.0; i += 0.1) {
        for (float j = -1.0; j <= 1.0; j += 0.1) {
            pcl::PointXYZ point;
            point.x = i;
            point.y = j;
            point.z = 0.0;
            cloud->points.push_back(point);
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    std::ostringstream oss;
    oss << "Original point cloud size: " << cloud->points.size() << ", ";

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);

    oss << "Filtered point cloud size: " << cloud_filtered->points.size();

    return oss.str();
}

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_nativelib_Model3DCreator_processPointCloud(JNIEnv *env, jobject thiz) {
    std::string result = PointCloudTo3DModel::processPointCloud();
    return env->NewStringUTF(result.c_str());
}

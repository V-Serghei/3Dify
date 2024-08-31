#include <jni.h>
#include <string>

using namespace std

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_pclibrarycpp_NativeLib_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_pclibrarycpp_NativeLib_processPointCloud(
        JNIEnv* env,
        jobject /* this */,
        jfloatArray point_cloud_array) {

    // 1. Получение данных облака точек из Java массива
    jfloat *pointCloudData = env->GetFloatArrayElements(point_cloud_array, NULL);
    jsize arrayLength = env->GetArrayLength(point_cloud_array);
    // 2. Создание облака точек PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 3. Заполнение облака точек данными из Java массива
    for (int i = 0; i < arrayLength / 3; ++i) {
        pcl::PointXYZ point;
        point.x = pointCloudData[i * 3];
        point.y = pointCloudData[i * 3 + 1];
        point.z = pointCloudData[i * 3 + 2];
        cloud->points.push_back(point);
    }

    // 4. Реконструкция поверхности (пример с использованием Concave Hull)
    pcl::PolygonMesh mesh;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.setAlpha(0.1); // Настройте параметр alpha по необходимости
    chull.reconstruct(mesh);

    // 5. Сохранение модели в формате OBJ (или другой формат)
    pcl::io::saveOBJFile("my_model.obj", mesh);

    env->ReleaseFloatArrayElements(point_cloud_array, pointCloudData, 0);

    std::string result = "PointCloud processed and saved to my_model.obj";
    return env->NewStringUTF(result.c_str());
}
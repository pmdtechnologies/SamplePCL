/****************************************************************************\
* Copyright (C) 2022 Infineon Technologies & pmdtechnologies ag
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#include <royale.hpp>

#include <condition_variable>

#include <algorithm>
#include <mutex>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/organized_edge_detection.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;

#define M_COLOR_LOOKUP_SIZE 180

namespace
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensity;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlanes;
    pcl::PointCloud<pcl::PointXYZRGB> ::Ptr cloudDuplicate;
    mutex cloudMutex;
    std::condition_variable cloudCV;
    bool newDataAvailable;
    bool showNormals;
    bool removeOutliers;
    bool bilateral;
    bool median;
    bool shadowpoints;
    bool voxelgrid;
    bool condition;
    bool growRegion;
    bool detectPlanes;

    typedef struct RgbColor
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } RgbColor;

    typedef struct HsvColor
    {
        uint8_t h;
        uint8_t s;
        uint8_t v;
    } HsvColor;

    RgbColor m_colorLookup[M_COLOR_LOOKUP_SIZE];

    RgbColor HsvToRgb(const HsvColor &hsv)
    {
        RgbColor rgb;
        uint16_t region, remainder;
        uint8_t p, q, t;

        if (hsv.s == 0)
        {
            rgb.r = hsv.v;
            rgb.g = hsv.v;
            rgb.b = hsv.v;
            return rgb;
        }

        region = hsv.h / 43;
        remainder = static_cast<uint16_t> ((hsv.h % 43) * 6);

        p = static_cast<uint8_t> ((hsv.v * (255 - hsv.s)) >> 8);
        q = static_cast<uint8_t> ((hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8);
        t = static_cast<uint8_t> ((hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8);

        switch (region)
        {
        case 0:
            rgb.r = hsv.v;
            rgb.g = t;
            rgb.b = p;
            break;
        case 1:
            rgb.r = q;
            rgb.g = hsv.v;
            rgb.b = p;
            break;
        case 2:
            rgb.r = p;
            rgb.g = hsv.v;
            rgb.b = t;
            break;
        case 3:
            rgb.r = p;
            rgb.g = q;
            rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t;
            rgb.g = p;
            rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v;
            rgb.g = p;
            rgb.b = q;
            break;
        }

        return rgb;
    }

    const RgbColor &getColor(const float dist)
    {
        const float minDist = 1.3f;
        const float maxDist = 2.0f;

        const float m_spanDist = 1.0f / (maxDist - minDist);

        float clampedDist = std::min(maxDist, dist);
        clampedDist = std::max(minDist, clampedDist);
        int index = std::min<uint16_t>(M_COLOR_LOOKUP_SIZE - 1,
            static_cast<uint16_t> (static_cast<float> (M_COLOR_LOOKUP_SIZE - 1) *
            (clampedDist - minDist) * m_spanDist));

        if (index < 0)
        {
            index = 0;
        }

        return m_colorLookup[index];
    }
} // namespace

class RoyaleListener : public IDepthDataListener
{
public:
    RoyaleListener()
    {
    }

    void onNewData(const DepthData *data)
    {
        {
            std::unique_lock<std::mutex> lock(cloudMutex);
            // Fill in the cloud data
            cloud->width = data->width;
            cloud->height = data->height;
            cloud->is_dense = false;
            cloud->points.resize(cloud->width * cloud->height);
            //duplicate the cloud
            cloudDuplicate->width = data->width;
            cloudDuplicate->height = data->height;
            cloudDuplicate->is_dense = false;
            cloudDuplicate->points.resize(cloud->width * cloud->height);
            // do the same for cloudIntensity
            cloudIntensity->width = data->width;
            cloudIntensity->height = data->height;
            cloudIntensity->is_dense = false;
            cloudIntensity->points.resize(cloudIntensity->width * cloudIntensity->height);

            for (size_t i = 0u; i < cloud->points.size(); ++i)
            {
                cloud->points[i].x = data->getX(i);
                cloud->points[i].y = data->getY(i);
                cloud->points[i].z = data->getZ(i);

                cloudDuplicate->points[i].x = data->getX(i);
                cloudDuplicate->points[i].y = data->getY(i);
                cloudDuplicate->points[i].z = data->getZ(i);

                cloudIntensity->points[i].x = data->getX(i);
                cloudIntensity->points[i].y = data->getY(i);
                cloudIntensity->points[i].z = data->getZ(i);

                const RgbColor col = getColor(data->getZ(i));

                cloud->points[i].r = col.r;
                cloud->points[i].g = col.g;
                cloud->points[i].b = col.b;

                cloudDuplicate->points[i].r = col.r;
                cloudDuplicate->points[i].g = col.g;
                cloudDuplicate->points[i].b = col.b;

                // calculate intensity from color
                cloudIntensity->points[i].intensity = data->getGrayValue(i) / 2000.0f;

                if (data->getDepthConfidence(i) > 0)
                {
                    cloud->points[i].a = 255;
                    cloudDuplicate->points[i].a = 255;
                }
                else
                {
                    cloud->points[i].a = 0;
                    // if the point is invalid, mark it with a special value
                    cloud->points[i].x = cloud->points[i].y = cloud->points[i].z =
                        std::numeric_limits<float>::quiet_NaN();
                        
                    cloudDuplicate->points[i].a = 0;
                }
            }

            // notify the waiting loop in the main thread
            newDataAvailable = true;
        }
        cloudCV.notify_all();
    }
};

void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    if (event.getKeySym() == "n" && event.keyDown())
    {
        // toggle computation of normals
        showNormals = !showNormals;
    }
    if (event.getKeySym() == "m" && event.keyDown())
    {
        // toggle statistical outlier removal
        removeOutliers = !removeOutliers;
    }
    else if (event.getKeySym() == "b" && event.keyDown())
    {
        // toggle bilateral filtering
        bilateral = !bilateral;
    }
    else if (event.getKeySym() == "v" && event.keyDown())
    {
        // toggle median filtering
        median = !median;
    }
    else if (event.getKeySym() == "a" && event.keyDown())
    {
        // toggle shadowpoint removal
        shadowpoints = !shadowpoints;
    }
    else if (event.getKeySym() == "d" && event.keyDown())
    {
        // toggle downsampling
        voxelgrid = !voxelgrid;
    }
    else if (event.getKeySym() == "y" && event.keyDown())
    {
        // toggle conditional removal
        condition = !condition;
    }
    else if (event.getKeySym() == "k" && event.keyDown())
    {
        // toggle region growing
        growRegion = !growRegion;
    }
    else if (event.getKeySym() == "p" && event.keyDown())
    {
        // toggle plane detection
        detectPlanes = !detectPlanes;
    }
}

int main(int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    RoyaleListener listener;

    for (auto i = 0u; i < M_COLOR_LOOKUP_SIZE; ++i)
    {
        auto h = static_cast<uint8_t> (i);

        HsvColor tempHsv;
        tempHsv.h = h;
        tempHsv.s = 255;
        tempHsv.v = 255;

        m_colorLookup[i] = HsvToRgb(tempHsv);
    }

    newDataAvailable = false;
    showNormals = false;
    removeOutliers = false;
    bilateral = false;
    median = false;
    shadowpoints = false;
    voxelgrid = false;
    condition = false;
    growRegion = false;
    detectPlanes = false;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        // check the number of arguments
        if (argc > 1)
        {
            // if the program was called with an argument try to open this as a file
            cout << "Trying to open : " << argv[1] << endl;
            cameraDevice = manager.createCamera(argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
            cout << "Detected " << camlist.size() << " camera(s)." << endl;

            if (!camlist.empty())
            {
                cameraDevice = manager.createCamera(camlist[0]);
            }
            else
            {
                cerr << "No suitable camera device detected." << endl
                    << "Please make sure that a supported camera is plugged in, all drivers are "
                    << "installed, and you have proper USB permission" << endl;
                return 1;
            }

            camlist.clear();
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        if (argc > 1)
        {
            // there was a problem opening the file
            cerr << "Could not open " << argv[1] << endl;
            return 1;
        }
        else
        {
            // we couldn't open any camera
            cerr << "Cannot create the camera device" << endl;
            return 1;
        }
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
        return 1;
    }

    // create PointCloud objects that will hold the original data and the filtered one
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudIntensity.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudPlanes.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudDuplicate.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setCameraPosition(0.0f, 0.0f, -2.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f);

    viewer.setShowFPS(false);

    // we want to be able to switch between filtered and unfiltered
    // point clouds, that's why we need to register a callback for keyboard events
    viewer.registerKeyboardCallback(keyboardEvent, (void *)&viewer);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_planes(cloudPlanes);

    // register a data listener
    if (cameraDevice->registerDataListener(&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    // start capturing from the device/file
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // first plane red, second plane blue, third plane green, fourth plane magenta
    const std::uint8_t R[20] = { 255, 0, 0, 255, 127, 200, 255, 0, 0, 0, 255, 0, 0, 255, 127, 200, 255, 0, 0, 0 };
    const std::uint8_t G[20] = { 0, 0, 255, 0, 0, 255, 255, 0, 0, 0, 0, 0, 255, 0, 0, 255, 255, 0, 0, 0 };
    const std::uint8_t B[20] = { 0, 255, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 0, 255, 255, 255, 255, 0, 0, 0 };

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    std::cout << "Below you see which key toggles which filter : \n - n : show normals\n - m : remove outliers\n - b : bilateral filter\n - v : median filter\n - a : remove shadow points\n - d : voxelgrid\n - y : conditional removal\n - j : take screenshot (implemented by pcl)\n - k : region growing\n - p : plane detection" << std::endl;

    while (!viewer.wasStopped())
    {
        // while the viewer window is not closed, wait for new data to arrive
        std::unique_lock<std::mutex> lock(cloudMutex);
        auto timeOut = (std::chrono::system_clock::now() + std::chrono::milliseconds(100));
        if (cloudCV.wait_until(lock, timeOut, [&] { return newDataAvailable; }))
        {
            if (removeOutliers)
            {
                // set up a point cloud outlier removal and show the result
                StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierRemoval;
                outlierRemoval.setMeanK(50);
                outlierRemoval.setStddevMulThresh(1.0);
                outlierRemoval.setInputCloud(cloud);
                outlierRemoval.filter(*cloud);
            }

            if (bilateral)
            {
                // attention: this filter requires intensity values, so use a cloud based on PointXYZI instead of PointXYZRGB
                BilateralFilter<pcl::PointXYZI> bilateralFilter;
                bilateralFilter.setHalfSize(0.2);
                bilateralFilter.setStdDev(0.2);
                bilateralFilter.filter(*cloudIntensity);

                // display cloudIntensity
                viewer.removeAllPointClouds();
                if (!viewer.updatePointCloud(cloud, "Bilateral Filtered"))
                {
                    viewer.addPointCloud<pcl::PointXYZI>(cloudIntensity, "Bilateral Filtered", 0);
                }
            }

            if (median)
            {
                // filter the data with a median filter with window size 11
                MedianFilter<pcl::PointXYZRGB> medianFilter;
                medianFilter.setInputCloud(cloud);
                medianFilter.setWindowSize(11);
                medianFilter.filter(*cloud);
            }

            if (shadowpoints)
            {
                // compute normals first
                ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
                ne.setMaxDepthChangeFactor(2.2f);
                ne.setNormalSmoothingSize(5.0f);
                ne.setInputCloud(cloud);
                ne.compute(*normals);

                double shadowThreshold = 0.9;
                ShadowPoints<pcl::PointXYZRGB, pcl::Normal> shadowPoints;
                shadowPoints.setNormals(normals);
                shadowPoints.setThreshold(shadowThreshold);
                shadowPoints.setInputCloud(cloud);
                shadowPoints.filter(*cloud);
            }

            if (voxelgrid)
            {
                ApproximateVoxelGrid<pcl::PointXYZRGB> appVoxelgrid;
                appVoxelgrid.setInputCloud(cloud);
                appVoxelgrid.setLeafSize(0.05, 0.05, 0.05);
                appVoxelgrid.setDownsampleAllData(true);
                appVoxelgrid.filter(*cloud);
            }

            if (condition)
            {
                // set condition: z > 0.0 and z < 1.5
                ConditionAnd<pcl::PointXYZRGB>::Ptr c(new pcl::ConditionAnd<pcl::PointXYZRGB>);
                c->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));
                c->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 1.5)));

                // remove points that don't meet the condition
                ConditionalRemoval<pcl::PointXYZRGB> conditionalRemoval;
                conditionalRemoval.setInputCloud(cloud);
                conditionalRemoval.setCondition(c);
                conditionalRemoval.filter(*cloud);
            }

            if (growRegion)
            {
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
                normal_estimator.setSearchMethod(tree);
                normal_estimator.setInputCloud(cloud);
                normal_estimator.setKSearch(50);
                normal_estimator.compute(*normals);

                cloudFiltered->points.resize(cloud->size());
                for (size_t i = 0; i < cloud->size(); i++)
                {
                    cloudFiltered->points[i].x = cloud->points[i].x;
                    cloudFiltered->points[i].y = cloud->points[i].y;
                    cloudFiltered->points[i].z = cloud->points[i].z;
                    cloudFiltered->points[i].r = 150; 
                    cloudFiltered->points[i].g = 150; 
                    cloudFiltered->points[i].b = 150; 
                }

                viewer.removeAllPointClouds();

                viewer.addPointCloud<pcl::PointXYZRGB>(cloudFiltered, rgb, "Triangulated points");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                    "Triangulated points");

                reg.setMinClusterSize(500);
                reg.setMaxClusterSize(5000);
                reg.setSearchMethod(tree);
                reg.setNumberOfNeighbours(10);
                reg.setInputCloud(cloud);
                reg.setInputNormals(normals);
                reg.setSmoothnessThreshold(3.5 / 180.0 * M_PI);
                reg.setCurvatureThreshold(90.0);

                std::vector<pcl::PointIndices> clusters;
                reg.extract(clusters);

                if (!clusters.empty ())
                {
                    int colIdx = 0;

                    for (auto i = 0; i < clusters.size() ; ++i) // iterate over the found clusters
                        {
                            pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices(clusters[i]));

                            pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
                            extract_indices.setIndices(indicesPtr);
                            extract_indices.setInputCloud(cloud);
                            extract_indices.filter(*cloudFiltered);

                            std::stringstream ss;
                            ss << "Cluster " << colIdx;

                            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> customHandler(
                                cloudFiltered, R[colIdx % 10], G[colIdx % 10], B[colIdx % 10]);
                            viewer.addPointCloud<pcl::PointXYZRGB>(cloudFiltered, customHandler, ss.str());
                            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4,
                                ss.str());

                            colIdx++;
                        }
                }
            }

            if (detectPlanes)
            {
                size_t s = 0u;

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZRGB> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.01);
                seg.setMaxIterations(1000);

                cloudPlanes->points.resize(cloudDuplicate->size());
                for (size_t i = 0; i < cloudDuplicate->size(); i++)
                {
                    cloudPlanes->points[i].x = 0.0f;
                    cloudPlanes->points[i].y = 0.0f;
                    cloudPlanes->points[i].z = 0.0f;
                }

                pcl::ExtractIndices<pcl::PointXYZRGB> extract;

                auto i = 0;
                const int nr_points = (int)cloudDuplicate->size();
                while (cloudDuplicate->size() > 0.1 * nr_points) // While x% of the original cloud is still there
                {
                    // Segment the largest planar component from the remaining cloud
                    seg.setInputCloud(cloudDuplicate);
                    seg.segment(*inliers, *coefficients);
                    if (inliers->indices.size() == 0)
                    {
                        break;
                    }

                    for (auto x : inliers->indices)
                    {
                        cloudPlanes->points[s].x = cloudDuplicate->points[x].x;
                        cloudPlanes->points[s].y = cloudDuplicate->points[x].y;
                        cloudPlanes->points[s].z = cloudDuplicate->points[x].z;
                        cloudPlanes->points[s].r = R[i];
                        cloudPlanes->points[s].g = G[i];
                        cloudPlanes->points[s].b = B[i];
                        ++s;
                    }

                    // Extract the outliers (outliers because we throw all inliers away now, and the remaining
                    // points get the new point cloud in which we search planes)
                    extract.setInputCloud(cloudDuplicate);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>());
                        extract.filter(*cloud_f);
                        cloudDuplicate.swap(cloud_f);
                    }
                    ++i;
                    if (i > 3)
                    {
                        break;
                    }
                }

                std::cout << "s : " << s << " cloudPlanes->size " << cloudPlanes->size() << std::endl;
                // points not belonging to a planar surface get grey-ish
                for (auto i = 0u; s < cloudDuplicate->size(); ++s, ++i)
                {
                    cloudPlanes->points[s].x = cloudDuplicate->points[i].x;
                    cloudPlanes->points[s].y = cloudDuplicate->points[i].y;
                    cloudPlanes->points[s].z = cloudDuplicate->points[i].z;
                    cloudPlanes->points[s].r = 100;
                    cloudPlanes->points[s].g = 100;
                    cloudPlanes->points[s].b = 100;
                }

                // display cloudPlanes
                viewer.removeAllPointClouds();
                if (!viewer.updatePointCloud(cloudPlanes, "Planes"))
                {
                    viewer.addPointCloud<pcl::PointXYZRGB>(cloudPlanes, rgb_planes, "Planes");
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Planes");
                }
            }

            if (!growRegion && !bilateral && !detectPlanes)
            {
                viewer.removeAllPointClouds();

                if (!viewer.updatePointCloud(cloud, "Triangulated points"))
                {
                    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Triangulated points");
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Triangulated points");
                }
            }

            viewer.removePointCloud("normals", 0);
            if (showNormals)
            {
                ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
                ne.setMaxDepthChangeFactor(0.02f);
                ne.setNormalSmoothingSize(10.0f);
                ne.setInputCloud(cloud);
                ne.compute(*normals);

                viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 50, 0.05, "normals");
            }

            newDataAvailable = false;
            viewer.spinOnce();
        }
        else
        {
            // if we ran into a timeout, just sleep for some time
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    // stop capturing
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }

    return 0;
}

# Tutorial - Point Cloud Library
This tutorial shows how to use different functionalities of the Point Cloud Library ([PCL](https://pointclouds.org/documentation/))
together with Royale.

## How to install
To use the Point Cloud Library with Royale you need to install a Royale binary version and additionally
PCL. The easiest way to do this is by using the [all-in-one installer](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.9.1/PCL-1.9.1-AllInOne-msvc2017-win64.exe).
We tested this sample with PCL 1.9.1.

If you're running Debian or Ubuntu, please install the `libvtk6-qt-dev` and `libfontconfig1-dev` packages,
otherwise you might get errors during compilation.

After installing everything, you can start CMake. There you have to set the `PCL_DIR` to your PCL installation (e.g. C:/Program Files/PCL 1.9.1/cmake)
and the `royale_DIR` to the share folder of your Royale binary installation (e.g. D:/Program Files/royale/4.23.0.1062/share) and click **Generate**.

## Quick Start
Below you see which key toggles which filter:
- n: [show normals](#show-normals) 
- m: [remove outliers](#outlier-removal)
- b: [bilateral filter](#bilateral-filter)
- v: [median filter](#median-filter)
- a: [remove shadow points](#shadowpoint-removal)
- d: [voxelgrid](#voxelgrid)
- y: [conditional removal](#conditional-removal)
- j: [take screenshot](#take-screenshot) (implemented by PCL)
- k: [region growing](#region-growing)
- p: [plane detection](#plane-detection) 

## Code explanation
The PCL example shows how to apply different filters or segmentation techniques on a point cloud and how to switch between different clouds
when displaying the results. You can switch between the filters/techniques by pressing keyboard keys. After explaining the setup, we explain the available 
methods and which keyboard key triggers them.

**Attention:** The PCL visualizer offers some functions that are triggered by keyboard events. To see which these are press `h` in the viewers window.

#### Part I Setup
In the beginning we declare a data listener and a camera device. We also declare the platform resources as this will call the `CoInitializeEx` function on Windows.
Otherwise we won't be able to use camera devices that use the UVC standard. We also create a data listener, a color lookup table and initialize some variables.

```cpp
    // Windows requires that the application allocates these, not the DLL.
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

        m_colorLookup[i] = HsvToRgb (tempHsv);
    }

    newDataAvailable = false;
    showNormals = false;
    removeOutliers = false;
    bilateral = false;
    median = false;
    shadowpoints = false;
    voxelgrid = false;
    condition = false;
    takeScreenshot = false;
    growRegion = false;
    detectPlanes = false;
```

###### HsvToRgb
This function converts a given hsv color to the corresponding rgb color.
```cpp
RgbColor HsvToRgb (const HsvColor &hsv)
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
        remainder = static_cast<uint16_t> ( (hsv.h % 43) * 6);

        p = static_cast<uint8_t> ( (hsv.v * (255 - hsv.s)) >> 8);
        q = static_cast<uint8_t> ( (hsv.v * (255 - ( (hsv.s * remainder) >> 8))) >> 8);
        t = static_cast<uint8_t> ( (hsv.v * (255 - ( (hsv.s * (255 - remainder)) >> 8))) >> 8);

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
```

###### data listener
The `RoyaleListener` class has a function `onNewData` which is called for every frame. Inside this function we **fill the point cloud
with the collected data** from the camera device. This happens in another thread. If new data is available we notify the main thread.
As you may notice, we actually fill three clouds: `cloud`, `cloudDuplicate` and `cloudIntensity`.

```cpp
class RoyaleListener: public IDepthDataListener
{
public:
    RoyaleListener ()
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
            //duplicate
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
                cloud->points[i].x = data->points[i].x;
                cloud->points[i].y = data->points[i].y;
                cloud->points[i].z = data->points[i].z;

                cloudDuplicate->points[i].x = data->points[i].x;
                cloudDuplicate->points[i].y = data->points[i].y;
                cloudDuplicate->points[i].z = data->points[i].z;

                cloudIntensity->points[i].x = data->points[i].x;
                cloudIntensity->points[i].y = data->points[i].y;
                cloudIntensity->points[i].z = data->points[i].z;

                const RgbColor col = getColor(data->points[i].z);

                cloud->points[i].r = col.r;
                cloud->points[i].g = col.g;
                cloud->points[i].b = col.b;

                cloudDuplicate->points[i].r = col.r;
                cloudDuplicate->points[i].g = col.g;
                cloudDuplicate->points[i].b = col.b;

                // calculate intensity from color
                cloudIntensity->points[i].intensity = data->points[i].grayValue / 2000.0f;

                if (data->points[i].depthConfidence > 0)
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
        cloudCV.notify_all ();
    }
};
```

Next we **create the camera device** and set it up. You can either connect an usb camera or use an .rrf-file as camera device.

```cpp
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
            cameraDevice = manager.createCamera (argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist (manager.getConnectedCameraList ());
            cout << "Detected " << camlist.size () << " camera(s)." << endl;

            if (!camlist.empty ())
            {
                cameraDevice = manager.createCamera (camlist[0]);
            }
            else
            {
                cerr << "No suitable camera device detected." << endl
                     << "Please make sure that a supported camera is plugged in, all drivers are "
                     << "installed, and you have proper USB permission" << endl;
                return 1;
            }

            camlist.clear ();
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
    auto status = cameraDevice->initialize ();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return 1;
    }
```

Next we **create PointCloud objects** that will hold the original data and the filtered one 
and we also create a cloud that contains intensity values instead of color.

```cpp
    cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    cloudFiltered.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    cloudIntensity.reset (new pcl::PointCloud<pcl::PointXYZI>());
    cloudPlanes.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudDuplicate.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
```

Now we **create a viewer** in which we will display the point clouds. 

```cpp
    pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
    viewer.setCameraPosition (0.0f, 0.0f, -2.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f);
    viewer.setBackgroundColor (0.0, 0.0, 0.0);

    viewer.setShowFPS (false);
```

To be able to switch between different clouds and filters, we **register a keyboard callback**.
We also **define colour handlers** for the clouds we want to display.

```cpp
    viewer.registerKeyboardCallback (keyboardEvent, (void *) &viewer);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb (cloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_planes(cloudPlanes);
```

Next we **register the data listener** and **start capturing**.

```cpp
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    if (cameraDevice->startCapture () != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }
```

Finally we declare another point cloud for the normals and an integral image normal estimator to calculate
them. For the region growing we define different colors for the planes. We also declare the region growing itself and 
a [kd-tree](https://pointclouds.org/documentation/classpcl_1_1_kd_tree.html). 

```cpp
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    
    // first plane red, second plane blue, third plane green, fourth plane magenta
    const std::uint8_t R[20] = {255, 0, 0, 255, 127, 200, 255, 0, 0, 0, 255, 0, 0, 255, 127, 200, 255, 0, 0, 0};
    const std::uint8_t G[20] = {0, 0, 255, 0, 0, 255, 255, 0, 0, 0, 0, 0, 255, 0, 0, 255, 255, 0, 0, 0};
    const std::uint8_t B[20] = {0, 255, 0, 255, 255, 255, 255, 0, 0, 0, 0, 255, 0, 255, 255, 255, 255, 0, 0, 0};

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
```

#### Part II Filters
The **creation of the different filters** is similar for all filters.
1. create instance of the filter class
2. set parameters
3. set input cloud
4. apply the filter

Each filter is toggled through a boolean variable that is connected with a keyboard key. So when you push the key, the value
of the corresponding boolean gets inverted and the filter is turned on or off. 

Please notice, that **some of the filters let the fps drop drastically**, so it may be that you have to wait a bit to see the results.

###### Outlier Removal
[This filter](https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html) is toggled by pressing `m`.
The parameters you have to set here are mean and standard deviation. Feel free to experiment with these values. 
```cpp
            if (removeOutliers)
            {
                // set up a point cloud filter and show the filtered result
                StatisticalOutlierRemoval<pcl::PointXYZRGBA> outlierRemoval;
                outlierRemoval.setMeanK (50);
                outlierRemoval.setStddevMulThresh (1.0);
                outlierRemoval.setInputCloud (cloud);
                outlierRemoval.filter (*cloud);
            }
```

###### Bilateral Filter
[This filter](https://pointclouds.org/documentation/classpcl_1_1_bilateral_filter.html) is toggled by pressing `b`.
For this filter we have to set the standard deviation and the half size. You can experiment with the values if you like. 
Since this filter works on intensity values it needs its own cloud which we display here. 
```cpp
            if (bilateral)
            {
                // attention: this filter requires intensity values, so use a cloud based on PointXYZI instead of PointXYZRGBA
                BilateralFilter<pcl::PointXYZI> bilateralFilter;
                bilateralFilter.setHalfSize (0.2);
                bilateralFilter.setStdDev (0.2);
                bilateralFilter.filter (*cloudIntensity);
                
                // display cloudIntensity 
                viewer.removeAllPointClouds();
                if (!viewer.updatePointCloud(cloud, "Bilateral Filtered"))
                {
                    std::cout << "Intensity" << std::endl;
                    viewer.addPointCloud<pcl::PointXYZI>(cloudIntensity, "Bilateral Filtered", 0);
                }
            }
```

###### Median Filter
[This filter](https://pointclouds.org/documentation/classpcl_1_1_median_filter.html) is toggled by pressing `v`.
For this filter the only parameter that needs to be set is the filter kernel size. You can change this value according to your needs.
```cpp
            if (median)
            {
                // filter the data with a median filter with window size 11
                MedianFilter<pcl::PointXYZRGBA> medianFilter;
                medianFilter.setInputCloud (cloud);
                medianFilter.setWindowSize (11);
                medianFilter.filter (*cloud);
            }
```

###### Shadowpoint Removal
[This filter](https://pointclouds.org/documentation/classpcl_1_1_shadow_points.html) is toggled by pressing `a`.
First we compute the normals in the same way as [here](#ShowNormals). Then we define the threshold which defines whether 
a point is considered as shadow point or not. The normals and the threshold are set and then the filter can be applied. 

```cpp
            if (shadowpoints)
            {
                // compute normals first
                ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
                ne.setMaxDepthChangeFactor (2.2f);
                ne.setNormalSmoothingSize (5.0f);
                ne.setInputCloud (cloud);
                ne.compute (*normals);

                double shadowThreshold = 0.9;
                ShadowPoints<pcl::PointXYZRGBA, pcl::Normal> shadowPoints;
                shadowPoints.setNormals (normals);
                shadowPoints.setThreshold (shadowThreshold);
                shadowPoints.setInputCloud (cloud);
                shadowPoints.filter (*cloud);
            }
```

###### Voxelgrid
[This filter](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html) is toggled by pressing `d`.
The parameters that we need to set are the leaf size and whether we want to downsample all the data or not. 

```cpp
            if (voxelgrid)
            {
                ApproximateVoxelGrid<pcl::PointXYZRGBA> appVoxelgrid;
                appVoxelgrid.setInputCloud (cloud);
                appVoxelgrid.setLeafSize (0.05, 0.05, 0.05);
                appVoxelgrid.setDownsampleAllData (true);
                appVoxelgrid.filter (*cloud);
            }
```

###### Conditional Removal
[This filter](https://pointclouds.org/documentation/classpcl_1_1_conditional_removal.html) is toggled by pressing `y`.
First we need to set the condition that defines which points get removed. In this example we remove all the points with a z-value greater 
than or equal 1.5. This condition needs to be set and then the filter can be applied. You can try other conditions if you like. 

```cpp
            if (condition)
            {
                // set condition: z > 0.0 and z < 1.5
                ConditionAnd<pcl::PointXYZRGBA>::Ptr c (new pcl::ConditionAnd<pcl::PointXYZRGBA>);
                c->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::GT, 0.0)));
                c->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::LT, 1.5)));

                // remove points that don't meet the condition
                ConditionalRemoval<pcl::PointXYZRGBA> conditionalRemoval;
                conditionalRemoval.setInputCloud (cloud);
                conditionalRemoval.setCondition (c);
                conditionalRemoval.filter (*cloud);
            }
```

###### Region Growing
[This technique](https://pointclouds.org/documentation/classpcl_1_1_region_growing.html) is toggled by pressing `k`.
The first step here is to compute the normals. To do so, we use the KD Tree we defined earlier. 
Next we rezize the filtered cloud to match the size of the original cloud and add the points at in a grey colour. 
Then we remove all currently displayed clouds from the viewer and instead add the filtered cloud. 
In the next steps we set a few parameters. Now we are able to extract the clusters. 
For each cluster we then extract the indices and display the clusters (each in a different colour) in the cloud. 

```cpp  
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
```

###### Plane Detection
[This technique](https://pointclouds.org/documentation/classpcl_1_1_random_sample_consensus.html) is toggled by pressing `p`.
We detect the planes iterative: we find the largest plane in the cloud, add it to another cloud and remove it from the original cloud.
Then we detect the new largest plane in the cloud and so on. 
To avoid changing the original cloud, we use a duplicate of it. The planes are displayed in "cloudPlanes". 
First we need to set some parameters for the used RANSAC model. Then we fill the "cloudPlanes" with as many points as the (duplicated) original
loud contains. 
While 10% of the original cloud are still there, we find the largest plane and add its points to the cloudPlanes. All other points remain in the 
duplicated original cloud, so for each new found plane the size of this cloud is reduced and the size of cloudPlanes increases. We assign each plane
a new colour. 
If we break out of this while loop, we colour all remaining points (that do not belong to any plane) grey.
Then we remove all currently displayed clouds and add the cloudPlanes to the viewer. 

```cpp
            size_t s = 0u;
            
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
                for (auto i = 0u; s < cloudFiltered->size(); ++s, ++i)
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
                    std::cout << "Planes" << std::endl;
                    viewer.addPointCloud<pcl::PointXYZRGB>(cloudPlanes, rgb_planes, "Planes");
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Planes");
                }
            }
```

###### Display the clouds
By adding the cloud to the viewer, we display it. Because we have different clouds, we remove clouds that belong
to filters that are not active in the moment. It is possible to add more than one cloud to the viewer, so you can 
for example see the normals in a median filtered image. (This works only for filters that do not use their own cloud.)
The region growing, plane detection and bilateral filter use their own clouds, which are displayed in the corresponding code above.

```cpp
            if (!growRegion & !bilateral & !detectPlanes)
            {
                viewer.removeAllPointClouds();

                if (!viewer.updatePointCloud(cloud, "Triangulated points"))
                {
                    std::cout << "Triangulated" << std::endl;
                    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Triangulated points");
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Triangulated points");
                }
            }
```
            
###### Show Normals
This function is toggled by pressing `n`.
Some parameters need to be set, these are: normal estimation method, depth change factor and normal smoothing size.

```cpp
            viewer.removePointCloud("normals", 0);
            if (showNormals)
            {
                // compute and show normals
                ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
                ne.setMaxDepthChangeFactor (0.02f);
                ne.setNormalSmoothingSize (10.0f);
                ne.setInputCloud (cloud);
                ne.compute (*normals);

                viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals, 50, 0.05, "normals");
            }
```

###### Take Screenshot
This option is triggered by the key `j` and is implemented by the PCL visualizer class.
The screenshots are saved in the same directory as this code.

#### Part III The End
When the viewer window is closed, we stop capturing and the program ends.

```cpp
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }

    return 0;
}
```

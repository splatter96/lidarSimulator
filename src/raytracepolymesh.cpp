/**
 * A Node which provides a service to generate a point cloud
 * based on the stl objects provided
 */

#include <cstdio>
#include <vector>
#include <fstream>
#include <cmath>
#include <chrono>

#include <CL/cl.hpp>

#include "ros/ros.h"
#include <resource_retriever/retriever.h>
#include <tf/transform_datatypes.h>
#include "lidarSimulator/LiDARSimulation.h"

#include "visualization_msgs/Marker.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

#include "geometry.h"

// typedef tf::Vector3 Vec3f;

static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;

ros::Publisher cloudPub;

resource_retriever::Retriever r;

typedef struct tag_Vec
{
    float       x;
    float       y;
    float       z;
} Vec3f;

float dot(Vec3f v1, Vec3f v2){
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vec3f normalize(Vec3f v1){
    float norm = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
    float factor = sqrt(norm);

     Vec3f v = {
                v1.x * factor,
                v1.y * factor,
                v1.z * factor
     };

    return v;
}

Vec3f cross(Vec3f v1, Vec3f v2){
     Vec3f v = {
                v1.x * v2.z - v1.z * v2.y,
                v1.y * v2.x - v1.x * v2.z,
                v1.z * v2.y - v1.y * v2.x
     };

     return v;
}

Vec3f vecAddVec(Vec3f v1, Vec3f v2){
    Vec3f v = {
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z
    };

    return v;
}

Vec3f vecSubVec(Vec3f v1, Vec3f v2){
    Vec3f v = {
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    };

    return v;
}

Vec3f vecMulVecElem(Vec3f v1, Vec3f v2){
    Vec3f v = {
        v1.x * v2.x,
        v1.y * v2.y,
        v1.z * v2.z
    };

    return v;
}

Vec3f vecMulScalar(Vec3f v1, float s){
    Vec3f v = {
        v1.x * s,
        v1.y * s,
        v1.z * s
    };

    return v;
}

/* Struct to hold the ray information
*/
typedef struct tag_ray
{
    Vec3f       origin;
    Vec3f       dir;
} ray;

/**
 * Class to represent a single Triangle
 */
/*
class Triangle
{
    public:
        Triangle() : normal(tf::Vector3()), v1(tf::Vector3()), v2(tf::Vector3()), v3(tf::Vector3()) {}
        Triangle(tf::Vector3 normalp, tf::Vector3 v1p, tf::Vector3 v2p, tf::Vector3 v3p) :
            normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}
        Vec3f normal;
        Vec3f v1;
        Vec3f v2;
        Vec3f v3;
};
*/

typedef struct tag_Triangle
{
    Vec3f normal;
    Vec3f v1;
    Vec3f v2;
    Vec3f v3;
} Triangle;

inline float deg2rad(const float &deg) {
     return deg * M_PI / 180;
}

float parse_float(std::ifstream& s) {
   char f_buf[sizeof(float)];
   s.read(f_buf, 4);
   float* fptr = (float*) f_buf;
   return *fptr;
 }

 float parse_float(uint8_t* data, int offset){
   char f_buf[sizeof(float)];
   std::copy(data + offset, data + offset + 4, f_buf);
   float* fptr = (float*) f_buf;
   return *fptr;
 }

 Vec3f parse_point(std::ifstream& s) {
    float x = parse_float(s);
    float y = parse_float(s);
    float z = parse_float(s);
    return Vec3f{x, y, z};
 }

 Vec3f parse_point(uint8_t* data, int* offset){
    float x = parse_float(data, *offset);
    float y = parse_float(data, *offset+4);
    float z = parse_float(data, *offset+8);
    *offset += 12;
    return Vec3f{x, y, z};
 }


struct Options
{
    uint32_t horizontalBeams = 3600;
    uint32_t verticalBeams = 16;
    float horizontalResolution = 0.1;
    float verticalResolution = 2.0;
};

class Object
{
 public:
    Object() {}
    virtual ~Object() {}
    virtual bool intersect(const Vec3f &, const Vec3f &, float &, uint32_t &, Vec2f &) const = 0;
    virtual void getSurfaceProperties(const Vec3f &, const Vec3f &, const uint32_t &, const Vec2f &, Vec3f &, Vec2f &) const = 0;
};

bool rayTriangleIntersect(
    const Vec3f &orig, const Vec3f &dir,
    const Vec3f &v0, const Vec3f &v1, const Vec3f &v2,
    float &t, float &u, float &v)
{
    Vec3f v0v1 = vecSubVec(v1, v0);
    Vec3f v0v2 = vecSubVec(v2, v0);
    Vec3f pvec = cross(dir, v0v2);
    float det = dot(v0v1, pvec);

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;

    float invDet = 1 / det;

    Vec3f tvec = vecSubVec(orig, v0);
    u = dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vec3f qvec = cross(tvec, v0v1);
    v = dot(dir, qvec) * invDet;
    if (v < 0 || u + v > 1) return false;
    
    t = dot(v0v2, qvec) * invDet;
    if(t < 0) return false;

    return true;
}

class TriangleMesh2 : public Object{
    public:
        TriangleMesh2(std::vector<Triangle> tris):triangles(tris), offset(Vec3f{0, 0, 0}){}
        bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear, uint32_t &triIndex, Vec2f &uv) const {
            bool isect = false;
            for (uint32_t i = 0; i < triangles.size(); ++i) {
                const Vec3f &v0 = vecAddVec(triangles[i].v1, offset);
                const Vec3f &v1 = vecAddVec(triangles[i].v2, offset);
                const Vec3f &v2 = vecAddVec(triangles[i].v3, offset);

                float t = kInfinity, u, v;
                if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear) {
                    tNear = t;
                    uv.x = u;
                    uv.y = v;
                    triIndex = i;
                    isect = true;
                }
            }
            return isect;
        }

        
    void getSurfaceProperties( const Vec3f &hitPoint, const Vec3f &viewDirection, const uint32_t &triIndex, const Vec2f &uv, Vec3f &hitNormal, Vec2f &hitTextureCoordinates) const
    {
        // face normal
        // const Vec3f &v0 = P[trisIndex[triIndex * 3]];
        // const Vec3f &v1 = P[trisIndex[triIndex * 3 + 1]];
        // const Vec3f &v2 = P[trisIndex[triIndex * 3 + 2]];
        // hitNormal = (v1 - v0).cross(v2 - v0);
        // hitNormal.normalize();
    }

    /**
     * Offset each triangles vertices by the specified vector
     */
    void setOffset(Vec3f off){
        this->offset = off;
        for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto v0 = vecAddVec(triangles[i].v1, offset);
            auto v1 = vecAddVec(triangles[i].v2, offset);
            auto v2 = vecAddVec(triangles[i].v3, offset);
            auto normal = triangles[i].normal;
            triangles[i] = Triangle{normal, v0, v1, v2};
        }
    }

    void setScale(Vec3f scale){
        this->scale = scale;

        for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto v0 = vecMulVecElem(triangles[i].v1, scale);
            auto v1 = vecMulVecElem(triangles[i].v2, scale);
            auto v2 = vecMulVecElem(triangles[i].v3, scale);
            auto normal = triangles[i].normal;
            triangles[i] = Triangle{normal, v0, v1, v2};
        }
    }

        //Members
        std::vector<Triangle> triangles;
    private:
        Vec3f offset;
        Vec3f scale;
};


TriangleMesh2* parse_stl(uint8_t* data, uint32_t size) {
    char n_triangles[4];
    
    // skip over header information and read the number of triangles
    std::copy(data + 80, data + 84, n_triangles);
    int offset = 84;

    unsigned int* r = (unsigned int*) n_triangles;
    unsigned int num_triangles = *r;
    std::vector<Triangle> tris(num_triangles);

    for (unsigned int i = 0; i < num_triangles; i++) {
      auto normal = parse_point(data, &offset);
      auto v1 = parse_point(data, &offset);
      auto v2 = parse_point(data, &offset);
      auto v3 = parse_point(data, &offset);
      tris[i] = Triangle{normal, v1, v2, v3};

      // skip over the 2 'attribute bytes'
      offset += 2;
    }

    return new TriangleMesh2(tris);
}

TriangleMesh2* parse_stl(const std::string& stl_path) {
    std::ifstream stl_file(stl_path.c_str(), std::ios::in | std::ios::binary);

    if (!stl_file) {
      std::cout << "ERROR: COULD NOT READ STL FILE" << std::endl;
      exit(1);
    }

    char header_info[80] = "";
    char n_triangles[4];
    stl_file.read(header_info, 80);
    stl_file.read(n_triangles, 4);
    std::string h(header_info);

    unsigned int* r = (unsigned int*) n_triangles;
    unsigned int num_triangles = *r;
    std::vector<Triangle> tris(num_triangles);

    for (unsigned int i = 0; i < num_triangles; i++) {
      auto normal = parse_point(stl_file);
      auto v1 = parse_point(stl_file);
      auto v2 = parse_point(stl_file);
      auto v3 = parse_point(stl_file);
      tris[i] = Triangle{normal, v1, v2, v3};

      char dummy[2];
      stl_file.read(dummy, 2);
    }

    return new TriangleMesh2(tris);
}

bool trace(
    const Vec3f &orig, const Vec3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects,
    float &tNear, uint32_t &index, Vec2f &uv, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearTriangle = kInfinity;
        uint32_t indexTriangle;
        Vec2f uvTriangle;
        if (objects[k]->intersect(orig, dir, tNearTriangle, indexTriangle, uvTriangle) && tNearTriangle < tNear) {
            *hitObject = objects[k].get();
            tNear = tNearTriangle;
            index = indexTriangle;
            uv = uvTriangle;
        }
    }

    return (*hitObject != nullptr);
}

std::vector<Vec3f> render(const Options &options, const std::vector<std::unique_ptr<Object>> &objects) {
    std::vector<Vec3f> pointList(options.horizontalBeams * options.verticalBeams);
    int hitCnt = 0;
    Vec3f orig{0, 0, 0};

    auto timeStart = std::chrono::high_resolution_clock::now();

    for (uint32_t j = 0; j < options.horizontalBeams; ++j) {
        #pragma omp parallel for
        for (uint32_t i = 0; i < options.verticalBeams; ++i) {
            // generate primary ray direction
            float vert = i - options.verticalBeams/2.0;
            float x = cos(deg2rad((float)j * options.horizontalResolution)) * cos(deg2rad(vert * options.verticalResolution));
            float z = sin(deg2rad((float)j * options.horizontalResolution)) * cos(deg2rad(vert * options.verticalResolution));
            float y = sin(deg2rad(vert * options.verticalResolution));

            Vec3f dir = normalize(Vec3f{x, y, -z});

            float tnear = kInfinity;
            Vec2f uv;
            uint32_t index = 0;
            Object *hitObject = nullptr;
            if (trace(orig, dir, objects, tnear, index, uv, &hitObject)) {
                Vec3f hitPoint = vecMulScalar(vecAddVec(orig, dir), tnear);

                pointList[hitCnt] = Vec3f{hitPoint.x, hitPoint.y, hitPoint.z};
                hitCnt++;
            }
        }
        // fprintf(stderr, "\r%3d%c", uint32_t(j / (float)options.height * 100), '%');
    }

    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
    fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);

    pointList.resize(hitCnt);
    return pointList;
}

bool simulate(lidarSimulator::LiDARSimulationRequest  &req, lidarSimulator::LiDARSimulationResponse &res) {
    Options options;

    options.horizontalBeams = req.scanner.horizontalBeams;
    options.verticalBeams = req.scanner.verticalBeams;
    options.horizontalResolution = req.scanner.horizontalResolution;
    options.verticalResolution = req.scanner.verticalResolution;

    // TriangleMesh2* object = parse_stl(std::string("cow.stl"));
    // TriangleMesh2* object = parse_stl(std::string("twizy.stl"));
    // TriangleMesh2* object2 = parse_stl(std::string("qube.stl"));
    // TriangleMesh2* object = parse_stl(std::string("twizy_low_poly.stl"));

    std::vector<std::unique_ptr<Object>> objects;

    resource_retriever::MemoryResource resource;
    for(auto object : req.objects){
        try {
            resource = r.get(object.object); 
        } catch (resource_retriever::Exception& e) {
            ROS_ERROR("Failed to retrieve file: %s", e.what());
            return false;
        }

        uint8_t* r_data = resource.data.get();
        TriangleMesh2* triangleObject = parse_stl(resource.data.get(), resource.size);

        triangleObject->setOffset(Vec3f{object.pose.position.x, object.pose.position.y, object.pose.position.z});
        triangleObject->setScale(Vec3f{object.scale.x, object.scale.y, object.scale.z});

        objects.push_back(std::unique_ptr<Object>(triangleObject));
    }


    auto pointList = render(options, objects);
    ROS_INFO("NmbrPoints: %d", pointList.size());

    visualization_msgs::Marker cloud;
    cloud.header.frame_id = "origin";
    cloud.type = visualization_msgs::Marker::POINTS;
    cloud.scale.x = 0.01;
    cloud.scale.y = 0.01;
    cloud.scale.z = 0.01;
    cloud.color.a = 1;
    cloud.color.r = 1;
    cloud.color.g = 0;
    cloud.color.b = 0;

    std::vector<geometry_msgs::Point> cloud_points;
    for(uint32_t i = 0; i < pointList.size(); i++){
        geometry_msgs::Point tmp;
        tmp.x = pointList[i].x;
        tmp.y = -pointList[i].z;
        tmp.z = pointList[i].y;

        cloud_points.push_back(tmp);
    }

    cloud.points = cloud_points;

    cloudPub.publish(cloud);

    res.cloud = cloud;
    
    return true;
}

void saveToFile(std::vector<Vec3f> pointList){
    // save pointcloud to file
    std::ofstream ofs;
    ofs.open("plc_out.ply");
    ofs << "ply\n"
     << "format ascii 1.0\n"
     << "element vertex " << pointList.size() <<"\n"
     << "property float32 x\n"
     << "property float32 y\n"
     << "property float32 z\n"
     << "end_header\n";

    for (uint32_t i = 0; i < pointList.size(); ++i) {
        ofs << pointList[i].x << " " << pointList[i].y  << " " << pointList[i].z << std::endl;
    }

    ofs.close();
}

int main(int argc, char **argv) {
    // ros::init(argc, argv, "lidarSimulator");
    // ros::NodeHandle nh;
    // cloudPub = nh.advertise<visualization_msgs::Marker>("/simMarker", 1);

    // ros::ServiceServer service = nh.advertiseService("lidarSimulatorService", simulate);

    //get all platforms (drivers)
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if(all_platforms.size() == 0){
        std::cout << " No platforms found. Check OpenCL installation!" << std::endl;
        exit(1);
    }

    cl::Platform default_platform = all_platforms[0];
    std::cout << "Using platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << std::endl;

    //get default device of the default platform
    std::vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    if(all_devices.size() == 0){
        std::cout<<" No devices found. Check OpenCL installation!" << std::endl;
        exit(1);
    }

    cl::Device default_device = all_devices[0];
    std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>() << std::endl;
    std::cout << "Maximum items per workgroup: " << default_device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() << std::endl;

    cl::Context context({default_device});
    cl::Program::Sources sources;

    std::ifstream sourceFile("src/kernel.cl");
    std::string kernel_code( std::istreambuf_iterator<char>(sourceFile), (std::istreambuf_iterator<char>()));

    sources.push_back({kernel_code.c_str(), kernel_code.length()});

    cl::Program program(context, sources);
    if(program.build({default_device}, "-cl-denorms-are-zero -cl-single-precision-constant -cl-no-signed-zeros -cl-unsafe-math-optimizations -cl-mad-enable -cl-finite-math-only -cl-fast-relaxed-math") != CL_SUCCESS){
        std::cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << std::endl;
        exit(1);
    }

    TriangleMesh2* object = parse_stl(std::string("twizy_low_poly.stl"));
    object->setScale(Vec3f{0.01, 0.01, 0.01});
    object->setOffset(Vec3f{0, 0, 3});

    // for(int i=0; i<object->triangles.size(); i++){
    //     auto v = object->triangles[i].v1;
    //     std::cout << "X: " << v.x << "Y: " << v.y << "Z: " << v.z << std::endl;
    //     v = object->triangles[i].v2;
    //     std::cout << "X: " << v.x << "Y: " << v.y << "Z: " << v.z << std::endl;
    //     v = object->triangles[i].v3;
    //     std::cout << "X: " << v.x << "Y: " << v.y << "Z: " << v.z << std::endl;
    //     std::cout << std::endl;
    // }

    int numberTris = object->triangles.size();
    printf("Numbertris: %d\n", numberTris);
    ray testray = {
                    Vec3f{0, 0, 0},
                    Vec3f{0, 0, 1}
                  };

    int horizontalBeams = 3600;
    int verticalBeams = 15;
    float horizontalResolution = 0.1;
    float verticalResolution = 2.0;

    int numberRays = horizontalBeams * verticalBeams;
    ray *testrays = new ray[numberRays];

    for (uint32_t j = 0; j < horizontalBeams; ++j) {
        for (uint32_t i = 0; i < verticalBeams; ++i) {
            // generate primary ray direction
            float vert = i - verticalBeams/2.0;
            float x = cos(deg2rad((float)j * horizontalResolution)) * cos(deg2rad(vert * verticalResolution));
            float z = sin(deg2rad((float)j * horizontalResolution)) * cos(deg2rad(vert * verticalResolution));
            float y = sin(deg2rad(vert * verticalResolution));

            Vec3f dir = normalize(Vec3f{x, y, -z});
            ray r = {
                Vec3f{0, 0, 0},
                dir
            };

            int id = j*verticalBeams + i;
            testrays[id] = r;
        }
    }

    // create buffers on the device
    cl::Buffer buffer_tris(context, CL_MEM_READ_ONLY, sizeof(Triangle)*numberTris);
    cl::Buffer buffer_ray(context, CL_MEM_READ_ONLY, sizeof(ray)*numberRays);
    cl::Buffer buffer_distances(context, CL_MEM_READ_WRITE, sizeof(float)*numberTris*numberRays);

    cl::Buffer buffer_number_tris(context, CL_MEM_READ_ONLY, sizeof(int));
    cl::Buffer buffer_shortest_dist(context, CL_MEM_READ_WRITE, sizeof(float)*numberRays);

    //build the intersect kernel
    cl::Kernel kernel_intersect = cl::Kernel(program, "rayTriangleIntersect");
    kernel_intersect.setArg(0, buffer_tris);
    kernel_intersect.setArg(1, buffer_ray);
    kernel_intersect.setArg(2, buffer_distances);

    //build the filter kernel
    cl::Kernel kernel_filter = cl::Kernel(program, "filterShortestDistance");
    kernel_filter.setArg(0, buffer_distances);
    kernel_filter.setArg(1, buffer_number_tris);
    kernel_filter.setArg(2, buffer_shortest_dist);

    //create queue to which we will push commands for the device.
    cl::CommandQueue queue(context, default_device);

    auto timeStart = std::chrono::high_resolution_clock::now();

    //write arrays tris and ray to the device
    queue.enqueueWriteBuffer(buffer_tris,  CL_TRUE, 0, sizeof(Triangle)*numberTris, &(object->triangles[0]));
    queue.enqueueWriteBuffer(buffer_ray,  CL_TRUE, 0, sizeof(ray)*numberRays, &testrays[0]);

    queue.enqueueWriteBuffer(buffer_number_tris,  CL_TRUE, 0, sizeof(int), &numberTris);

    //run both kernels
    queue.enqueueNDRangeKernel(kernel_intersect, cl::NullRange, cl::NDRange(numberTris, numberRays), cl::NullRange);
    queue.finish();

    queue.enqueueNDRangeKernel(kernel_filter, cl::NullRange, cl::NDRange(numberRays), cl::NullRange);
    queue.finish();

    //get result from device to host
    float *distances = new float[numberTris*numberRays];
    queue.enqueueReadBuffer(buffer_shortest_dist, CL_TRUE, 0, sizeof(float)*numberRays, distances);

    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<double, std::micro>(timeEnd - timeStart).count();
    fprintf(stderr, "\rDone: %.5f (sec)\n", passedTime / 1000000);

    //output the results to a pointcloud
    std::vector<Vec3f> pointList;
    for(int j=0; j<numberRays; j++){
        float dist = distances[j];
        if(!std::isnan(dist) && !std::isinf(dist)){
            ray the_ray = testrays[j];
            Vec3f hitPoint = vecMulScalar(vecAddVec(the_ray.origin, the_ray.dir), dist);
            pointList.push_back(hitPoint);
        }
    }

    saveToFile(pointList);

    // ros::spin();

    return 0;
}

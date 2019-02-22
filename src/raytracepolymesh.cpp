/**
 * A Node which provides a service to generate a point cloud
 * based on the stl objects provided
 */

#include <cstdio>
#include <vector>
#include <fstream>
#include <cmath>
#include <chrono>

#include "ros/ros.h"
#include <resource_retriever/retriever.h>
#include <tf/transform_datatypes.h>
#include "lidarSimulator/LiDARSimulation.h"

#include "visualization_msgs/Marker.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

#include "geometry.h"

typedef tf::Vector3 Vec3f;

static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;

ros::Publisher cloudPub;

resource_retriever::Retriever r;

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
//    s.read(f_buf, 4);
   std::copy(data + offset, data + offset + 4, f_buf);
   float* fptr = (float*) f_buf;
   return *fptr;
 }

 Vec3f parse_point(std::ifstream& s) {
    float x = parse_float(s);
    float y = parse_float(s);
    float z = parse_float(s);
    return Vec3f(x, y, z);
 }

 Vec3f parse_point(uint8_t* data, int* offset){
    float x = parse_float(data, *offset);
    float y = parse_float(data, *offset+4);
    float z = parse_float(data, *offset+8);
    *offset += 12;
    return Vec3f(x, y, z);
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
    Vec3f v0v1 = v1 - v0;
    Vec3f v0v2 = v2 - v0;
    Vec3f pvec = dir.cross(v0v2);
    float det = v0v1.dot(pvec);

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;

    float invDet = 1 / det;

    Vec3f tvec = orig - v0;
    u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vec3f qvec = tvec.cross(v0v1);
    v = dir.dot(qvec) * invDet;
    if (v < 0 || u + v > 1) return false;
    
    t = v0v2.dot(qvec) * invDet;
    if(t < 0) return false;

    return true;
}

class TriangleMesh2 : public Object{
    public:
        TriangleMesh2(std::vector<Triangle> tris):triangles(tris), offset(Vec3f(0, 0, 0)){}
        bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear, uint32_t &triIndex, Vec2f &uv) const {
            bool isect = false;
            for (uint32_t i = 0; i < triangles.size(); ++i) {
                const Vec3f &v0 = triangles[i].v1 + offset;
                const Vec3f &v1 = triangles[i].v2 + offset;
                const Vec3f &v2 = triangles[i].v3 + offset;

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
            auto v0 = triangles[i].v1 + offset;
            auto v1 = triangles[i].v2 + offset;
            auto v2 = triangles[i].v3 + offset;
            auto normal = triangles[i].normal;
            triangles[i] = Triangle(normal, v0, v1, v2);
        }
    }

    void setScale(Vec3f scale){
        this->scale = scale;

        for (uint32_t i = 0; i < triangles.size(); ++i) {
            auto v0 = triangles[i].v1 * scale;
            auto v1 = triangles[i].v2 * scale;
            auto v2 = triangles[i].v3 * scale;
            auto normal = triangles[i].normal;
            triangles[i] = Triangle(normal, v0, v1, v2);
        }
    }

        //Members
        std::vector<Triangle> triangles;
    private:
        Vec3f offset;
        Vec3f scale;
};

/*
class TriangleMesh : public Object
{
public:
    // Build a triangle mesh from a face index array and a vertex index array
    TriangleMesh(
        const uint32_t nfaces,
        const std::unique_ptr<uint32_t []> &faceIndex,
        const std::unique_ptr<uint32_t []> &vertsIndex,
        const std::unique_ptr<Vec3f []> &verts,
        std::unique_ptr<Vec3f []> &normals,
        std::unique_ptr<Vec2f []> &st) :
        numTris(0)
    {
        uint32_t k = 0, maxVertIndex = 0;
        // find out how many triangles we need to create for this mesh
        for (uint32_t i = 0; i < nfaces; ++i) {
            numTris += faceIndex[i] - 2;
            for (uint32_t j = 0; j < faceIndex[i]; ++j)
                if (vertsIndex[k + j] > maxVertIndex)
                    maxVertIndex = vertsIndex[k + j];
            k += faceIndex[i];
        }
        maxVertIndex += 1;
        
        // allocate memory to store the position of the mesh vertices
        P = std::unique_ptr<Vec3f []>(new Vec3f[maxVertIndex]);
        for (uint32_t i = 0; i < maxVertIndex; ++i) {
            P[i] = verts[i];
        }
        
        // allocate memory to store triangle indices
        trisIndex = std::unique_ptr<uint32_t []>(new uint32_t [numTris * 3]);
        uint32_t l = 0;
        // [comment]
        // Generate the triangle index array
        // Keep in mind that there is generally 1 vertex attribute for each vertex of each face.
        // So for example if you have 2 quads, you only have 6 vertices but you have 2 * 4
        // vertex attributes (that is 8 normals, 8 texture coordinates, etc.). So the easiest
        // lazziest method in our triangle mesh, is to create a new array for each supported
        // vertex attribute (st, normals, etc.) whose size is equal to the number of triangles
        // multiplied by 3, and then set the value of the vertex attribute at each vertex
        // of each triangle using the input array (normals[], st[], etc.)
        // [/comment]
        N = std::unique_ptr<Vec3f []>(new Vec3f[numTris * 3]);
        texCoordinates = std::unique_ptr<Vec2f []>(new Vec2f[numTris * 3]);
        for (uint32_t i = 0, k = 0; i < nfaces; ++i) { // for each  face
            for (uint32_t j = 0; j < faceIndex[i] - 2; ++j) { // for each triangle in the face
                trisIndex[l] = vertsIndex[k];
                trisIndex[l + 1] = vertsIndex[k + j + 1];
                trisIndex[l + 2] = vertsIndex[k + j + 2];
                N[l] = normals[k];
                N[l + 1] = normals[k + j + 1];
                N[l + 2] = normals[k + j + 2];
                texCoordinates[l] = st[k];
                texCoordinates[l + 1] = st[k + j + 1];
                texCoordinates[l + 2] = st[k + j + 2];
                l += 3;
            }
            k += faceIndex[i];
        }
    }

    // Test if the ray interesests this triangle mesh
    bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear, uint32_t &triIndex, Vec2f &uv) const
    {
        uint32_t j = 0;
        bool isect = false;
        for (uint32_t i = 0; i < numTris; ++i) {
            j = 3*i;
            const Vec3f &v0 = P[trisIndex[j]];
            const Vec3f &v1 = P[trisIndex[j + 1]];
            const Vec3f &v2 = P[trisIndex[j + 2]];
            float t = kInfinity, u, v;
            if (rayTriangleIntersect(orig, dir, v0, v1, v2, t, u, v) && t < tNear) {
              tNear = t;
              uv.x = u;
              uv.y = v;
              triIndex = i;
              isect = true;
            }
            //j += 3;
        }

        return isect;
    }
    void getSurfaceProperties(
        const Vec3f &hitPoint,
        const Vec3f &viewDirection,
        const uint32_t &triIndex,
        const Vec2f &uv,
        Vec3f &hitNormal,
        Vec2f &hitTextureCoordinates) const
    {
        // face normal
        const Vec3f &v0 = P[trisIndex[triIndex * 3]];
        const Vec3f &v1 = P[trisIndex[triIndex * 3 + 1]];
        const Vec3f &v2 = P[trisIndex[triIndex * 3 + 2]];
        hitNormal = (v1 - v0).cross(v2 - v0);
        hitNormal.normalize();
        
        // texture coordinates
        const Vec2f &st0 = texCoordinates[triIndex * 3];
        const Vec2f &st1 = texCoordinates[triIndex * 3 + 1];
        const Vec2f &st2 = texCoordinates[triIndex * 3 + 2];
        hitTextureCoordinates = (1 - uv.x - uv.y) * st0 + uv.x * st1 + uv.y * st2;

    }

    // member variables
    uint32_t numTris;                         // number of triangles
    std::unique_ptr<Vec3f []> P;              // triangles vertex position
    std::unique_ptr<uint32_t []> trisIndex;   // vertex index array
    std::unique_ptr<Vec3f []> N;              // triangles vertex normals
    std::unique_ptr<Vec2f []> texCoordinates; // triangles texture coordinates
};
*/

/*
TriangleMesh* loadPolyMeshFromFile(const char *file)
{
    std::ifstream ifs;
    try {
        ifs.open(file);
        if (ifs.fail()) throw;
        std::stringstream ss;
        ss << ifs.rdbuf();
        uint32_t numFaces;
        ss >> numFaces;
        std::unique_ptr<uint32_t []> faceIndex(new uint32_t[numFaces]);
        uint32_t vertsIndexArraySize = 0;
        // reading face index array
        for (uint32_t i = 0; i < numFaces; ++i) {
            ss >> faceIndex[i];
            vertsIndexArraySize += faceIndex[i];
        }
        std::unique_ptr<uint32_t []> vertsIndex(new uint32_t[vertsIndexArraySize]);
        uint32_t vertsArraySize = 0;
        // reading vertex index array
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> vertsIndex[i];
            if (vertsIndex[i] > vertsArraySize) vertsArraySize = vertsIndex[i];
        }
        vertsArraySize += 1;
        // reading vertices
        std::unique_ptr<Vec3f []> verts(new Vec3f[vertsArraySize]);
        for (uint32_t i = 0; i < vertsArraySize; ++i) {
            ss >> verts[i].x() >> verts[i].y() >> verts[i].z();
            // printf("Vert X: %.2f Y: %.2f Z: %.2f\n", verts[i].x, verts[i].y, verts[i].z);
        }
        // reading normals
        std::unique_ptr<Vec3f []> normals(new Vec3f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> normals[i].x() >> normals[i].y() >> normals[i].z();
        }
        // reading st coordinates
        std::unique_ptr<Vec2f []> st(new Vec2f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> st[i].x >> st[i].y;
        }
        
        return new TriangleMesh(numFaces, faceIndex, vertsIndex, verts, normals, st);
    }
    catch (...) {
        ifs.close();
    }
    ifs.close();
    
    return nullptr;
}
*/

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
      tris[i] = Triangle(normal, v1, v2, v3);

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
      tris[i] = Triangle(normal, v1, v2, v3);

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
    Vec3f orig(0, 0, 0);

    auto timeStart = std::chrono::high_resolution_clock::now();

    for (uint32_t j = 0; j < options.horizontalBeams; ++j) {
        #pragma omp parallel for
        for (uint32_t i = 0; i < options.verticalBeams; ++i) {
            // generate primary ray direction
            float vert = i - options.verticalBeams/2.0;
            float x = cos(deg2rad((float)j * options.horizontalResolution)) * cos(deg2rad(vert * options.verticalResolution));
            float z = sin(deg2rad((float)j * options.horizontalResolution)) * cos(deg2rad(vert * options.verticalResolution));
            float y = sin(deg2rad(vert * options.verticalResolution));

            Vec3f dir = Vec3f(x, y, -z).normalize();

            float tnear = kInfinity;
            Vec2f uv;
            uint32_t index = 0;
            Object *hitObject = nullptr;
            if (trace(orig, dir, objects, tnear, index, uv, &hitObject)) {
                Vec3f hitPoint = orig + dir * tnear;

                pointList[hitCnt] = Vec3f(hitPoint.x(), hitPoint.y(), hitPoint.z());
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

        Vec3f offset;
        Vec3f scale;
        tf::pointMsgToTF(object.pose.position, offset);
        tf::vector3MsgToTF(object.scale, scale);
        triangleObject->setOffset(offset);
        triangleObject->setScale(scale);

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
        tmp.x = pointList[i].x();
        tmp.y = -pointList[i].z();
        tmp.z = pointList[i].y();

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
        ofs << pointList[i].x() << " " << pointList[i].y()  << " " << pointList[i].z() << std::endl;
    }

    ofs.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidarSimulator");
    ros::NodeHandle nh;
    cloudPub = nh.advertise<visualization_msgs::Marker>("/simMarker", 1);


    ros::ServiceServer service = nh.advertiseService("lidarSimulatorService", simulate);

    ros::spin();

    return 0;
}

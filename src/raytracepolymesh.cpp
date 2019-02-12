//[header]
// A simple program to demonstrate how to ray-trace a polygon mesh
//[/header]
//[compile]
// Download the raytracepolygonmesh.cpp, geometry.h and cow.geo file to a folder.
// Open a shell/terminal, and run the following command where the files is saved:
//
// c++ -o raytracepolymesh raytracepolymesh.cpp -std=c++11 -O3
//
// Run with: ./raytracepolygonmesh. Open the file ./out.0000.png in Photoshop or any program
// reading PPM files.
//[/compile]

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>

#include "geometry.h"

static const float kInfinity = std::numeric_limits<float>::max();
static const float kEpsilon = 1e-8;
static const Vec3f kDefaultBackgroundColor = Vec3f(0.235294, 0.67451, 0.843137);

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline float deg2rad(const float &deg)
{ return deg * M_PI / 180; }

float parse_float(std::ifstream& s) {
   char f_buf[sizeof(float)];
   s.read(f_buf, 4);
   float* fptr = (float*) f_buf;
   return *fptr;
 }

 Vec3f parse_point(std::ifstream& s) {
    float offset_x = 0;
    float offset_y = 0;
    float offset_z = -3;
    float x = parse_float(s)/100.f + offset_x;
    float y = parse_float(s)/100.f + offset_y;
    float z = parse_float(s)/100.f + offset_z;
    return Vec3f(x, y, z);
 }


struct Options
{
    uint32_t width = 640;
    uint32_t height = 480;
    // uint32_t width = 1920;
    // uint32_t height = 1080;
    float fov = 90;
    Vec3f backgroundColor = kDefaultBackgroundColor;
    Matrix44f cameraToWorld;
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
    Vec3f pvec = dir.crossProduct(v0v2);
    float det = v0v1.dotProduct(pvec);

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;

    float invDet = 1 / det;

    Vec3f tvec = orig - v0;
    u = tvec.dotProduct(pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vec3f qvec = tvec.crossProduct(v0v1);
    v = dir.dotProduct(qvec) * invDet;
    if (v < 0 || u + v > 1) return false;
    
    t = v0v2.dotProduct(qvec) * invDet;
    
    return true;
}

class TriangleMesh2 : public Object{
    public:
        TriangleMesh2(std::vector<Triangle<float>> tris):triangles(tris){}
        bool intersect(const Vec3f &orig, const Vec3f &dir, float &tNear, uint32_t &triIndex, Vec2f &uv) const {
            bool isect = false;
            for (uint32_t i = 0; i < triangles.size(); ++i) {
                const Vec3f &v0 = triangles[i].v1;
                const Vec3f &v1 = triangles[i].v2;
                const Vec3f &v2 = triangles[i].v3;

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
        // hitNormal = (v1 - v0).crossProduct(v2 - v0);
        // hitNormal.normalize();
    }

        //Members
        std::vector<Triangle<float>> triangles;
};

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
        hitNormal = (v1 - v0).crossProduct(v2 - v0);
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
            ss >> verts[i].x >> verts[i].y >> verts[i].z;
            // printf("Vert X: %.2f Y: %.2f Z: %.2f\n", verts[i].x, verts[i].y, verts[i].z);
        }
        // reading normals
        std::unique_ptr<Vec3f []> normals(new Vec3f[vertsIndexArraySize]);
        for (uint32_t i = 0; i < vertsIndexArraySize; ++i) {
            ss >> normals[i].x >> normals[i].y >> normals[i].z;
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

TriangleMesh2* parse_stl(const std::string& stl_path) {
    std::ifstream stl_file(stl_path.c_str(), std::ios::in | std::ios::binary);

    if (!stl_file) {
      std::cout << "ERROR: COULD NOT READ FILE" << std::endl;
      exit(1);
    }

    char header_info[80] = "";
    char n_triangles[4];
    stl_file.read(header_info, 80);
    stl_file.read(n_triangles, 4);
    std::string h(header_info);

    unsigned int* r = (unsigned int*) n_triangles;
    unsigned int num_triangles = *r;
    std::vector<Triangle<float>> tris(num_triangles);

    for (unsigned int i = 0; i < num_triangles; i++) {
      auto normal = parse_point(stl_file);
      auto v1 = parse_point(stl_file);
      auto v2 = parse_point(stl_file);
      auto v3 = parse_point(stl_file);
      tris[i] = Triangle<float>(normal, v1, v2, v3);

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

Vec3f castRay(
    const Vec3f &orig, const Vec3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects,
    const Options &options)
{
    Vec3f hitColor = options.backgroundColor;
    float tnear = kInfinity;
    Vec2f uv;
    uint32_t index = 0;
    Object *hitObject = nullptr;
    if (trace(orig, dir, objects, tnear, index, uv, &hitObject)) {
        Vec3f hitPoint = orig + dir * tnear;
        // printf("HIT: X: %.2f, Y: %.2f, Z: %.2f\n", hitPoint.x, hitPoint.y, hitPoint.z);

        //Vec3f hitNormal;
        //Vec2f hitTexCoordinates;
        //hitObject->getSurfaceProperties(hitPoint, dir, index, uv, hitNormal, hitTexCoordinates);
        //float NdotView = std::max(0.f, hitNormal.dotProduct(-dir));
        //const int M = 10;
        //float checker = (fmod(hitTexCoordinates.x * M, 1.0) > 0.5) ^ (fmod(hitTexCoordinates.y * M, 1.0) < 0.5);
        //float c = 0.3 * (1 - checker) + 0.7 * checker;
        
        //hitColor = c * NdotView; //Vec3f(uv.x, uv.y, 0);

        // hitColor = Vec3f(1.0, 0, 0);
        hitColor = Vec3f(uv.x, uv.y, 0);
    }

    return hitColor;
}

void render(
    const Options &options,
    const std::vector<std::unique_ptr<Object>> &objects,
    const uint32_t &frame)
{
    std::unique_ptr<Vec3f []> framebuffer(new Vec3f[options.width * options.height]);
    Vec3f *pix = framebuffer.get();
    float scale = tan(deg2rad(options.fov * 0.5));
    float imageAspectRatio = options.width / (float)options.height;
    Vec3f orig;
    // options.cameraToWorld.multVecMatrix(Vec3f(0), orig);
    auto timeStart = std::chrono::high_resolution_clock::now();
    for (uint32_t j = 0; j < options.height; ++j) {
#pragma omp parallel for
        for (uint32_t i = 0; i < options.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)options.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)options.height) * scale;
            // Vec3f dir;
            // options.cameraToWorld.multDirMatrix(Vec3f(x, y, -1), dir);
            Vec3f dir(x, y, -1);
            dir.normalize();
            pix[i+j*options.width] = castRay(orig, dir, objects, options);
        }
        fprintf(stderr, "\r%3d%c", uint32_t(j / (float)options.height * 100), '%');
    }
    auto timeEnd = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
    fprintf(stderr, "\rDone: %.2f (sec)\n", passedTime / 1000);
    
    // save framebuffer to file
    char buff[256];
    sprintf(buff, "out.%04d.ppm", frame);
    std::ofstream ofs;
    ofs.open(buff);
    ofs << "P6\n" << options.width << " " << options.height << "\n255\n";
    for (uint32_t i = 0; i < options.height * options.width; ++i) {
        char r = (char)(255 * clamp(0, 1, framebuffer[i].x));
        char g = (char)(255 * clamp(0, 1, framebuffer[i].y));
        char b = (char)(255 * clamp(0, 1, framebuffer[i].z));
        ofs << r << g << b;
    }
    ofs.close();
    
}

int main(int argc, char **argv) {
    Options options;
    // Matrix44f tmp = Matrix44f(0.707107, -0.331295,   0.624695, 0,
    //                                  0,  0.883452,   0.468521, 0,
    //                          -0.707107, -0.331295,   0.624695, 0,
    //                           -7.63871, 0.747777, -10.400412, 1);
    Matrix44f tmp = Matrix44f(1.0, 0.0,  0.0, 0,
                              0.0, 1.0,  0.0, 0,
                              0.0, 0.0, 1.0, 0,
                              -0.0, 0.0, 0.0, 1);
    options.cameraToWorld = tmp.inverse();
    options.fov = 60;

    std::vector<std::unique_ptr<Object>> objects;
    //TriangleMesh *mesh = loadPolyMeshFromFile("./cow.geo");
    // if (mesh != nullptr) objects.push_back(std::unique_ptr<Object>(mesh));

    // TriangleMesh2* cowStl = parse_stl(std::string("cow.stl"));
    TriangleMesh2* cowStl = parse_stl(std::string("twizy.stl"));
    // TriangleMesh2* cowStl = parse_stl(std::string("twizy_low_poly.stl"));
    printf("Numtris %d\n", cowStl->triangles.size());

    objects.push_back(std::unique_ptr<Object>(cowStl));

    // finally, render
    render(options, objects, 0);
    return 0;
}

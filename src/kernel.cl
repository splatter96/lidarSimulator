__constant float kEpsilon = 1e-8;

typedef struct tag_Vec
{
    float       x;
    float       y;
    float       z;
} Vec3f;

typedef struct tag_triangle
{
    Vec3f        normal;
    Vec3f        v1;
    Vec3f        v2;
    Vec3f        v3;
} Triangle;

typedef struct tag_ray
{
    Vec3f       origin;
    Vec3f       dir;
} ray_type;

inline float dot(Vec3f v1, Vec3f v2){
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline Vec3f cross(Vec3f v1, Vec3f v2){
     Vec3f v = {
                v1.y * v2.z - v1.z * v2.y,
                v1.z * v2.x - v1.x * v2.z,
                v1.x * v2.y - v1.y * v2.x
     };

     return v;
}

inline Vec3f vecSubVec(Vec3f v1, Vec3f v2){
    Vec3f v = {
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    };

    return v;
}

void kernel rayTriangleIntersect(global Triangle* tris, global ray_type* rays, global float* distances){
    int id = get_global_id(0);

    Vec3f v0 = tris[id].v1;
    Vec3f v1 = tris[id].v2;
    Vec3f v2 = tris[id].v3;

    ray_type ray = rays[get_global_id(1)];
    Vec3f orig = ray.origin;
    Vec3f dir = ray.dir;

    float t, u, v;

    Vec3f v0v1 = vecSubVec(v1, v0);
    Vec3f v0v2 = vecSubVec(v2, v0);
    Vec3f pvec = cross(dir, v0v2);
    float det = dot(v0v1, pvec);

    distances[id*get_global_size(1)+get_global_id(1)] = INFINITY;

    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon)
        return;

    float invDet = 1 / det;

    Vec3f tvec = vecSubVec(orig, v0);
    u = dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1)
        return;

    Vec3f qvec = cross(tvec, v0v1);
    v = dot(dir, qvec) * invDet;
    if (v < 0 || u + v > 1)
        return;

    t = dot(v0v2, qvec) * invDet;
    if(t < 0)
        return;

    distances[id*get_global_size(1)+get_global_id(1)] = t;
}

void kernel filterShortestDistance(global float* distances, global int* numberTris, global float* shortest){
    int id = get_global_id(0);
    int number_rays = get_global_size(0);

    float current_shortest = INFINITY;

    for(int i=0; i<*numberTris; i++){
        if(distances[i*number_rays+id] < current_shortest)
            current_shortest = distances[i*number_rays+id];
    }

    shortest[id] = current_shortest;
}
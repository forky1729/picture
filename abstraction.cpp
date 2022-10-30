#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>

// struct ======================================================================================

struct vec3
{
    float x, y, z;
};

inline vec3 operator+(vec3 const &v1, vec3 const &v2)
{
    return
    {
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z,
    };
}

inline vec3 operator-(vec3 const &v1, vec3 const &v2)
{
    return
    {
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z,
    };
}

inline vec3 operator-(vec3 const &v)
{
    return {-v.x, -v.y, -v.z};
}

inline vec3 operator*(vec3 const &v1, vec3 const &v2)
{
    return
    {
        v1.x * v2.x,
        v1.y * v2.y,
        v1.z * v2.z,
    };
}

inline vec3 operator*(vec3 const &v, float const f)
{
    return
    {
        v.x * f,
        v.y * f,
        v.z * f,
    };
}

inline vec3 operator/(vec3 const &v1, vec3 const &v2)
{
    return
    {
        v1.x / v2.x,
        v1.y / v2.y,
        v1.z / v2.z,
    };
}

inline vec3 operator/(vec3 const &v, float const f)
{
    return
    {
        v.x / f,
        v.y / f,
        v.z / f,
    };
}

inline float dot(vec3 const &v1, vec3 const &v2)
{
    return v1.x * v2.x
         + v1.y * v2.y
         + v1.z * v2.z;
}

inline vec3 cross(vec3 const &v1, vec3 const &v2)
{
    return
    {
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x,
    };
}

inline float length(vec3 const &v)
{
    return std::sqrt(dot(v, v));
}

inline vec3 normalize(vec3 const &v)
{
    return v / length(v);
}

struct MaybeTwoIntersections
{
    float tMin, tMax;
};

MaybeTwoIntersections const None = {.tMin = 0.f, .tMax = -1.f};

inline bool happened(MaybeTwoIntersections const i)
{
    return i.tMin < i.tMax;
}

struct Ray
{
    vec3 origin;
    vec3 direction;
    vec3 point(float const t) const {return origin + direction * t;}
};

// box ====================================================================================
// задаются центр коробки и вектор одной из полудиагоналей + прямая. возвращает 2 точки пересечения

struct AABB
{
    vec3 min, max;
};

inline MaybeTwoIntersections rayAABBIntersection(Ray const &ray, AABB const &aabb)
{
    vec3 const a = (aabb.min - ray.origin) / ray.direction;
    vec3 const b = (aabb.max - ray.origin) / ray.direction;
    auto const min = [](float const x, float const y) {return x < y ? x : y;};
    auto const max = [](float const x, float const y) {return x < y ? y : x;};
    vec3 const tMin =
    {
        min(a.x, b.x),
        min(a.y, b.y),
        min(a.z, b.z),
    };
    vec3 const tMax =
    {
        max(a.x, b.x),
        max(a.y, b.y),
        max(a.z, b.z),
    };
    return
    {
        max(max(tMin.x, tMin.y), tMin.z),
        min(min(tMax.x, tMax.y), tMax.z),
    };
}

// cylinder =============================================================================================
// задаются точка начала и конца высоты цилиндра, его радиус + прямая. возвращаются 2 точки пересечения

struct Cylinder
{
    vec3 r0, r1;
    float radius;
};

inline MaybeTwoIntersections rayCylinderIntersection(Ray const &ray, Cylinder const &cylinder)
{
    vec3 const e = ray.origin - cylinder.r0;
    vec3 const l = normalize(cylinder.r1 - cylinder.r0);

    float const dl = dot(ray.direction, l);
    float const el = dot(e, l);
    float const l2 = dot(l, l);

    float const a = 1.f - dl * dl;
    float const b = dot(ray.direction, e) - dl * el;
    float const c = dot(e, e) - cylinder.radius * cylinder.radius - el * el;
    float const det = b * b - a * c;
    if(det < 0.f)
        return None;

    float const t[] =
    {
        (-b - sqrt(det)) / a,
        (-b + sqrt(det)) / a,
        dot(cylinder.r0 - ray.origin, l) / dl,
        dot(cylinder.r1 - ray.origin, l) / dl,
    };
    auto const min = [](float const x, float const y) {return x < y ? x : y;};
    auto const max = [](float const x, float const y) {return x < y ? y : x;};
    return
    {
        max(min(t[0], t[1]), min(t[2], t[3])),
        min(max(t[0], t[1]), max(t[2], t[3])),
    };
}

// sphere ================================================================================
// задается начальная точка и радиус + прямая. возваращаются 2 точки пересечения

struct Sphere
{
    vec3 origin;
    float radius;
};

inline MaybeTwoIntersections raySphereIntersection(Ray const &ray, Sphere const &sphere)
{
    vec3 const l = ray.origin - sphere.origin;
    float const dl = dot(ray.direction, l);
    float const det = dl * dl - dot(l, l) + sphere.radius * sphere.radius;
    return
    {
        .tMin = -dl - std::sqrt(det),
        .tMax = -dl + std::sqrt(det),
    };
}

// triangle ========================================================================================
// задаются 3 вершины треугольника + прямая. возвращает координаты точки пересечения с треугольником

struct Triangle
{
    vec3 r0, r1, r2;
};

struct RayTriangleIntersection
{
    float t, p, q;                  // t - параметр луча в точке пересечения; p, q называются барицентрические координаты по-умному
};

inline bool happened(RayTriangleIntersection const &i)
{
    return i.p >= 0.f
        && i.q >= 0.f
        && i.p + i.q <= 1.f;
}

inline RayTriangleIntersection rayTriangleIntersection(Ray const &ray, Triangle const &triangle)
{
    vec3 const a = triangle.r1 - triangle.r0;
    vec3 const b = triangle.r2 - triangle.r0;
    vec3 const c = ray.origin - triangle.r0;
    vec3 const d = ray.direction;
    float const det0 = dot(-d, cross(a, b));
    float const det1 = dot( c, cross(a, b));
    float const det2 = dot(-d, cross(c, b));
    float const det3 = dot(-d, cross(a, c));
    return 
    {
        .t = det1 / det0,
        .p = det2 / det0,
        .q = det3 / det0,
    };
}

// camera ==============================================================================

struct Camera
{
    vec3 position;              //точка, в которой находится камера
    vec3 at;                    //точка, на которую направлена камера
    vec3 up;                    //направление "вверх", фиксирующее собственное вращение камеры вокруг оси
    float aspectRatio;          //соотношение сторон прямоугольника камеры, width / height
    float verticalFOV;          //тангенс угла от оси камеры до верхней стороны прямоугольника камеры
    // u, v ∈ [-1;1]
    Ray castRay(float const u, float const v) const
    {
        vec3 const z = normalize(position - at);
        vec3 const x = normalize(cross(up, z));
        vec3 const y = cross(z, x);
        vec3 const d = x * (u * verticalFOV * aspectRatio)
                     + y * (v * verticalFOV)
                     + z;
        return
        {
            .origin = position,
            .direction = normalize(d),
        };
    }
};

int main()
{
    unsigned int const width  = 1920u;
    unsigned int const height = 1080u;

    std::ofstream out("result.ppm");
    out << "P3" << std::endl;
    out << width << " " << height << std::endl;
    out << "255" << std::endl;

    Camera camera = {
        .position = {960, 540, 540},
        .at = {960, 0, 540},
        .up = {0, 0, 1},
        .aspectRatio = 16.f/9.f,
        .verticalFOV = 1
    };

    Sphere one = {
        .origin = {960, 0, 540},
        .radius = 200
    };

    Triangle two = {
        .r0 = {100, 0, 100},
        .r1 = {1600, 0, 100},
        .r2 = {750, 0, 800}
    };

    Sphere three = {
        .origin = {1200, 0, 550},
        .radius = 250
    };

    Cylinder four = {
        .r0 = {800, 0, 350},
        .r1 = {400, 0, 700},
        .radius = 110
    };

    Sphere five = {
        .origin = {960, 0, -2000},
        .radius = 2500
    };


    for (int j = -540; j<540; ++j)
    {
        for (int i = -960; i<960; ++i)
        {
            Ray ray = camera.castRay(i/960.f, j/540.f);
            MaybeTwoIntersections r_one = raySphereIntersection(ray, one);
            RayTriangleIntersection r_two = rayTriangleIntersection(ray, two);
            MaybeTwoIntersections r_three = raySphereIntersection(ray, three);
            MaybeTwoIntersections r_four = rayCylinderIntersection(ray, four);
            MaybeTwoIntersections r_five = raySphereIntersection(ray, five);

            if (happened(r_one)==true)
            {
                unsigned int const r = r_one.tMax*(2 + i/960.f + j/540.f) *10./4;
                unsigned int const g = r_one.tMax*(2 + i/960.f + j/540.f) *30./4;
                unsigned int const b = r_one.tMax*(2 + i/960.f + j/540.f) *15./4;
                out << r << " " << g << " " << b << " ";
            }
            else if (happened(r_four)==true)
            {
                unsigned int const r = (1+j/540.f)*90./3;
                unsigned int const g = (1+j/540.f)*175./3;
                unsigned int const b = (1+j/540.f)*100./3;
                out << r << " " << g << " " << b << " ";
            }
            else if (happened(r_two)==true)
            {
                unsigned int const r = r_two.p *255.;
                unsigned int const g = r_two.p *255.;
                unsigned int const b = r_two.p *255.;
                out << r << " " << g << " " << b << " ";    
            }
            else if (happened(r_three)==true)
            {
                unsigned int const r = (1+(i/std::abs(i))*std::sqrt(std::abs(i/960.f))) *255. / 3;
                unsigned int const g = (1+(i/std::abs(i))*std::sqrt(std::abs(i/960.f))) *100. / 3;
                unsigned int const b = (1+(i/std::abs(i))*std::sqrt(std::abs(i/960.f))) *225. / 3;
                out << r << " " << g << " " << b << " ";
            }
            else if (happened(r_five)==true)
            {
                unsigned int const r = r_five.tMin*255./1000;
                unsigned int const g = r_five.tMin*180./1000;
                unsigned int const b = r_five.tMin*100./1000;
                out << r << " " << g << " " << b << " ";
            }
            else
            {
                unsigned int const r = 0.;
                unsigned int const g = 0.;
                unsigned int const b = 0.;
                out << r << " " << g << " " << b << " ";
            }
        }
    }
}


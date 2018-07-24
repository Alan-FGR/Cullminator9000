#include <iostream>
#include "Bigged/bigged.cpp"
#include "entt.hpp"
#include <chrono>
#include <cmath>
#include <execution>
#include <tuple>

#define TIME_HERE std::chrono::high_resolution_clock::now();
#define ELAPSEDuS(time_point) (uint)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-time_point).count());

using namespace entt;

class FrustumTest : public TestBed {
    DefaultRegistry registry;
    virtual void Init() override;
    virtual void Update(float dt) override;
    virtual void Shutdown() override;
public:
    static bool drawListMtx;
};

struct Transform
{
    vec3 scalexyz;
    quat rotation;
    vec3 position;

    void Translate(vec3 local) {
        position += rotation * local;
    }

    void Rotate(quat dr) {
        rotation = dr * rotation;
    }

    mat4 GetMatrix() {
        auto m = mat4(rotation);
        m = scale(m, scalexyz);
        return translate(m, position);
    }

    void DrawDebug() {
        TestBed::DrawArrow(position, position + rotation * vec3(1, 0, 0), col32::red);
        TestBed::DrawArrow(position, position + rotation * vec3(0, 1, 0), col32::green);
        TestBed::DrawArrow(position, position + rotation * vec3(0, 0, 1), col32::blue);
        TestBed::DrawOBB(GetMatrix(), col32::purple, true);
        float sphereRad = length(scalexyz);
        TestBed::DrawSphere(position, sphereRad, col32(255,255,255,40), 2);
    }

};

struct BSphere
{
    __m128 simdCacheX;
    __m128 simdCacheY;
    __m128 simdCacheZ;
    __m128 simdCacheR;
    float radius;
    uint32_t pad;

    void UpdateCaches(Transform t)
    {
        simdCacheX = _mm_set_ps1(t.position.x);
        simdCacheY = _mm_set_ps1(t.position.y);
        simdCacheZ = _mm_set_ps1(t.position.z);
        simdCacheR = _mm_set_ps1(-radius);
    }

    std::tuple<vec3, float> GetCachedDataSlow()
    {
        return {
            vec3(
                simdCacheX.m128_f32[0],
                simdCacheY.m128_f32[0],
                simdCacheZ.m128_f32[0]
            ),
            radius
        };
    }

};

struct Velocity {
    vec3 linear;
    vec3 angular;

    void ApplyToTransform(Transform& t, float dt) {
        t.Rotate(quat(angular*dt));
        t.Translate(linear*dt);
    }
};

struct Frustum
{
    float fov;
    float nearPlane;
    float farPlane;
    float aspectRatio;

    mat4 GetFrustumMatrix(mat4 viewMatrix) {
        mat4 pm = glm::perspectiveFov(radians(fov), aspectRatio, 1.f, nearPlane, farPlane);
        mat4 invView = inverse(viewMatrix);
        vec3 nm = vec3(invView[3]);
        invView[3] = vec4(0, 0, 0, 1);
        viewMatrix = translate(invView, nm);
        mat4 vpm = pm * viewMatrix;
        return vpm;
    }

    mat4 GetFrustumMatrix(Transform holder) {
        return GetFrustumMatrix(holder.GetMatrix());
    }
};

void FrustumTest::Init()
{
    const int width = 100; //100
    const int height = 6; //6
    const float spacing = 2;

    //add stuff to cull
    for (int z = 0; z < width; z++)
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
            {
                auto entity = registry.create();
                
                 auto& tf = registry.assign<Transform>(entity,
                     vec3(((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f),
                     quat(1, 0, 0, 0),
                     vec3((x - (width/2)) * spacing, (y - (height/2)) * spacing, (z - (width/2)) * spacing)
                     );

                auto& sphere = registry.assign<BSphere>(entity);

                sphere.radius = ((20 + std::rand()) % 100) / 100.f;
                sphere.UpdateCaches(tf);
            }

    //add cullers
    auto entity = registry.create();
    registry.assign<Transform>(entity, vec3(1), quat(1, 0, 0, 0), vec3(0));
    registry.assign<Velocity>(entity, vec3(0, 0.f, 0), vec3(0, 1.1f, 0));
    registry.assign<Frustum>(entity, 50.f, 0.01f, 500.f, 2.f);

    SetCameraPosition(vec3(-5, 30, 5));
    SetCameraYawPitch(-35, 90);
}

__m128 _mm_add_ps(__m128 a, __m128 b, __m128 c)
{
    return _mm_add_ps(_mm_add_ps(a, b), c);
}

__m128 _mm_add_ps(__m128 a, __m128 b, __m128 c, __m128 d)
{
    return _mm_add_ps(_mm_add_ps(a, b), _mm_add_ps(c, d));
}

__m128 _mm_set_ps_bw(float x, float y, float z, float w)
{
    return _mm_set_ps(w, z, y, x);
}

bool draw = false;

std::vector<uint> times;
uint maxavg;
float lerpAvg;

std::vector<BSphere> inView;
bool FrustumTest::drawListMtx;

//#define MT
void simdCull(BSphere& s, __m128* planes)
{
    __m128 xs = _mm_mul_ps(planes[0], s.simdCacheX);
    __m128 ys = _mm_mul_ps(planes[1], s.simdCacheY);
    __m128 zs = _mm_mul_ps(planes[2], s.simdCacheZ);

    __m128 added = _mm_add_ps(xs, ys, zs, planes[3]);

    __m128 results = _mm_cmplt_ps(added, s.simdCacheR);

    auto cull = _mm_movemask_ps(results);

    if (draw) {
#ifdef MT
        FrustumTest::drawListMtx = true;
#endif
        if (!cull)
            inView.push_back(s);
#ifdef MT
        FrustumTest::drawListMtx = false;
#endif
    }
}

void FrustumTest::Update(float dt)
{
    DrawGrid();

    registry.view<Transform, Velocity>().each([this, &dt](auto entity, Transform& transform, Velocity& vel) {
        vel.ApplyToTransform(transform, dt);
        transform.DrawDebug();
    });

    registry.view<Transform, Frustum>().each([this](auto entity, Transform& transform, Frustum& fr) {
        
        mat4 frustumMat4 = fr.GetFrustumMatrix(transform);
        DrawFrustum(frustumMat4, col32::white);

        mat4& m = frustumMat4;

        vec4 right;
        right.x = m[0][3] + m[0][0];right.y = m[1][3] + m[1][0];right.z = m[2][3] + m[2][0];right.w = m[3][3] + m[3][0];

        vec4 left;
        left.x = m[0][3] - m[0][0];left.y = m[1][3] - m[1][0];left.z = m[2][3] - m[2][0];left.w = m[3][3] - m[3][0];

        vec4 top;
        top.x = m[0][3] - m[0][1];top.y = m[1][3] - m[1][1];top.z = m[2][3] - m[2][1];top.w = m[3][3] - m[3][1];

        vec4 bottom;
        bottom.x = m[0][3] + m[0][1];bottom.y = m[1][3] + m[1][1];bottom.z = m[2][3] + m[2][1];bottom.w = m[3][3] + m[3][1];
        
        __m128 planes[4] = {
            _mm_set_ps_bw(left.x, right.x, top.x, bottom.x),
            _mm_set_ps_bw(left.y, right.y, top.y, bottom.y),
            _mm_set_ps_bw(left.z, right.z, top.z, bottom.z),
            _mm_set_ps_bw(left.w, right.w, top.w, bottom.w),
        };

        std::vector<BSphere> spheres;
        for (int i = 0; i < 100 * 100 * 6; ++i)
        {
            BSphere bs{};
            bs.radius = 1;
            Transform ts{ vec3(1),quat(1,0,0,0),vec3(0,i,-10) };
            bs.UpdateCaches(ts);
            spheres.push_back(bs);
        }

        auto tp = TIME_HERE;

        for (int i = 0; i < 100*100*6; ++i)
        {
            simdCull(spheres[i], &planes[0]);
        }

        //registry.view<BSphere>().each([&planes](auto entity, BSphere& s) {simdCull(s, &planes[0]); });

        auto el = (int)ELAPSEDuS(tp);


        for (BSphere sphere : inView)
        {
            auto[pos, r] = sphere.GetCachedDataSlow();
            DrawSphere(pos, r, col32::white, 8);
        }


        times.emplace_back(el);

        float avg = accumulate(times.begin(), times.end(), 0) / (float)times.size();
        if (times.size() > 120) { times.erase(times.begin()); }
        maxavg = fmax(maxavg, avg);

        ImGui::Checkbox("draw", &draw);
        ImGui::SliderInt("microSeconds", &el, avg-100, avg+100);
        ImGui::SliderFloat("AVG microSeconds", &avg, 0, maxavg, "%.1f");

        inView.clear();

    });

};

void FrustumTest::Shutdown()
{

}

//todo macro?
int main(int argc, char** argv)
{
    FrustumTest app;
    return app.Run(argc, argv);
}
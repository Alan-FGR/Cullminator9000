#include <iostream>
#include "Bigged/bigged.cpp"
#include "entt.hpp"
#include <chrono>
#include <cmath>
#include <execution>

#define TIME_HERE std::chrono::high_resolution_clock::now();
#define ELAPSEDµS(time_point) (uint)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-time_point).count());

using namespace entt;

class FrustumTest : public TestBed {
    DefaultRegistry registry;
    virtual void Init() override;
    virtual void Update(float dt) override;
    virtual void Shutdown() override;
};

struct AABB
{
    vec3 min, max;
    AABB(vec3 min, vec3 max) : min(min), max(max) {}
};

struct BSphere
{
    vec3 position;
    float radius;
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
    //add stuff to cull
    for (int z = 0; z < 100; z++)
        for (int y = 0; y < 50; y++)
            for (int x = 0; x < 100; x++)
            {
                auto entity = registry.create();
                
//                 registry.assign<Transform>(entity,
//                     vec3(((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f),
//                     quat(1, 0, 0, 0),
//                     vec3((x - 5) * 5, y * 3, (z - 5) * 5));

//                 registry.assign<Velocity>(entity,
//                     vec3(0, 0, (std::rand() % 1000) / 100.f) * 0.01f,
//                     vec3((std::rand() % 1000) / 1000.f, (std::rand() % 1000) / 1000.f, (std::rand() % 1000) / 1000.f) * 0.1f
//                     );

                registry.assign<BSphere>(entity,
                    vec3((x - 50) * 2, (y - 3) * 2, (z - 50) * 2),
                    ((20 + std::rand()) % 100) / 100.f
                    );

            }

    //add cullers
    auto entity = registry.create();
    registry.assign<Transform>(entity, vec3(1), quat(1, 0, 0, 0), vec3(0));
    registry.assign<Velocity>(entity, vec3(0, 0.f, 0), vec3(0, 0.1f, 0));
    registry.assign<Frustum>(entity, 50.f, 0.01f, 500.f, 2.f);

    SetCameraPosition(vec3(-5, 30, 5));
    SetCameraYawPitch(-35, 90);
}

bool NaiveCull(vec3& pos, float& radius, vec4& plane)
{
    return plane.x * pos.x + plane.y * pos.y + plane.z * pos.z + plane.w <= -radius;
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
bool simd = true;

std::vector<uint> times;
uint maxavg;

void FrustumTest::Update(float dt)
{
    //DrawGrid();

    registry.view<Transform, Velocity>().each([this, &dt](auto entity, Transform& transform, Velocity& vel) {
        vel.ApplyToTransform(transform, dt);
        transform.DrawDebug();
    });


    registry.view<Transform, Frustum>().each([this](auto entity, Transform& transform, Frustum& fr) {
        
        mat4 frustumMat4 = fr.GetFrustumMatrix(transform);
        DrawFrustum(frustumMat4, col32::white);

        mat4& m = frustumMat4;

        vec4 right;
        right.x = m[0][3] + m[0][0];
        right.y = m[1][3] + m[1][0];
        right.z = m[2][3] + m[2][0];
        right.w = m[3][3] + m[3][0];

        vec4 left;
        left.x = m[0][3] - m[0][0];
        left.y = m[1][3] - m[1][0];
        left.z = m[2][3] - m[2][0];
        left.w = m[3][3] - m[3][0];

        vec4 top;
        top.x = m[0][3] - m[0][1];
        top.y = m[1][3] - m[1][1];
        top.z = m[2][3] - m[2][1];
        top.w = m[3][3] - m[3][1];

        vec4 bottom;
        bottom.x = m[0][3] + m[0][1];
        bottom.y = m[1][3] + m[1][1];
        bottom.z = m[2][3] + m[2][1];
        bottom.w = m[3][3] + m[3][1];

        //vec4 far;
        //far.x = m[0][2];
        //far.y = m[1][2];
        //far.z = m[2][2];
        //far.w = m[3][2];

        //vec4 near;
        //near.x = m[0][3] - m[0][2];
        //near.y = m[1][3] - m[1][2];
        //near.z = m[2][3] - m[2][2];
        //near.w = m[3][3] - m[3][2];


        const __m128 planes[4] = {
            _mm_set_ps_bw(left.x, right.x, top.x, bottom.x),
            _mm_set_ps_bw(left.y, right.y, top.y, bottom.y),
            _mm_set_ps_bw(left.z, right.z, top.z, bottom.z),
            _mm_set_ps_bw(left.w, right.w, top.w, bottom.w),
        };


       



        auto view = registry.view<BSphere>();

        auto tp = TIME_HERE;
        
        int culled = 0;

        if (simd) {
            std::for_each(std::execution::par, view.begin(), view.end(), [&view, &planes, &culled](const auto entity) {
                BSphere& s = view.get(entity);

                auto color = col32::white;

                __m128 fourSpheresX = _mm_set_ps1(s.position.x);
                __m128 fourSpheresY = _mm_set_ps1(s.position.y);
                __m128 fourSpheresZ = _mm_set_ps1(s.position.z);
                __m128 fourSpheresR = _mm_set_ps1(-s.radius);

                __m128 xs = _mm_mul_ps(planes[0], fourSpheresX);
                __m128 ys = _mm_mul_ps(planes[1], fourSpheresY);
                __m128 zs = _mm_mul_ps(planes[2], fourSpheresZ);

                __m128 added = _mm_add_ps(xs, ys, zs, planes[3]);

                __m128 results = _mm_cmplt_ps(added, fourSpheresR);

                if (_mm_movemask_ps(results)) {
                    color = col32::red;
                    culled++;
                }

                if (draw)
                    DrawSphere(s.position, s.radius, color, 5);

            });
        }
        else {
            std::for_each(std::execution::par, view.begin(), view.end(), [&view, &right, &left, &bottom, &top, &culled](const auto entity) {
                BSphere& s = view.get(entity);

                auto color = col32::white;

                if (NaiveCull(s.position, s.radius, right)) color = col32::red;
                else if (NaiveCull(s.position, s.radius, left)) color = col32::red;
                else if (NaiveCull(s.position, s.radius, bottom)) color = col32::red;
                else if (NaiveCull(s.position, s.radius, top)) color = col32::red;

                if (draw)
                    DrawSphere(s.position, s.radius, color, 5);
                else
                    culled++;

            });
        }

        auto el = (int)ELAPSEDµS(tp);

        times.emplace_back(el);

        float avg = accumulate(times.begin(), times.end(), 0) / (float)times.size();

        if (times.size() > 60) {
            times.erase(times.begin());
        }

        maxavg = fmax(maxavg, avg);

        ImGui::Checkbox("draw", &draw);
        ImGui::Checkbox("SIMD", &simd);
        ImGui::SliderInt("microSeconds", &el, avg-100, avg+100);
        ImGui::SliderFloat("AVG microSeconds", &avg, 0, maxavg, "%.1f");
        ImGui::Text("%d, %d", culled, 50 * 100 * 100 - culled);


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
#include <iostream>
#include "Bigged/bigged.cpp"
#include "entt.hpp"

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
    for (int z = 0; z < 10; z++)
        for (int y = 0; y < 2; y++)
            for (int x = 0; x < 10; x++)
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
                    vec3((x - 5) * 5, y * 3, (z - 5) * 5),
                    ((10 + std::rand()) % 100) / 100.f
                    );

            }

    //add cullers
    auto entity = registry.create();
    registry.assign<Transform>(entity, vec3(1), quat(1, 0, 0, 0), vec3(0));
    registry.assign<Velocity>(entity, vec3(0, 0.1f, 0), vec3(0, 1, 0));
    registry.assign<Frustum>(entity, 50.f, 0.01f, 50.f, 2.f);

    SetCameraPosition(vec3(-5, 30, 5));
    SetCameraYawPitch(-35, 90);
}

bool NaiveCull(vec3& pos, float& radius, vec4& plane)
{
    return plane.x * pos.x + plane.y * pos.y + plane.z * pos.z + plane.w <= -radius;
}

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

        registry.view<BSphere>().each([this, &frustumMat4](auto entity, BSphere& s) {
            auto color = col32::white;

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

            vec4 far;
            far.x = m[0][2];
            far.y = m[1][2];
            far.z = m[2][2];
            far.w = m[3][2];

            vec4 near;
            near.x = m[0][3] - m[0][2];
            near.y = m[1][3] - m[1][2];
            near.z = m[2][3] - m[2][2];
            near.w = m[3][3] - m[3][2];
            
            /*if (NaiveCull(s.position, s.radius, right)) color = col32::red;
            else if (NaiveCull(s.position, s.radius, left)) color = col32::red;
            else if (NaiveCull(s.position, s.radius, bottom)) color = col32::red;
            else if (NaiveCull(s.position, s.radius, top)) color = col32::red;*/



            __m128 sphereVec = _mm_load_ps(&s);
            //__m128 planeVec = _mm_load_ps(&left);
            //__m128 _mm_mul_ps(sphereVec, planeVec);


            __m128 fourSpheresX = _mm_splat_ps(sphereVec, 0);



            //plane.x * pos.x + plane.y * pos.y + plane.z * pos.z + plane.w <= -radius;
            



            DrawSphereAuto(s.position, s.radius, color);
        });

    });

}

void FrustumTest::Shutdown()
{

}

//todo macro?
int main(int argc, char** argv)
{
    FrustumTest app;
    return app.Run(argc, argv);
}
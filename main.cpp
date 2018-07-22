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
                
                registry.assign<Transform>(entity,
                    vec3(((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f, ((10 + std::rand()) % 100) / 100.f),
                    quat(1, 0, 0, 0),
                    vec3((x - 5) * 5, y * 3, (z - 5) * 5));

                registry.assign<Velocity>(entity,
                    vec3(0, 0, (std::rand() % 1000) / 100.f) * 0.01f,
                    vec3((std::rand() % 1000) / 1000.f, (std::rand() % 1000) / 1000.f, (std::rand() % 1000) / 1000.f) * 0.1f
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

void FrustumTest::Update(float dt)
{
    //DrawGrid();

    registry.view<Transform, Velocity>().each([this, &dt](auto entity, Transform& transform, Velocity& vel) {
        vel.ApplyToTransform(transform, dt);
        transform.DrawDebug();
    });

    registry.view<Transform, Frustum>().each([this](auto entity, Transform& transform, Frustum& fr) {
        DrawFrustum(fr.GetFrustumMatrix(transform), col32::white);
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
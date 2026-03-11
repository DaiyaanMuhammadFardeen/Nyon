#include "nyon/graphics/Renderer2D.h"
#include "nyon/graphics/PhysicsDebugRenderer.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <iostream>

using namespace Nyon::Graphics;
using namespace Nyon::ECS;

/**
 * @brief Comprehensive rendering demo showcasing all 2D shape rendering capabilities
 */
class RenderingDemo
{
public:
    void Run()
    {
        std::cout << "=== Advanced 2D Renderer Demo ===" << std::endl;
        
        // Initialize renderer
        Renderer2D::Init();
        
        // Setup camera
        Camera2D camera;
        camera.position = {640.0f, 360.0f};  // Center of 1280x720 screen
        camera.zoom = 1.0f;
        
        // Create debug renderer
        PhysicsDebugRenderer debugRenderer;
        debugRenderer.SetActiveFlags(DebugRenderFlag::All);
        
        // Test basic shapes
        TestBasicShapes(camera);
        
        // Test advanced shapes
        TestAdvancedShapes(camera);
        
        // Test filled shapes
        TestFilledShapes(camera);
        
        // Test physics collider visualization
        TestColliderVisualization(debugRenderer);
        
        // Cleanup
        Renderer2D::Shutdown();
        
        std::cout << "=== Demo Complete ===" << std::endl;
    }
    
private:
    void TestBasicShapes(const Camera2D& camera)
    {
        std::cout << "\nTesting Basic Shapes..." << std::endl;
        
        Renderer2D::BeginScene(camera);
        
        // Draw quads
        Renderer2D::DrawQuad({100.0f, 100.0f}, {50.0f, 50.0f}, {25.0f, 25.0f}, {1.0f, 0.0f, 0.0f});
        Renderer2D::DrawQuad({200.0f, 100.0f}, {50.0f, 50.0f}, {25.0f, 25.0f}, {0.0f, 1.0f, 0.0f});
        Renderer2D::DrawQuad({300.0f, 100.0f}, {50.0f, 50.0f}, {25.0f, 25.0f}, {0.0f, 0.0f, 1.0f});
        
        // Draw circles
        Renderer2D::DrawCircle({100.0f, 200.0f}, 25.0f, {1.0f, 1.0f, 0.0f});
        Renderer2D::DrawCircle({200.0f, 200.0f}, 25.0f, {0.0f, 1.0f, 1.0f});
        Renderer2D::DrawCircle({300.0f, 200.0f}, 25.0f, {1.0f, 0.0f, 1.0f});
        
        // Draw polygons
        std::vector<Math::Vector2> triangle = {
            {100.0f, 300.0f},
            {125.0f, 350.0f},
            {75.0f, 350.0f}
        };
        Renderer2D::DrawPolygon(triangle, {1.0f, 0.5f, 0.0f});
        
        std::vector<Math::Vector2> square = {
            {200.0f, 300.0f},
            {250.0f, 300.0f},
            {250.0f, 350.0f},
            {200.0f, 350.0f}
        };
        Renderer2D::DrawPolygon(square, {0.5f, 0.0f, 1.0f});
        
        // Draw lines
        Renderer2D::DrawLine({50.0f, 400.0f}, {350.0f, 400.0f}, {1.0f, 1.0f, 1.0f}, 1.0f);
        Renderer2D::DrawLine({50.0f, 420.0f}, {350.0f, 420.0f}, {1.0f, 1.0f, 1.0f}, 3.0f);
        Renderer2D::DrawLine({50.0f, 440.0f}, {350.0f, 440.0f}, {1.0f, 1.0f, 1.0f}, 5.0f);
        
        Renderer2D::EndScene();
    }
    
    void TestAdvancedShapes(const Camera2D& camera)
    {
        std::cout << "\nTesting Advanced Shapes..." << std::endl;
        
        Renderer2D::BeginScene(camera);
        
        // Draw capsules
        Renderer2D::DrawCapsule({100.0f, 100.0f}, {100.0f, 200.0f}, 15.0f, {1.0f, 0.0f, 0.0f});
        Renderer2D::DrawCapsule({200.0f, 100.0f}, {300.0f, 200.0f}, 15.0f, {0.0f, 1.0f, 0.0f});
        
        // Draw segments with thickness
        Renderer2D::DrawSegment({100.0f, 300.0f}, {200.0f, 300.0f}, 10.0f, {1.0f, 1.0f, 0.0f});
        Renderer2D::DrawSegment({100.0f, 350.0f}, {200.0f, 350.0f}, 20.0f, {0.0f, 1.0f, 1.0f});
        
        // Draw chains
        std::vector<Math::Vector2> chainPoints = {
            {300.0f, 300.0f},
            {350.0f, 320.0f},
            {400.0f, 300.0f},
            {450.0f, 320.0f},
            {500.0f, 300.0f}
        };
        Renderer2D::DrawChain(chainPoints, {1.0f, 0.0f, 1.0f}, 5.0f, false);
        
        // Draw ellipses
        Renderer2D::DrawEllipse({100.0f, 500.0f}, 40.0f, 25.0f, {1.0f, 0.5f, 0.0f});
        Renderer2D::DrawEllipse({200.0f, 500.0f}, 25.0f, 40.0f, {0.5f, 0.0f, 1.0f});
        
        // Draw arcs
        Renderer2D::DrawArc({350.0f, 500.0f}, 40.0f, 0.0f, 3.14159f / 2.0f, 
                           {1.0f, 1.0f, 1.0f}, 3.0f);
        
        // Draw sectors
        Renderer2D::DrawSector({450.0f, 500.0f}, 40.0f, 0.0f, 3.14159f, 
                              {0.0f, 1.0f, 0.5f});
        
        Renderer2D::EndScene();
    }
    
    void TestFilledShapes(const Camera2D& camera)
    {
        std::cout << "\nTesting Filled Shapes..." << std::endl;
        
        Renderer2D::BeginScene(camera);
        
        // Filled circles
        Renderer2D::DrawSolidCircle({100.0f, 100.0f}, 30.0f, {1.0f, 0.0f, 0.0f});
        Renderer2D::DrawSolidCircle({200.0f, 100.0f}, 30.0f, {0.0f, 1.0f, 0.0f});
        
        // Filled polygons
        std::vector<Math::Vector2> hexagon = {
            {300.0f, 85.0f},
            {330.0f, 100.0f},
            {330.0f, 130.0f},
            {300.0f, 145.0f},
            {270.0f, 130.0f},
            {270.0f, 100.0f}
        };
        Renderer2D::DrawSolidPolygon(hexagon, {0.0f, 0.0f, 1.0f});
        
        // Filled capsules
        Renderer2D::DrawSolidCapsule({100.0f, 250.0f}, {200.0f, 250.0f}, 20.0f, {1.0f, 1.0f, 0.0f});
        
        // Filled ellipses
        Renderer2D::DrawSolidEllipse({350.0f, 250.0f}, 50.0f, 30.0f, {1.0f, 0.0f, 1.0f});
        
        // Filled sectors
        Renderer2D::DrawSolidSector({500.0f, 250.0f}, 40.0f, 0.0f, 3.14159f / 3.0f, {0.0f, 1.0f, 1.0f});
        
        Renderer2D::EndScene();
    }
    
    void TestColliderVisualization(PhysicsDebugRenderer& debugRenderer)
    {
        std::cout << "\nTesting Collider Visualization..." << std::endl;
        
        Camera2D camera;
        camera.position = {640.0f, 360.0f};
        camera.zoom = 1.0f;
        
        Renderer2D::BeginScene(camera);
        
        // Create test colliders
        TransformComponent circleTransform;
        circleTransform.position = {200.0f, 200.0f};
        circleTransform.rotation = 0.0f;
        
        ColliderComponent circleCollider(25.0f);
        debugRenderer.DrawCollider(circleCollider, circleTransform, {1.0f, 0.0f, 0.0f});
        
        // Polygon collider
        TransformComponent polyTransform;
        polyTransform.position = {400.0f, 200.0f};
        polyTransform.rotation = 0.5f;
        
        ColliderComponent::PolygonShape polyShape({
            {-25.0f, -25.0f},
            {25.0f, -25.0f},
            {25.0f, 25.0f},
            {-25.0f, 25.0f}
        });
        ColliderComponent polyCollider(polyShape);
        debugRenderer.DrawCollider(polyCollider, polyTransform, {0.0f, 1.0f, 0.0f});
        
        // Capsule collider
        TransformComponent capsuleTransform;
        capsuleTransform.position = {600.0f, 200.0f};
        capsuleTransform.rotation = 0.3f;
        
        ColliderComponent::CapsuleShape capsuleShape;
        capsuleShape.center1 = {-20.0f, -20.0f};
        capsuleShape.center2 = {20.0f, 20.0f};
        capsuleShape.radius = 10.0f;
        
        ColliderComponent capsuleCollider(capsuleShape);
        debugRenderer.DrawCollider(capsuleCollider, capsuleTransform, {0.0f, 0.0f, 1.0f});
        
        Renderer2D::EndScene();
    }
};

int main()
{
    RenderingDemo demo;
    demo.Run();
    return 0;
}

#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <variant>
#include <string>
#include <cstdint>
#include <algorithm>
namespace Nyon::ECS {
    struct ColliderComponent {
        enum class ShapeType { 
            Circle, 
            Polygon, 
            Capsule, 
            Segment,
            Chain,
            Composite  };
        struct CircleShape {
            Math::Vector2 center = {0.0f, 0.0f};
            float radius = 16.0f; };
        struct PolygonShape {
            std::vector<Math::Vector2> vertices;
            std::vector<Math::Vector2> normals;   
            Math::Vector2 centroid = {0.0f, 0.0f};  
            float radius = 0.0f;  
            PolygonShape() = default;
            PolygonShape(const std::vector<Math::Vector2>& verts) : vertices(verts) {
                CalculateProperties(); }
            void CalculateProperties() {
                if (vertices.empty()) return;
                if (!IsCounterClockwise()) {
                    std::reverse(vertices.begin(), vertices.end()); }
                centroid = {0.0f, 0.0f};
                float twoArea = 0.0f;
                size_t n = vertices.size();
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    float cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
                    twoArea += cross;
                    centroid.x += (vertices[i].x + vertices[j].x) * cross;
                    centroid.y += (vertices[i].y + vertices[j].y) * cross; }
                if (std::abs(twoArea) > 0.0002f) {
                    centroid = centroid * (1.0f / (3.0f * twoArea));   } else {
                    centroid = {0.0f, 0.0f};
                    for (const auto& vertex : vertices) {
                        centroid = centroid + vertex; }
                    centroid = centroid * (1.0f / vertices.size()); }
                normals.clear();
                for (size_t i = 0; i < vertices.size(); ++i) {
                    size_t next = (i + 1) % vertices.size();
                    Math::Vector2 edge = vertices[next] - vertices[i];
                    Math::Vector2 normal = {edge.y, -edge.x};  
                    float length = sqrt(normal.x * normal.x + normal.y * normal.y);
                    if (length > 0.0001f) {
                        normal = normal * (1.0f / length);
                        normals.push_back(normal); } } }
            bool IsCounterClockwise() const {
                if (vertices.size() < 3) return true;
                float area = 0.0f;
                size_t n = vertices.size();
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    area += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y; }
                return area > 0.0f; } };
        struct CapsuleShape {
            Math::Vector2 center1 = {0.0f, 0.0f};
            Math::Vector2 center2 = {0.0f, 32.0f};
            float radius = 8.0f; };
        struct SegmentShape {
            Math::Vector2 point1 = {0.0f, 0.0f};
            Math::Vector2 point2 = {32.0f, 0.0f};
            float radius = 0.0f;   };
        struct ChainShape {
            std::vector<Math::Vector2> vertices;
            bool isLoop = false;  
            float radius = 0.0f; };
        struct CompositeShape {
            std::vector<std::variant<CircleShape, PolygonShape, CapsuleShape, SegmentShape>> subShapes; };
        ShapeType type = ShapeType::Polygon;
        Math::Vector3 color = {1.0f, 1.0f, 1.0f};  
        float density = 1.0f;  
        std::variant<CircleShape, PolygonShape, CapsuleShape, SegmentShape, ChainShape, CompositeShape> shape;
        struct Filter {
            uint16_t categoryBits = 0x0001;   
            uint16_t maskBits = 0xFFFF;       
            int16_t groupIndex = 0;           
            bool ShouldCollide(const Filter& other) const {
                if (groupIndex != 0 && groupIndex == other.groupIndex) {
                    return groupIndex > 0;   }
                bool collideA = (categoryBits & other.maskBits) != 0;
                bool collideB = (other.categoryBits & maskBits) != 0;
                return collideA && collideB; } };
        Filter filter;
        bool isSensor = false;            
        bool enableContactEvents = true;  
        bool enableSensorEvents = true;   
        bool enableHitEvents = false;     
        struct Material {
            float friction = 0.2f;        
            float restitution = 0.0f;     
            float density = 1.0f;         
            uint64_t userData = 0;        
            std::string name = "default";   };
        Material material;
        int proxyId = -1;                 
        bool forceUpdate = false;         
        ColliderComponent() {
            shape = PolygonShape({{0, 0}, {32, 0}, {32, 32}, {0, 32}}); }
        ColliderComponent(const PolygonShape& poly) : type(ShapeType::Polygon) {
            shape = poly; }
        ColliderComponent(float radius) : type(ShapeType::Circle) {
            shape = CircleShape{{0, 0}, radius}; }
        ColliderComponent(const CircleShape& circle) : type(ShapeType::Circle) {
            shape = circle; }
        ColliderComponent(const CapsuleShape& capsule) : type(ShapeType::Capsule) {
            shape = capsule; }
        ColliderComponent(const SegmentShape& segment) : type(ShapeType::Segment) {
            shape = segment; }
        template<typename T>
        T& GetShape() { return std::get<T>(shape); }
        template<typename T>
        const T& GetShape() const { return std::get<T>(shape); }
        PolygonShape& GetPolygon() { return std::get<PolygonShape>(shape); }
        const PolygonShape& GetPolygon() const { return std::get<PolygonShape>(shape); }
        CircleShape& GetCircle() { return std::get<CircleShape>(shape); }
        const CircleShape& GetCircle() const { return std::get<CircleShape>(shape); }
        CapsuleShape& GetCapsule() { return std::get<CapsuleShape>(shape); }
        const CapsuleShape& GetCapsule() const { return std::get<CapsuleShape>(shape); }
        SegmentShape& GetSegment() { return std::get<SegmentShape>(shape); }
        const SegmentShape& GetSegment() const { return std::get<SegmentShape>(shape); }
        void CalculateAABB(const Math::Vector2& position, float rotation, Math::Vector2& outMin, Math::Vector2& outMax) const {
            const float speculativeDistance = 0.1f;  
            switch (type) {
                case ShapeType::Circle: {
                    const auto& circle = GetCircle();
                    Math::Vector2 worldCenter = circle.center + position;
                    float radius = circle.radius + speculativeDistance;
                    outMin = {worldCenter.x - radius, worldCenter.y - radius};
                    outMax = {worldCenter.x + radius, worldCenter.y + radius};
                    break; }
                case ShapeType::Polygon: {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.empty()) {
                        outMin = position;
                        outMax = position;
                        return; }
                    if (std::abs(rotation) > 1e-6f) {
                        float cosTheta = std::cos(rotation);
                        float sinTheta = std::sin(rotation);
                        Math::Vector2 rotatedVertex = {
                            polygon.vertices[0].x * cosTheta - polygon.vertices[0].y * sinTheta,
                            polygon.vertices[0].x * sinTheta + polygon.vertices[0].y * cosTheta };
                        outMin = rotatedVertex + position;
                        outMax = outMin;
                        for (size_t i = 1; i < polygon.vertices.size(); ++i) {
                            rotatedVertex = {
                                polygon.vertices[i].x * cosTheta - polygon.vertices[i].y * sinTheta,
                                polygon.vertices[i].x * sinTheta + polygon.vertices[i].y * cosTheta };
                            Math::Vector2 worldVertex = rotatedVertex + position;
                            outMin.x = std::min(outMin.x, worldVertex.x);
                            outMin.y = std::min(outMin.y, worldVertex.y);
                            outMax.x = std::max(outMax.x, worldVertex.x);
                            outMax.y = std::max(outMax.y, worldVertex.y); } }
                    else {
                        outMin = polygon.vertices[0] + position;
                        outMax = outMin;
                        for (const auto& vertex : polygon.vertices) {
                            Math::Vector2 worldVertex = vertex + position;
                            outMin.x = std::min(outMin.x, worldVertex.x);
                            outMin.y = std::min(outMin.y, worldVertex.y);
                            outMax.x = std::max(outMax.x, worldVertex.x);
                            outMax.y = std::max(outMax.y, worldVertex.y); } }
                    outMin.x -= speculativeDistance;
                    outMin.y -= speculativeDistance;
                    outMax.x += speculativeDistance;
                    outMax.y += speculativeDistance;
                    break; }
                case ShapeType::Capsule: {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 center1 = capsule.center1 + position;
                    Math::Vector2 center2 = capsule.center2 + position;
                    Math::Vector2 min = {
                        std::min(center1.x, center2.x) - capsule.radius,
                        std::min(center1.y, center2.y) - capsule.radius };
                    Math::Vector2 max = {
                        std::max(center1.x, center2.x) + capsule.radius,
                        std::max(center1.y, center2.y) + capsule.radius };
                    outMin = min;
                    outMax = max;
                    break; }
                case ShapeType::Segment: {
                    const auto& segment = GetSegment();
                    Math::Vector2 p1 = segment.point1 + position;
                    Math::Vector2 p2 = segment.point2 + position;
                    outMin = {
                        std::min(p1.x, p2.x) - segment.radius,
                        std::min(p1.y, p2.y) - segment.radius };
                    outMax = {
                        std::max(p1.x, p2.x) + segment.radius,
                        std::max(p1.y, p2.y) + segment.radius };
                    break; }
                default:
                    outMin = position;
                    outMax = position;
                    break; } }
        float CalculateArea() const {
            switch (type) {
                case ShapeType::Circle: {
                    const auto& circle = GetCircle();
                    return 3.14159f * circle.radius * circle.radius; }
                case ShapeType::Polygon: {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.size() < 3) return 0.0f;
                    float area = 0.0f;
                    size_t n = polygon.vertices.size();
                    for (size_t i = 0; i < n; ++i) {
                        size_t j = (i + 1) % n;
                        area += polygon.vertices[i].x * polygon.vertices[j].y;
                        area -= polygon.vertices[j].x * polygon.vertices[i].y; }
                    return std::abs(area) * 0.5f; }
                case ShapeType::Capsule: {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 diff = capsule.center2 - capsule.center1;
                    float length = sqrt(diff.x * diff.x + diff.y * diff.y);
                    return 3.14159f * capsule.radius * capsule.radius + length * 2 * capsule.radius; }
                default:
                    return 0.0f; } }
        float CalculateInertiaForUnitDensity() const {
            switch (type) {
                case ShapeType::Circle: {
                    const auto& circle = GetCircle();
                    return 0.5f * circle.radius * circle.radius; }
                case ShapeType::Polygon: {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.size() < 3) return 0.0f;
                    const auto& verts = polygon.vertices;
                    size_t n = verts.size();
                    float numerator = 0.0f;
                    float twoArea = 0.0f;
                    for (size_t i = 0; i < n; ++i) {
                        const Math::Vector2& p0 = verts[i];
                        const Math::Vector2& p1 = verts[(i + 1) % n];
                        float cross = std::abs(Math::Vector2::Cross(p0, p1));
                        numerator += cross * (
                            p0.x * p0.x + p0.x * p1.x + p1.x * p1.x +
                            p0.y * p0.y + p0.y * p1.y + p1.y * p1.y
                        );
                        twoArea += cross; }
                    return numerator / 12.0f; }
                case ShapeType::Capsule: {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 diff = capsule.center2 - capsule.center1;
                    float h = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                    float r = capsule.radius;
                    float rectArea = 2.0f * r * h;
                    float rectInertia = rectArea * (h * h + (2.0f * r) * (2.0f * r)) / 12.0f;
                    float circleArea = 3.14159f * r * r;
                    float circleInertia = 0.5f * 3.14159f * r * r * r * r * 2.0f;  
                    return rectInertia + circleInertia; }
                default:
                    return 0.0f; } }
        float CalculateMass(float densityOverride = -1.0f) const {
            float actualDensity = (densityOverride > 0.0f) ? densityOverride : material.density;
            return CalculateArea() * actualDensity; }
        ShapeType GetType() const { return type; }
        bool IsSensor() const { return isSensor; }
        void SetSensor(bool sensor) { isSensor = sensor; }
        void SetFilter(const Filter& newFilter) { filter = newFilter; }
        Filter GetFilter() const { return filter; }
        void GetBounds(const Math::Vector2& position, Math::Vector2& outMin, Math::Vector2& outMax) const {
            CalculateAABB(position, 0.0f, outMin, outMax); } }; }

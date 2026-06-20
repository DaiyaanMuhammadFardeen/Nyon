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
        ChainShape& GetChain() { return std::get<ChainShape>(shape); }
        const ChainShape& GetChain() const { return std::get<ChainShape>(shape); }
        CompositeShape& GetComposite() { return std::get<CompositeShape>(shape); }
        const CompositeShape& GetComposite() const { return std::get<CompositeShape>(shape); }
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
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto rotate2D = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR }; };
                    Math::Vector2 center1 = rotate2D(capsule.center1) + position;
                    Math::Vector2 center2 = rotate2D(capsule.center2) + position;
                    Math::Vector2 min = {
                        std::min(center1.x, center2.x) - capsule.radius,
                        std::min(center1.y, center2.y) - capsule.radius };
                    Math::Vector2 max = {
                        std::max(center1.x, center2.x) + capsule.radius,
                        std::max(center1.y, center2.y) + capsule.radius };
                    outMin = { min.x - speculativeDistance, min.y - speculativeDistance };
                    outMax = { max.x + speculativeDistance, max.y + speculativeDistance };
                    break; }
                case ShapeType::Segment: {
                    const auto& segment = GetSegment();
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto rotate2D = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR }; };
                    Math::Vector2 p1 = rotate2D(segment.point1) + position;
                    Math::Vector2 p2 = rotate2D(segment.point2) + position;
                    outMin = {
                        std::min(p1.x, p2.x) - segment.radius - speculativeDistance,
                        std::min(p1.y, p2.y) - segment.radius - speculativeDistance };
                    outMax = {
                        std::max(p1.x, p2.x) + segment.radius + speculativeDistance,
                        std::max(p1.y, p2.y) + segment.radius + speculativeDistance };
                    break; }
                case ShapeType::Chain: {
                    const auto& chain = GetChain();
                    if (chain.vertices.empty()) {
                        outMin = position;
                        outMax = position;
                        break; }
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto rotate2D = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR }; };
                    Math::Vector2 first = rotate2D(chain.vertices[0]) + position;
                    outMin = first;
                    outMax = first;
                    for (size_t i = 1; i < chain.vertices.size(); ++i) {
                        Math::Vector2 v = rotate2D(chain.vertices[i]) + position;
                        outMin.x = std::min(outMin.x, v.x);
                        outMin.y = std::min(outMin.y, v.y);
                        outMax.x = std::max(outMax.x, v.x);
                        outMax.y = std::max(outMax.y, v.y); }
                    float r = std::max(chain.radius + speculativeDistance, speculativeDistance);
                    outMin.x -= r;
                    outMin.y -= r;
                    outMax.x += r;
                    outMax.y += r;
                    break; }
                case ShapeType::Composite: {
                    const auto& composite = GetComposite();
                    if (composite.subShapes.empty()) {
                        outMin = position;
                        outMax = position;
                        break; }
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto compRotate = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR }; };
                    bool first = true;
                    for (const auto& sub : composite.subShapes) {
                        Math::Vector2 subMin, subMax;
                        if (std::holds_alternative<ColliderComponent::CircleShape>(sub)) {
                            const auto* c = std::get_if<ColliderComponent::CircleShape>(&sub);
                            subMin = Math::Vector2(position.x - c->radius - speculativeDistance, position.y - c->radius - speculativeDistance);
                            subMax = Math::Vector2(position.x + c->radius + speculativeDistance, position.y + c->radius + speculativeDistance); }
                        else if (std::holds_alternative<ColliderComponent::PolygonShape>(sub)) {
                            const auto* p = std::get_if<ColliderComponent::PolygonShape>(&sub);
                            Math::Vector2 pFirst = compRotate(p->vertices[0]) + position;
                            subMin = subMax = pFirst;
                            for (const auto& v : p->vertices) {
                                Math::Vector2 rv = compRotate(v) + position;
                                subMin.x = std::min(subMin.x, rv.x);
                                subMin.y = std::min(subMin.y, rv.y);
                                subMax.x = std::max(subMax.x, rv.x);
                                subMax.y = std::max(subMax.y, rv.y); }
                            subMin.x -= speculativeDistance;
                            subMin.y -= speculativeDistance;
                            subMax.x += speculativeDistance;
                            subMax.y += speculativeDistance; }
                        else {
                            subMin = Math::Vector2(position.x - speculativeDistance, position.y - speculativeDistance);
                            subMax = Math::Vector2(position.x + speculativeDistance, position.y + speculativeDistance); }
                        if (first) {
                            outMin = subMin;
                            outMax = subMax;
                            first = false; }
                        else {
                            outMin.x = std::min(outMin.x, subMin.x);
                            outMin.y = std::min(outMin.y, subMin.y);
                            outMax.x = std::max(outMax.x, subMax.x);
                            outMax.y = std::max(outMax.y, subMax.y); } }
                    break; }
                default:
                    outMin = position - Math::Vector2{ speculativeDistance, speculativeDistance };
                    outMax = position + Math::Vector2{ speculativeDistance, speculativeDistance };
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
        float CalculateInertiaPerUnitMass() const {
            switch (type) {
                case ShapeType::Circle: {
                    const auto& circle = GetCircle();
                    return 0.5f * circle.radius * circle.radius; }
                case ShapeType::Polygon: {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.size() < 3) return 0.0f;
                    const auto& verts = polygon.vertices;
                    const auto& centroid = polygon.centroid;
                    size_t n = verts.size();
                    float numerator = 0.0f;
                    float twoArea = 0.0f;
                    for (size_t i = 0; i < n; ++i) {
                        Math::Vector2 p0 = verts[i] - centroid;
                        Math::Vector2 p1 = verts[(i + 1) % n] - centroid;
                        float cross = p0.x * p1.y - p1.x * p0.y;
                        twoArea += cross;
                        numerator += cross * (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x +
                                              p0.y * p0.y + p0.y * p1.y + p1.y * p1.y); }
                    if (std::abs(twoArea) < 1e-6f) return 0.0f;
                    return std::abs(numerator / (6.0f * twoArea)); }
                case ShapeType::Capsule: {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 diff = capsule.center2 - capsule.center1;
                    float h = sqrt(diff.x * diff.x + diff.y * diff.y);
                    float r = capsule.radius;
                    float areaRect = h * 2.0f * r;
                    float areaCircles = 3.14159f * r * r;
                    float totalArea = areaRect + areaCircles;
                    float iRectPerMass = (h * h + 4.0f * r * r) / 12.0f;
                    float massRectFrac = areaRect / totalArea;
                float offset = h / 2.0f;
                float iCirclePerMass = 0.5f * r * r + offset * offset;
                float massCircleFrac = areaCircles / totalArea;
                return iRectPerMass * massRectFrac + iCirclePerMass * massCircleFrac; }
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

#pragma once

#include "nyon/math/Vector2.h"
#include <string>

namespace Nyon::ECS
{
    /**
     * @brief Joint component for connecting physics bodies.
     * 
     * Implements various joint types inspired by Box2D for constraining body motion.
     * Supports distance, revolute, prismatic, and weld joints.
     * 
     * WARNING: Joint solver is NOT implemented in this engine version!
     * This component is defined for future implementation only.
     * Do not use - joints will have no physical effect.
     */
    struct JointComponent
    {
        // Note: Joint solver is not implemented in this engine version.
        // This component is defined for future implementation only.
        // Joints will have no physical effect until solver is added.
        
        // === JOINT TYPES ===
        enum class Type
        {
            Distance,    // Maintains fixed distance between anchor points
            Revolute,    // Allows rotation around anchor point
            Prismatic,   // Allows translation along axis
            Weld,        // Welds bodies together completely
            Wheel,       // Wheel joint with spring suspension
            Motor        // Motor joint for controlled motion
        };
        
        // === BASE JOINT PROPERTIES ===
        Type type = Type::Distance;
        uint32_t entityIdA = 0;         // First connected entity
        uint32_t entityIdB = 0;         // Second connected entity
        Math::Vector2 localAnchorA = {0.0f, 0.0f}; // Anchor point on body A (local coordinates)
        Math::Vector2 localAnchorB = {0.0f, 0.0f}; // Anchor point on body B (local coordinates)
        bool collideConnected = false;  // Whether connected bodies can collide
        std::string name = "joint";     // Joint name for debugging
        
        // === JOINT-SPECIFIC DATA ===
        // Separate structs instead of union to avoid undefined behavior with non-trivial members
        struct DistanceJointData
        {
            float length = 1.0f;        // Target distance
            float frequencyHz = 0.0f;   // Spring frequency (0 = rigid)
            float dampingRatio = 0.0f;  // Spring damping ratio
        } distanceJoint;
        
        struct RevoluteJointData
        {
            bool enableLimit = false;       // Enable rotation limits
            float lowerAngle = 0.0f;        // Lower rotation limit (radians)
            float upperAngle = 0.0f;        // Upper rotation limit (radians)
            bool enableMotor = false;       // Enable motor
            float motorSpeed = 0.0f;        // Motor speed (radians/sec)
            float maxMotorTorque = 0.0f;    // Maximum motor torque
            float referenceAngle = 0.0f;    // Reference angle for limits
        } revoluteJoint;
        
        struct PrismaticJointData
        {
            Math::Vector2 localAxisA = {1.0f, 0.0f}; // Translation axis (body A local)
            bool enableLimit = false;       // Enable translation limits
            float lowerTranslation = 0.0f;  // Lower translation limit
            float upperTranslation = 0.0f;  // Upper translation limit
            bool enableMotor = false;       // Enable motor
            float motorSpeed = 0.0f;        // Motor speed
            float maxMotorForce = 0.0f;     // Maximum motor force
            float referenceAngle = 0.0f;    // Reference angle
        } prismaticJoint;
        
        struct WeldJointData
        {
            float referenceAngle = 0.0f;    // Reference angle between bodies
            float frequencyHz = 0.0f;       // Spring frequency
            float dampingRatio = 0.0f;      // Spring damping ratio
        } weldJoint;
        
        struct WheelJointData
        {
            Math::Vector2 localAxisA = {0.0f, 1.0f}; // Suspension axis (body A local)
            bool enableMotor = false;       // Enable motor
            float motorSpeed = 0.0f;        // Motor speed
            float maxMotorTorque = 0.0f;    // Maximum motor torque
            float springFrequencyHz = 2.0f; // Spring frequency
            float springDampingRatio = 0.7f; // Spring damping ratio
        } wheelJoint;
        
        struct MotorJointData
        {
            Math::Vector2 linearOffset = {0.0f, 0.0f}; // Target linear offset
            float angularOffset = 0.0f;     // Target angular offset
            float maxForce = 1000.0f;       // Maximum linear force
            float maxTorque = 100.0f;       // Maximum angular torque
            float correctionFactor = 0.3f;  // Position correction factor
        } motorJoint;
        
        // === RUNTIME DATA ===
        uint32_t jointId = 0;           // Unique joint identifier
        bool isActive = true;           // Whether joint is active
        float breakForce = 0.0f;        // Force at which joint breaks (0 = unbreakable)
        float breakTorque = 0.0f;       // Torque at which joint breaks (0 = unbreakable)
        
        // === CONSTRUCTORS ===
        JointComponent() = default;
        
        // Distance Joint Constructor
        JointComponent(Type jointType, uint32_t entityA, uint32_t entityB, 
                      const Math::Vector2& anchorA, const Math::Vector2& anchorB)
            : type(jointType), entityIdA(entityA), entityIdB(entityB)
            , localAnchorA(anchorA), localAnchorB(anchorB)
        {
            switch (jointType)
            {
                case Type::Distance:
                    // WARNING: This computes local-space distance, not world-space!
                    // Caller MUST manually set correct world-space length after construction:
                    //   joint.distanceJoint.length = (worldPosB + anchorB - (worldPosA + anchorA)).Length();
                    distanceJoint.length = (anchorB - anchorA).Length();
                    break;
                case Type::Revolute:
                    revoluteJoint.referenceAngle = 0.0f;
                    break;
                case Type::Prismatic:
                    prismaticJoint.referenceAngle = 0.0f;
                    break;
                case Type::Weld:
                    weldJoint.referenceAngle = 0.0f;
                    break;
                default:
                    break;
            }
        }
        
        // === UTILITY METHODS ===
        Type GetType() const { return type; }
        uint32_t GetEntityA() const { return entityIdA; }
        uint32_t GetEntityB() const { return entityIdB; }
        bool GetCollideConnected() const { return collideConnected; }
        void SetCollideConnected(bool collide) { collideConnected = collide; }
        
        // === JOINT STATE ACCESSORS ===
        void SetActive(bool active) { isActive = active; }
        bool IsActive() const { return isActive; }
        
        void SetBreakForce(float force) { breakForce = force; }
        void SetBreakTorque(float torque) { breakTorque = torque; }
        float GetBreakForce() const { return breakForce; }
        float GetBreakTorque() const { return breakTorque; }
        
        // === JOINT TYPE SPECIFIC METHODS ===
        
        // Distance Joint Methods
        void SetDistanceJoint(float length, float frequency = 0.0f, float damping = 0.0f)
        {
            if (type == Type::Distance)
            {
                distanceJoint.length = length;
                distanceJoint.frequencyHz = frequency;
                distanceJoint.dampingRatio = damping;
            }
        }
        
        float GetDistanceJointLength() const 
        { 
            return (type == Type::Distance) ? distanceJoint.length : 0.0f; 
        }
        
        // Revolute Joint Methods
        void SetRevoluteJointLimits(bool enable, float lower, float upper)
        {
            if (type == Type::Revolute)
            {
                revoluteJoint.enableLimit = enable;
                revoluteJoint.lowerAngle = lower;
                revoluteJoint.upperAngle = upper;
            }
        }
        
        void SetRevoluteJointMotor(bool enable, float speed, float maxTorque)
        {
            if (type == Type::Revolute)
            {
                revoluteJoint.enableMotor = enable;
                revoluteJoint.motorSpeed = speed;
                revoluteJoint.maxMotorTorque = maxTorque;
            }
        }
        
        // Prismatic Joint Methods
        void SetPrismaticJointLimits(bool enable, float lower, float upper)
        {
            if (type == Type::Prismatic)
            {
                prismaticJoint.enableLimit = enable;
                prismaticJoint.lowerTranslation = lower;
                prismaticJoint.upperTranslation = upper;
            }
        }
        
        void SetPrismaticJointMotor(bool enable, float speed, float maxForce)
        {
            if (type == Type::Prismatic)
            {
                prismaticJoint.enableMotor = enable;
                prismaticJoint.motorSpeed = speed;
                prismaticJoint.maxMotorForce = maxForce;
            }
        }
        
        // Weld Joint Methods
        void SetWeldJointSpring(float frequency, float damping)
        {
            if (type == Type::Weld)
            {
                weldJoint.frequencyHz = frequency;
                weldJoint.dampingRatio = damping;
            }
        }
    };
}

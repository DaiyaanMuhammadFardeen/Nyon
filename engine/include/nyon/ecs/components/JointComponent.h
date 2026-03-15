#pragma once
#include "nyon/math/Vector2.h"
#include <string>
namespace Nyon::ECS {
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
    struct JointComponent {
        enum class Type {
            Distance,     
            Revolute,     
            Prismatic,    
            Weld,         
            Wheel,        
            Motor         
        };
        Type type = Type::Distance;
        uint32_t entityIdA = 0;          
        uint32_t entityIdB = 0;          
        Math::Vector2 localAnchorA = {0.0f, 0.0f};  
        Math::Vector2 localAnchorB = {0.0f, 0.0f};  
        bool collideConnected = false;   
        std::string name = "joint";      
        struct DistanceJointData {
            float length = 1.0f;         
            float frequencyHz = 0.0f;    
            float dampingRatio = 0.0f;   
        } distanceJoint;
        struct RevoluteJointData {
            bool enableLimit = false;        
            float lowerAngle = 0.0f;         
            float upperAngle = 0.0f;         
            bool enableMotor = false;        
            float motorSpeed = 0.0f;         
            float maxMotorTorque = 0.0f;     
            float referenceAngle = 0.0f;     
        } revoluteJoint;
        struct PrismaticJointData {
            Math::Vector2 localAxisA = {1.0f, 0.0f};  
            bool enableLimit = false;        
            float lowerTranslation = 0.0f;   
            float upperTranslation = 0.0f;   
            bool enableMotor = false;        
            float motorSpeed = 0.0f;         
            float maxMotorForce = 0.0f;      
            float referenceAngle = 0.0f;     
        } prismaticJoint;
        struct WeldJointData {
            float referenceAngle = 0.0f;     
            float frequencyHz = 0.0f;        
            float dampingRatio = 0.0f;       
        } weldJoint;
        struct WheelJointData {
            Math::Vector2 localAxisA = {0.0f, 1.0f};  
            bool enableMotor = false;        
            float motorSpeed = 0.0f;         
            float maxMotorTorque = 0.0f;     
            float springFrequencyHz = 2.0f;  
            float springDampingRatio = 0.7f;  
        } wheelJoint;
        struct MotorJointData {
            Math::Vector2 linearOffset = {0.0f, 0.0f};  
            float angularOffset = 0.0f;      
            float maxForce = 1000.0f;        
            float maxTorque = 100.0f;        
            float correctionFactor = 0.3f;   
        } motorJoint;
        uint32_t jointId = 0;            
        bool isActive = true;            
        float breakForce = 0.0f;         
        float breakTorque = 0.0f;        
        JointComponent() = default;
        JointComponent(Type jointType, uint32_t entityA, uint32_t entityB, 
                      const Math::Vector2& anchorA, const Math::Vector2& anchorB)
            : type(jointType), entityIdA(entityA), entityIdB(entityB)
            , localAnchorA(anchorA), localAnchorB(anchorB)
        {
            switch (jointType)
            {
                case Type::Distance:
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
        Type GetType() const { return type; }
        uint32_t GetEntityA() const { return entityIdA; }
        uint32_t GetEntityB() const { return entityIdB; }
        bool GetCollideConnected() const { return collideConnected; }
        void SetCollideConnected(bool collide) { collideConnected = collide; }
        void SetActive(bool active) { isActive = active; }
        bool IsActive() const { return isActive; }
        void SetBreakForce(float force) { breakForce = force; }
        void SetBreakTorque(float torque) { breakTorque = torque; }
        float GetBreakForce() const { return breakForce; }
        float GetBreakTorque() const { return breakTorque; }
        void SetDistanceJoint(float length, float frequency = 0.0f, float damping = 0.0f)
        {
            if (type == Type::Distance)
            {
                distanceJoint.length = length;
                distanceJoint.frequencyHz = frequency;
                distanceJoint.dampingRatio = damping;
            }
        }
        float GetDistanceJointLength() const { 
            return (type == Type::Distance) ? distanceJoint.length : 0.0f; 
        }
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

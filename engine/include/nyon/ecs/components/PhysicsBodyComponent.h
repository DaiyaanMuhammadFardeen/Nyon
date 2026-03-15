#pragma once
#include "nyon/math/Vector2.h"
namespace Nyon::ECS {
    struct PhysicsBodyComponent {
        Math::Vector2 velocity = {0.0f, 0.0f};       
        Math::Vector2 acceleration = {0.0f, 0.0f};   
        Math::Vector2 force = {0.0f, 0.0f};          
        float angularVelocity = 0.0f;                
        float angularAcceleration = 0.0f;            
        float torque = 0.0f;                         
        float mass = 1.0f;                           
        float inverseMass = 1.0f;                    
        float inertia = 1.0f;                        
        float inverseInertia = 1.0f;                 
        Math::Vector2 centerOfMass = {0.0f, 0.0f};   
        float friction = 0.1f;                       
        float restitution = 0.0f;                    
        float angularDamping = 0.0f;                 
        float drag = 0.0f;                           
        float maxLinearSpeed = 1000.0f;              
        float maxAngularSpeed = 100.0f;              
        bool isStatic = false;                       
        bool isKinematic = false;                    
        bool isBullet = false;                       
        bool isAwake = true;                         
        bool allowSleep = true;                      
        float sleepTimer = 0.0f;                     
        static constexpr float TIME_TO_SLEEP = 0.5f;            
        static constexpr float LINEAR_SLEEP_TOLERANCE = 0.01f;  
        static constexpr float ANGULAR_SLEEP_TOLERANCE = 0.01f;  
        bool isGrounded = false;                     
        int groundedFrames = 0;                      
        static constexpr int GROUNDED_THRESHOLD = 2;  
        struct MotionLocks {
            bool lockTranslationX = false;
            bool lockTranslationY = false;
            bool lockRotation = false; } motionLocks;
        PhysicsBodyComponent() = default;
        PhysicsBodyComponent(float m) : mass(m) { UpdateMassProperties(); }
        PhysicsBodyComponent(float m, bool stat) : mass(m), isStatic(stat) { UpdateMassProperties(); }
        void SetMass(float newMass) {
            mass = newMass;
            UpdateMassProperties(); }
        void SetInertia(float newInertia) {
            inertia = newInertia;
            inverseInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f; }
        void UpdateMassProperties() {
            if (isStatic || isKinematic || mass <= 0.0f) {
                inverseMass = 0.0f;
                if (inertia > 0.0f) {
                    inverseInertia = 0.0f; } }
            else {
                inverseMass = 1.0f / mass;
                if (inertia > 0.0f) {
                    inverseInertia = 1.0f / inertia; }
                else {
                    inertia = 0.0f;
                    inverseInertia = 0.0f; } } }
        void SetAwake(bool awake) {
            if (isStatic) return;
            isAwake = awake;
            if (awake) {
                sleepTimer = 0.0f; } }
        void AllowSleep(bool enable) {
            allowSleep = enable;
            if (!enable) {
                SetAwake(true); } }
        void ApplyForce(const Math::Vector2& forceVec) {
            if (isStatic) return;
            force = force + forceVec;
            SetAwake(true); }
        void ApplyForceAtPoint(const Math::Vector2& forceVec, const Math::Vector2& point) {
            if (isStatic) return;
            force = force + forceVec;
            Math::Vector2 offset = point - centerOfMass;
            torque += offset.x * forceVec.y - offset.y * forceVec.x;
            SetAwake(true); }
        void ApplyTorque(float torqueAmount) {
            if (isStatic) return;
            torque += torqueAmount;
            SetAwake(true); }
        void ApplyLinearImpulse(const Math::Vector2& impulse) {
            if (isStatic) return;
            velocity = velocity + impulse * inverseMass;
            SetAwake(true); }
        void ApplyAngularImpulse(float impulse) {
            if (isStatic) return;
            angularVelocity += impulse * inverseInertia;
            SetAwake(true); }
        void ClearForces() {
            force = {0.0f, 0.0f};
            torque = 0.0f; }
        bool IsStablyGrounded() const { return groundedFrames >= GROUNDED_THRESHOLD; }
        void UpdateGroundedState(bool currentlyGrounded) {
            if (currentlyGrounded) {
                groundedFrames++; } else {
                groundedFrames = 0; }
            isGrounded = IsStablyGrounded(); }
        bool IsStatic() const { return isStatic; }
        bool IsKinematic() const { return isKinematic; }
        bool IsDynamic() const { return !isStatic && !isKinematic; }
        bool ShouldCollide() const { return true; }   
        float GetMass() const { return mass; }
        float GetInertia() const { return inertia; } }; }

#pragma once
#include "nyon/math/Vector2.h"
#include <vector>
#include <cstdint>
namespace Nyon::ECS {
    struct ContactPoint {
        Math::Vector2 position = {0.0f, 0.0f};           
        Math::Vector2 normal = {0.0f, 0.0f};             
        float separation = 0.0f;                         
        float normalImpulse = 0.0f;                      
        float tangentImpulse = 0.0f;                     
        float normalMass = 0.0f;                         
        float tangentMass = 0.0f;                        
        float velocityBias = 0.0f;                       
        uint32_t featureId = 0;                          
        bool persisted = false;                           };
    struct ContactManifold {
        std::vector<ContactPoint> points;            
        Math::Vector2 normal = {0.0f, 0.0f};         
        Math::Vector2 localNormal = {0.0f, 0.0f};    
        Math::Vector2 localPoint = {0.0f, 0.0f};     
        float friction = 0.0f;                       
        float restitution = 0.0f;                    
        float tangentSpeed = 0.0f;                   
        uint32_t entityIdA = 0;                      
        uint32_t entityIdB = 0;                      
        uint32_t shapeIdA = 0;                       
        uint32_t shapeIdB = 0;                       
        bool touching = false;                       
        bool persisted = false;                       }; }

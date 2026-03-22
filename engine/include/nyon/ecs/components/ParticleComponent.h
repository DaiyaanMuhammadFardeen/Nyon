#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
namespace Nyon::ECS {
    struct ParticleComponent {
        float lifetime = -1.0f;        
        float age = 0.0f;              
        bool alive = true;             
        float alpha = 1.0f;            
        float alphaStart = 1.0f;       
        float alphaEnd = 0.0f;         
        Math::Vector3 colorStart{1.0f, 1.0f, 1.0f};   
        Math::Vector3 colorEnd{1.0f, 1.0f, 1.0f};     
        float sizeScale = 1.0f;        
        EntityID emitterEntityId = INVALID_ENTITY;
        uint64_t userData = 0;         
        float prevAlpha = 1.0f;
        float prevSizeScale = 1.0f;
        Math::Vector3 prevColorStart{1.0f, 1.0f, 1.0f};
        Math::Vector3 prevColorEnd{1.0f, 1.0f, 1.0f};
        void Reset() {
            lifetime = -1.0f;
            age = 0.0f;
            alive = true;
            alpha = 1.0f;
            alphaStart = 1.0f;
            alphaEnd = 0.0f;
            colorStart = {1.0f, 1.0f, 1.0f};
            colorEnd = {1.0f, 1.0f, 1.0f};
            sizeScale = 1.0f;
            emitterEntityId = INVALID_ENTITY;
            userData = 0;
            prevAlpha = 1.0f;
            prevSizeScale = 1.0f;
            prevColorStart = {1.0f, 1.0f, 1.0f};
            prevColorEnd = {1.0f, 1.0f, 1.0f}; } }; }

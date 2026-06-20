#pragma once
#include "EntityManager.h"
#include <vector>
#include <typeindex>
#include <memory>
#include <cassert>
#include <algorithm>   
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <stdexcept>
namespace Nyon::ECS {
    class ComponentStore {
    private:
        struct IComponentContainer {
            virtual ~IComponentContainer() = default;
            virtual void RemoveComponent(EntityID entity) = 0;
            virtual bool HasComponent(EntityID entity) const = 0;
            virtual EntityID GetEntityAtIndex(size_t index) const = 0;
            virtual size_t GetComponentCount() const = 0; };
        template<typename T>
        struct ComponentContainer : public IComponentContainer {
            std::vector<T> components;            
            std::vector<EntityID> entityIds;      
            std::vector<bool> activeFlags;        
            std::unordered_map<EntityID, size_t> indexMap;  
            void RemoveComponent(EntityID entity) override {
                auto it = indexMap.find(entity);
                if (it == indexMap.end()) return;
                size_t idx = it->second;
                size_t last = components.size() - 1;
                if (idx != last) {
                    components[idx]  = std::move(components[last]);
                    entityIds[idx]   = entityIds[last];
                    activeFlags[idx] = activeFlags[last];
                    indexMap[entityIds[idx]] = idx; }
                components.pop_back();
                entityIds.pop_back();
                activeFlags.pop_back();
                indexMap.erase(it); }
            bool HasComponent(EntityID entity) const override {
                auto it = indexMap.find(entity);
                return it != indexMap.end() && activeFlags[it->second]; }
            EntityID GetEntityAtIndex(size_t index) const override {
                if (index < entityIds.size() && activeFlags[index]) {
                    return entityIds[index]; }
                return INVALID_ENTITY; }
            size_t GetComponentCount() const override {
                return indexMap.size(); }
            void AddComponent(EntityID entity, T&& component) {
                auto it = indexMap.find(entity);
                if (it != indexMap.end() && activeFlags[it->second]) {
                    components[it->second] = std::forward<T>(component);
                    return; }
                indexMap[entity] = components.size();
                components.push_back(std::forward<T>(component));
                entityIds.push_back(entity);
                activeFlags.push_back(true); }
            T& GetComponentByIndex(size_t index) {
                assert(index < components.size() && activeFlags[index]);
                return components[index]; }
            const T& GetComponentByIndex(size_t index) const {
                assert(index < components.size() && activeFlags[index]);
                return components[index]; }
            const std::vector<EntityID>& GetEntities() const {
                return entityIds; }
            const std::vector<bool>& GetActiveFlags() const {
                return activeFlags; } };
    public:
        ComponentStore(EntityManager& entityManager);
        template<typename T>
        void AddComponent(EntityID entity, T&& component) {
            assert(m_EntityManager.IsEntityValid(entity));
            auto& container = GetOrCreateContainer<T>();
            container.AddComponent(entity, std::forward<T>(component)); }
        template<typename T>
        void RemoveComponent(EntityID entity) {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                static_cast<ComponentContainer<T>*>(containerIt->second.get())->RemoveComponent(entity); } }
        template<typename T>
        T& GetComponent(EntityID entity) {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt == m_Containers.end()) {
                std::ostringstream oss;
                oss << "ComponentStore::GetComponent - Component type not registered: "
                    << typeid(T).name() << " (entity=" << entity << ")";
                std::cerr << "[FATAL ERROR] " << oss.str() << std::endl;
                throw std::runtime_error(oss.str()); }
            auto& container = *static_cast<ComponentContainer<T>*>(containerIt->second.get());
            auto mapIt = container.indexMap.find(entity);
            if (mapIt == container.indexMap.end() || !container.activeFlags[mapIt->second]) {
                std::ostringstream oss;
                oss << "ComponentStore::GetComponent - Entity does not have this component: "
                    << typeid(T).name() << " (entity=" << entity << ")";
                std::cerr << "[FATAL ERROR] " << oss.str() << std::endl;
                throw std::runtime_error(oss.str()); }
            return container.components[mapIt->second]; }
        template<typename T>
        const T& GetComponent(EntityID entity) const {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt == m_Containers.end()) {
                std::ostringstream oss;
                oss << "ComponentStore::GetComponent - Component type not registered: "
                    << typeid(T).name() << " (entity=" << entity << ")";
                std::cerr << "[FATAL ERROR] " << oss.str() << std::endl;
                throw std::runtime_error(oss.str()); }
            const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
            auto mapIt = container.indexMap.find(entity);
            if (mapIt == container.indexMap.end() || !container.activeFlags[mapIt->second]) {
                std::ostringstream oss;
                oss << "ComponentStore::GetComponent - Entity does not have this component: "
                    << typeid(T).name() << " (entity=" << entity << ")";
                std::cerr << "[FATAL ERROR] " << oss.str() << std::endl;
                throw std::runtime_error(oss.str()); }
            return container.components[mapIt->second]; }
        template<typename T>
        bool HasComponent(EntityID entity) const {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                return containerIt->second->HasComponent(entity); }
            return false; }
        template<typename T>
        const std::vector<EntityID>& GetEntitiesWithComponent() const {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
                return container.GetEntities(); }
            static std::vector<EntityID> empty;
            return empty; }
        template<typename T, typename Func>
        void ForEachComponent(Func&& func) {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                auto& container = *static_cast<ComponentContainer<T>*>(containerIt->second.get());
                for (size_t i = 0; i < container.entityIds.size(); ++i) {
                    if (container.activeFlags[i]) {
                        func(container.entityIds[i], container.components[i]); } } } }
        template<typename T, typename Func>
        void ForEachComponent(Func&& func) const {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
                for (size_t i = 0; i < container.entityIds.size(); ++i) {
                    if (container.activeFlags[i]) {
                        func(container.entityIds[i], container.components[i]); } } } }
        template<typename T>
        size_t GetComponentCount() const {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                return containerIt->second->GetComponentCount(); }
            return 0; }
        void RemoveAllComponents(EntityID entity);
    private:
        EntityManager& m_EntityManager;
        std::unordered_map<std::type_index, std::unique_ptr<IComponentContainer>> m_Containers;
        template<typename T>
        ComponentContainer<T>& GetOrCreateContainer() {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            if (it == m_Containers.end()) {
                auto container = std::make_unique<ComponentContainer<T>>();
                it = m_Containers.emplace(typeIdx, std::move(container)).first; }
            return *static_cast<ComponentContainer<T>*>(it->second.get()); }
        template<typename T>
        ComponentContainer<T>& GetContainer() {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            assert(it != m_Containers.end());
            return *static_cast<ComponentContainer<T>*>(it->second.get()); }
        template<typename T>
        const ComponentContainer<T>& GetContainer() const {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            assert(it != m_Containers.end());
            return *static_cast<const ComponentContainer<T>*>(it->second.get()); } }; }
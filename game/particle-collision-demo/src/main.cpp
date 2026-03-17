#include "ParticleCollisionDemo.h"
#include <iostream>

int main() {
    try {
        Nyon::ParticleCollisionDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
#include "PhysicsDemoGame.h"
#include <iostream>

int main()
{
    try
    {
        std::cout << "=====================================\n";
        std::cout << "  Nyon Physics Demo Game\n";
        std::cout << "  Box2D-Inspired Physics Playground\n";
        std::cout << "=====================================\n";
        std::cout << "Starting game...\n";
        
        Game::PhysicsDemoGame game;
        game.Run();
        
        std::cout << "\nGame finished successfully!\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred\n";
        return -1;
    }
}
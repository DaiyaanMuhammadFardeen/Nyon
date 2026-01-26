#include "GameApplication.h"

int main()
{
    GameApplication app;
    // Don't call OnStart() manually - it's called by Run()
    app.Run();
    
    return 0;
}
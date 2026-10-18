#include<algorithm>
#include<array>
#include<raylib.h>
using namespace std;
int main(){
    InitWindow(1200,1200,"Simulation of AeroFleet!");
    while(!WindowShouldClose()){
        BeginDrawing();
        ClearBackground(RAYWHITE);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
//
//  main.cpp
//  Reconing
//
//  Created by apple on 07/04/2021.
//

#include <iostream>
#include "Engine.hpp"

// M O D U L E S /////////////////////////////
#include "Modules/ImGuiDemoWindow.hpp"
#include "Modules/Pipeline.hpp"


int main(int argc, const char * argv[]) {
    Engine engine;
    engine.register_module(new ImGuiDemoWindowModule());
    engine.register_module(new PipelineModule());
    return engine.run();
}

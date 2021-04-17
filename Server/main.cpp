//
//  main.cpp
//  Server
//
//  Created by apple on 17/04/2021.
//

#include <iostream>
#include "Server.hpp"


int main(int argc, const char * argv[]) {
    Server server;
    server.init();
    server.run();
    return 0;
}

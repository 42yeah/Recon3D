//
//  Server.hpp
//  Server
//
//  Created by apple on 17/04/2021.
//

#ifndef Server_hpp
#define Server_hpp

#include <iostream>
#include <set>
#include <optional>
#include <vector>
#include <filesystem>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "online.pb.h"

#define RECON_PORT 17290

class Server {
public:
    Server() {}
    
    ~Server();
    
    auto init() -> bool;
    
    auto run() -> bool;
    
    template<typename T>
    auto receive(int sock) -> std::optional<T>;
    
    template<typename T>
    auto send(int sock, T what) -> bool;
    
private:
    auto mkdir_if_not_exists(std::filesystem::path path) -> void;

    std::set<int> sockets;
    bool ready;
    int server_sock;
    online::Users users;
    online::ReconRecords records;
};

#endif /* Server_hpp */

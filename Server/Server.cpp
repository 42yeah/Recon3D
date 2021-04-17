//
//  Server.cpp
//  Server
//
//  Created by apple on 17/04/2021.
//

#include "Server.hpp"
#include <unistd.h>
#include <thread>
#include <mutex>

template<typename T>
auto request_append(online::Request &request, T last) -> void {
    request.add_arg(std::string(last));
}

template<typename T, typename ...Ar>
auto request_append(online::Request &request, T first, Ar... args) -> void {
    request.add_arg(std::string(first));
    request_append(request, args...);
}

template<typename ...Ar>
auto make_request(Ar... args) -> online::Request {
    online::Request request;
    request_append(request, args...);
    return request;
}

auto Server::init() -> bool {
    ready = false;
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in sin;
    sin.sin_addr.s_addr = INADDR_ANY;
    sin.sin_port = htons(RECON_PORT);
    sin.sin_family = AF_INET;
    if (bind(server_sock, (sockaddr *) &sin, sizeof(sin)) == -1) {
        std::cerr << "服务器启动错误！无法监听端口：" << RECON_PORT << std::endl;
        return false;
    }
    listen(server_sock, 5);
    std::cout << "Recon 服务器启动完毕，正在监听" << std::endl;
    ready = true;
    return true;
}

#define BAIL(why) mutex.lock(); \
    sockets.erase(client_sock); \
    close(client_sock); \
    std::cerr << why << std::endl; \
    mutex.unlock(); \
    return;

auto Server::run() -> bool {
    std::mutex mutex;
    if (!ready) {
        std::cerr << "服务器尚未准备完毕" << std::endl;
        return false;
    }
    while (true) {
        int client_sock;
        sockaddr_in client_sockaddr;
        socklen_t slen = sizeof(client_sockaddr);
        client_sock = accept(server_sock, (sockaddr *) &client_sock, &slen);

        std::cout << "有新连接：" << client_sockaddr.sin_addr.s_addr << std::endl;
        sockets.insert(client_sock);
        std::thread thread([&, client_sock] () {
            online::User user;
            bool logged_in = false;

            while (true) {
                auto req_opt = receive<online::Request>(client_sock);
                if (!req_opt.has_value()) {
                    BAIL("数据接收异常，正在关闭连接。");
                }
                const auto request = *req_opt;
                for (auto i = 0; i < request.arg_size(); i++) {
                    std::cout << request.arg(i) << " ";
                }
                std::cout << std::endl;
                if (request.arg_size() == 0) {
                    send(client_sock, make_request("error", "no args"));
                } else if (request.arg_size() == 1) {
                    const auto &cmd = request.arg(0);
                    
                    if (cmd == "login") {
                        std::cout << "用户正在尝试登陆..." << std::endl;
                        auto credentials = receive<online::User>(client_sock);
                        if (!credentials.has_value()) {
                            BAIL("用户信息接收异常，正在关闭连接。");
                        }
                        auto located = std::find_if(users.begin(), users.end(), [&] (auto &u) {
                            return u.username() == credentials->username() &&
                            u.password() == credentials->password();
                        });
                        if (located != users.end()) {
                            user = *located;
                            logged_in = true;
                            std::cout << "用户成功证明自己是 " << user.username() << std::endl;
                            send(client_sock, make_request("success"));
                            return;
                        }
                        send(client_sock, make_request("error", "wrong credentials"));
                    } else if (cmd == "register") {
                        std::cout << "用户正在尝试注册..." << std::endl;
                        auto user = receive<online::User>(client_sock);
                        if (!user.has_value()) {
                            BAIL("用户信息接收异常，正在关闭连接。");
                        }
                        if (!user->has_username() || !user->has_password()) {
                            send(client_sock, make_request("error", "not enough credentials"));
                            return;
                        }
                        if (std::find_if(users.begin(), users.end(), [&] (auto &u) {
                            return u.username() == user->username();
                        }) != users.end()) {
                            send(client_sock, make_request("error", "user exist"));
                            return;
                        }
                        users.push_back(*user);
                        send(client_sock, make_request("success"));
                    }
                } else if (request.arg_size() == 2) {
                    const auto &cmd = request.arg(0);
                    
                    if (cmd == "hello") {
                        if (request.arg(1) != "server") {
                            send(client_sock, make_request("error", "fuck off"));
                            BAIL("客户端未能正确指代服务器。正在断开连接...");
                        } else {
                            send(client_sock, make_request("hello"));
                            std::cout << "与新连接成功互相打招呼。连接已经稳定。" << std::endl;
                        }
                    }
                } else {
                    send(client_sock, make_request("error", "unknown request"));
                }
            }
        });
        thread.detach();
    }
    return true;
}

template<typename T>
auto Server::receive(int sock) -> std::optional<T> {
    int request_size;
    
    auto recv_len = recv(sock, &request_size, sizeof(request_size), 0);
    if (recv_len != sizeof(request_size)) {
        return {};
    }
    
    char *packet = new char[request_size];
    recv_len = recv(sock, packet, request_size, 0);
    if (recv_len != request_size) {
        delete[] packet;
        return {};
    }
    
    T t;
    t.ParseFromArray(packet, request_size);
    
    delete[] packet;
    return t;
}

template<typename T>
auto Server::send(int sock, T what) -> bool {
    int size = (int) what.ByteSizeLong();
    char *packet = new char[size];
    what.SerializeToArray(packet, size);

    if (::send(sock, (char *) &size, sizeof(size), 0) != sizeof(size)) {
        delete[] packet;
        return false;
    }
    if (::send(sock, packet, size, 0) != size) {
        delete[] packet;
        return false;
    }
    
    delete[] packet;
    return true;
}

Server::~Server() { 
    if (ready) {
        ready = false;
        close(server_sock);
        for (auto sock : sockets) {
            close(sock);
        }
    }
}



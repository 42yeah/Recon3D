//
//  Server.cpp
//  Server
//
//  Created by apple on 17/04/2021.
//

#include "Server.hpp"
#include <unistd.h>
#include <thread>
#include <fstream>
#include <mutex>
#include <sstream>
#include <fstream>

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
    
    mkdir_if_not_exists("uploads");
    std::ifstream records_reader("uploads/records.bin");
    if (records_reader.good()) {
        records.ParseFromIstream(&records_reader);
        records_reader.close();
    }
    std::ifstream users_reader("uploads/users.bin");
    if (users_reader.good()) {
        users.ParseFromIstream(&users_reader);
        users_reader.close();
    }
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
                        auto located = -1;
                        for (auto i = 0; i < users.users_size(); i++) {
                            if (users.users(i).username() == credentials->username() &&
                                users.users(i).password() == credentials->password()) {
                                located = true;
                                break;
                            }
                        }
                        if (located != -1) {
                            user = users.users(located);
                            logged_in = true;
                            std::cout << "用户成功证明自己是 " << user.username() << std::endl;
                            send(client_sock, make_request("success"));
                            continue;
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
                            continue;
                        }
                        bool found = false;
                        for (auto i = 0; i < users.users_size(); i++) {
                            if (users.users(i).username() == user->username()) {
                                found = true;
                                break;
                            }
                        }
                        if (found) {
                            send(client_sock, make_request("error", "user exist"));
                            continue;
                        }
                        auto *new_user = users.add_users();
                        new_user->set_id((int) users.users_size() - 1);
                        new_user->set_username(user->username());
                        new_user->set_password(user->password());
                        
                        send(client_sock, make_request("success"));
                        std::ofstream users_writer("uploads/users.bin");
                        if (users_writer.good()) {
                            users.SerializeToOstream(&users_writer);
                            users_writer.close();
                        }
                    } else if (logged_in && cmd == "records") {
                        std::cout << user.username() << " 正在访问重建记录" << std::endl;
                        send(client_sock, records);
                    } else if (logged_in && cmd == "upload") {
                        std::cout << "用户正在尝试上传" << std::endl;
                        auto buffer = receive<online::ReconBuffer>(client_sock);
                        if (!buffer.has_value()) {
                            BAIL("用户上传数据失败，正在关闭连接。");
                        }
                        if (buffer->has_file_base() && buffer->has_obj_content() &&
                            buffer->has_mtl_content() && buffer->has_texture_content()) {
                            auto base = user.username() + "/" + std::filesystem::path(buffer->file_base()).filename().string();
                            online::ReconRecord * record = nullptr;
                            for (auto i = 0; i < records.records_size(); i++) {
                                if (records.records(i).name() == base) {
                                    std::cerr << "用户尝试上传的记录已经存在。正在覆盖..." << std::endl;
                                    record = records.mutable_records(i);
                                }
                            }
                            if (record == nullptr) {
                                record = records.add_records();
                            }
                            mkdir_if_not_exists("uploads");
                            mkdir_if_not_exists(std::string("uploads/") + user.username());
                            std::ofstream obj_writer(std::string("uploads/") + base + ".obj");
                            std::ofstream mtl_writer(std::string("uploads/") + base + ".mtl");
                            std::ofstream tex_writer(std::string("uploads/") + base + ".png");
                            if (!obj_writer.good() || !mtl_writer.good() || !tex_writer.good()) {
                                std::cerr << "尝试写入文件失败：" << base << std::endl;
                                if (obj_writer.good()) {
                                    obj_writer.close();
                                }
                                if (mtl_writer.good()) {
                                    mtl_writer.close();
                                }
                                if (tex_writer.good()) {
                                    tex_writer.close();
                                }
                                send(client_sock, make_request("error", "failed to open file"));
                                continue;
                            }
                            obj_writer << buffer->obj_content();
                            mtl_writer << buffer->mtl_content();
                            tex_writer << buffer->texture_content();
                            obj_writer.close();
                            mtl_writer.close();
                            tex_writer.close();
                            send(client_sock, make_request("success"));
                            std::cout << "文件上传成功。正在保存记录..." << std::endl;

                            record->set_id(records.records_size());
                            record->set_name(base);
                            record->set_owner(user.username());
                            
                            std::ofstream records_writer("uploads/records.bin");
                            if (records_writer.good()) {
                                records.SerializeToOstream(&records_writer);
                                records_writer.close();
                            }
                        } else {
                            std::cerr << "上传数据未齐全。";
                            send(client_sock, make_request("error", "buffer not complete"));
                        }
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
                    } else if (logged_in && cmd == "download") {
                        std::cout << "用户正在尝试下载数据" << std::endl;
                        auto id = std::stoi(request.arg(1));
                        const online::ReconRecord *record = nullptr;
                        for (auto i = 0; i < records.records_size(); i++) {
                            if (records.records(i).id() == id) {
                                record = &records.records(i);
                                break;
                            }
                        }
                        if (record == nullptr) {
                            std::cerr << "记录条未找到：" << id << std::endl;
                            send(client_sock, make_request("error", "record not found"));
                            continue;
                        }
                        // U P L O A D ////////////////////////////////////////
                        std::stringstream obj_stream, mtl_stream, tex_stream;
                        std::string base = "uploads/" + record->name();
                        std::ifstream obj_reader(base + ".obj"), mtl_reader(base + ".mtl"), tex_reader(base + ".png");
                        if (!obj_reader.good() || !mtl_reader.good() || !tex_reader.good()) {
                            if (obj_reader.good()) {
                                obj_reader.close();
                            }
                            if (mtl_reader.good()) {
                                mtl_reader.close();
                            }
                            if (tex_reader.good()) {
                                tex_reader.close();
                            }
                            std::cerr << "文件未找到：" << base << std::endl;
                            send(client_sock, make_request("error", "file not found"));
                            continue;
                        }
                        obj_stream << obj_reader.rdbuf();
                        mtl_stream << mtl_reader.rdbuf();
                        tex_stream << tex_reader.rdbuf();
                        obj_reader.close();
                        mtl_reader.close();
                        tex_reader.close();
                        online::ReconBuffer buffer;
                        buffer.set_file_base(record->name());
                        buffer.set_obj_content(obj_stream.str());
                        buffer.set_mtl_content(mtl_stream.str());
                        buffer.set_texture_content(tex_stream.str());
                        if (!send(client_sock, buffer)) {
                            BAIL("发送重建包给客户端失败。");
                        }
                        std::cout << "文件发送成功。" << std::endl;
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
    auto left_to_receive = request_size;
    auto offset = 0;
    while (left_to_receive != 0) {
        recv_len = recv(sock, packet + offset, left_to_receive, 0);
        if (recv_len <= 0) {
            delete[] packet;
            return {};
        }
        offset += recv_len;
        left_to_receive -= recv_len;
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

auto Server::mkdir_if_not_exists(std::filesystem::path path) -> void { 
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directory(path);
    }
}


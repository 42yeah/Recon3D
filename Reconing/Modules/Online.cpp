//
//  Online.cpp
//  Reconing
//
//  Created by apple on 17/04/2021.
//

#include "Online.hpp"
#include <imgui.h>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include "online.pb.h"

using namespace OnlineNS;


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

auto OnlineModule::update(float delta_time) -> bool {
    return false;
}

auto OnlineModule::update_ui() -> void {
    ImGui::SetNextWindowPos({ (float) (window_size.x / 2.0f - 310), 10 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 300, 600 }, ImGuiCond_FirstUseEver);
    ImGui::Begin("在线功能");

    switch (state) {
        case State::WELCOME:
            ImGui::TextWrapped("如果你想启用在线功能的话，可以从这里启用。");
            if (ImGui::Button("启用")) {
                connect();
            }
            break;
            
        case State::CONNECTING:
            ImGui::TextWrapped("稍等...");
            break;

        case State::LOGIN:
            ImGui::TextWrapped("你需要先登陆才能使用在线功能。");
            ImGui::InputText("账户名", username, sizeof(username));
            ImGui::InputText("密码", password, sizeof(password), ImGuiInputTextFlags_Password);
            if (ImGui::Button("登陆")) {
                login();
            }
            ImGui::Separator();
            ImGui::TextWrapped("没有账户？");
            ImGui::SameLine();
            if (ImGui::Button("注册")) {
                state = State::REGISTER;
            }
            break;

        case State::REGISTER:
            ImGui::InputText("账户名", username, sizeof(username));
            ImGui::InputText("密码", password, sizeof(password), ImGuiInputTextFlags_Password);
            if (ImGui::Button("注册")) {
                register_account();
            }
            ImGui::Separator();
            ImGui::TextWrapped("已有账户？");
            ImGui::SameLine();
            if (ImGui::Button("登陆")) {
                state = State::LOGIN;
            }
            break;
            
        case State::MAIN_INTERFACE:
            ImGui::TextWrapped("在线功能：在这里，你可以选择你想上传到服务器与其他人分享的重建历史，也可以下载他人的重建历史来观赏。");
            if (ImGui::Button("上传")) {
                // TODO: upload
            }
            
            break;
    }

    ImGui::End();
}

#define BAIL(why) mutex().lock(); \
    RECON_LOG(ONLINE) << why; \
    state = State::WELCOME; \
    close(sock); \
    mutex().unlock(); \
    return;

auto OnlineModule::connect() -> void {
    std::thread thread([&] () {
        mutex().lock();
        state = State::CONNECTING;
        mutex().unlock();
        sock = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in sin;
        inet_pton(AF_INET, ONLINE_REMOTE, &sin.sin_addr.s_addr);
        sin.sin_port = htons(ONLINE_PORT);
        sin.sin_family = AF_INET;
        
        if (::connect(sock, (sockaddr *) &sin, sizeof(sin)) == -1) {
            BAIL("与服务器建立连接失败。");
        }
        mutex().lock();
        RECON_LOG(ONLINE) << "与服务器建立连接成功。";
        mutex().unlock();

        auto request = make_request("hello", "server");
        send(request);
        auto response = receive<online::Request>();
        if (!response.has_value() || response->arg(0) != "hello") {
            BAIL("错误！服务器返回了未知数据。正在断开...");
        }
        mutex().lock();
        RECON_LOG(ONLINE) << "与服务器连接已稳定。";
        state = State::LOGIN;
        mutex().unlock();
    });
    thread.detach();
}

auto OnlineModule::login() -> void { 
    std::thread thread([&] () {
        mutex().lock();
        state = State::CONNECTING;
        mutex().unlock();
        
        if (!send(make_request("login"))) {
            BAIL("发送信息给服务器失败。请重新建立连接。");
        }
        
        online::User user;
        user.set_username(std::string(username));
        user.set_password(std::string(password));
        if (!send(user)) {
            BAIL("发送用户信息失败。请重新建立连接。");
        }
        
        auto response = receive<online::Request>();
        if (response.has_value() && response->arg_size() == 1 && response->arg(0) == "success") {
            mutex().lock();
            RECON_LOG(ONLINE) << "登陆成功。";
            state = State::MAIN_INTERFACE;
            mutex().unlock();
            return;
        }
        mutex().lock();
        RECON_LOG(ONLINE) << "登陆失败。请重试。";
        state = State::LOGIN;
        mutex().unlock();
    });
    thread.detach();
}

auto OnlineModule::register_account() -> void { 
    std::thread thread([&] () {
        mutex().lock();
        state = State::CONNECTING;
        mutex().unlock();
        
        if (!send(make_request("register"))) {
            BAIL("发送信息给服务器失败。请重新建立连接。");
        }
        
        online::User user;
        user.set_username(std::string(username));
        user.set_password(std::string(password));
        if (!send(user)) {
            BAIL("发送用户信息失败。请重新建立连接。");
        }
        
        auto response = receive<online::Request>();
        if (response.has_value() && response->arg_size() == 1 && response->arg(0) == "success") {
            mutex().lock();
            RECON_LOG(ONLINE) << "注册成功。可以登陆了。";
            state = State::LOGIN;
            mutex().unlock();
            return;
        }
        mutex().lock();
        RECON_LOG(ONLINE) << "注册失败。请重试。";
        state = State::REGISTER;
        mutex().unlock();
    });
    thread.detach();
}



template<typename T>
auto OnlineModule::receive() -> std::optional<T> {
    int request_size;
    
    auto recv_len = recv(sock, &request_size, sizeof(request_size), 0);
    if (recv_len != sizeof(request_size)) {
        return {};
    }
    
    char *packet = new char[request_size];
    auto left_to_receive = request_size;
    while (left_to_receive != 0) {
        recv_len = recv(sock, packet, left_to_receive, 0);
        if (recv_len <= 0) {
            delete[] packet;
            return {};
        }
        left_to_receive -= recv_len;
    }
    
    T t;
    t.ParseFromArray(packet, request_size);
    
    delete[] packet;
    return t;
}

template<typename T>
auto OnlineModule::send(T what) -> bool {
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

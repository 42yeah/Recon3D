//
//  Online.cpp
//  Reconing
//
//  Created by apple on 17/04/2021.
//

#include "Online.hpp"
#include <imgui.h>
#include <thread>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

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
            ImGui::TextWrapped("点击 “上传” 上传重建内容到服务端。选中列表中的内容，点击 “下载” 下载到本地重建历史。");
            if (ImGui::Button("上传")) {
                state = State::UPLOAD;
                mkdir_if_not_exists("recons");
                records = read_recon_records("recons/records.bin");
                upload_index = 0;
            }
            if (online_records.records_size() > 0) {
                ImGui::TextWrapped("服务端内容");
            }
            if (online_records.records_size() > 0 &&
                ImGui::BeginListBox("", ImVec2 { -FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing() })) {
                for (auto i = 0; i < online_records.records_size() ; i++) {
                    const auto is_selected = online_index == i;
                    const auto rec = online_records.records(i);
                    if (ImGui::Selectable((rec.name() + " 由 " + rec.owner() + " 创作").c_str(), is_selected)) {
                        online_index = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
                if (ImGui::Button("下载")) {
                    download();
                }
            }
            break;
            
        case State::UPLOAD:
            ImGui::TextWrapped("选择想上传的记录。");
            if (records.size() > 0) {
                ImGui::SameLine();
                if (ImGui::Button("上传")) {
                    upload();
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("返回")) {
                state = State::MAIN_INTERFACE;
            }
            if (records.size() > 0 &&
                ImGui::BeginListBox("", ImVec2 { -FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing() })) {
                for (auto i = 0; i < records.size(); i++) {
                    const auto is_selected = upload_index == i;
                    if (ImGui::Selectable(records[i].name, is_selected)) {
                        upload_index = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndListBox();
            } else if (records.size() <= 0) {
                ImGui::TextWrapped("现在还没有重建记录。在管线内进行重建吧。");
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
            update_online_list();
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

auto OnlineModule::upload() -> void { 
    std::thread thread([&] () {
        mutex().lock();
        RECON_LOG(ONLINE) << "正在上传记录到服务端...";
        state = State::CONNECTING;
        mutex().unlock();
        
        const auto &selected = records[upload_index];
        std::filesystem::path path = std::filesystem::path("recons") / selected.obj_file;
        std::stringstream obj_stream, mtl_stream, tex_stream;
        std::ifstream obj_reader(path), mtl_reader(path.replace_extension(".mtl")), tex_reader(path.replace_extension(".png"));
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
            mutex().lock();
            RECON_LOG(ONLINE) << "无法打开 obj 文件。";
            state = State::UPLOAD;
            mutex().unlock();
            return;
        }
        obj_stream << obj_reader.rdbuf();
        mtl_stream << mtl_reader.rdbuf();
        tex_stream << tex_reader.rdbuf();
        obj_reader.close();
        mtl_reader.close();
        tex_reader.close();
        online::ReconBuffer buffer;
        buffer.set_file_base(path.replace_extension("").filename().string());
        buffer.set_obj_content(obj_stream.str());
        buffer.set_mtl_content(mtl_stream.str());
        buffer.set_texture_content(tex_stream.str());
        if (!send(make_request("upload"))) {
            BAIL("无法发送指令给服务器。");
        }
        if (!send(buffer)) {
            BAIL("无法发送数据给服务器。");
        }
        auto response = receive<online::Request>();
        if (!response.has_value()) {
            BAIL("服务端未发回有效数据。");
        }
        if (response->arg_size() == 1 && response->arg(0) == "success") {
            mutex().lock();
            RECON_LOG(ONLINE) << "上传成功。";
            state = State::MAIN_INTERFACE;
            mutex().unlock();
            update_online_list();
            return;
        }
        mutex().lock();
        RECON_LOG(ONLINE) << "上传失败。";
        state = State::UPLOAD;
        mutex().unlock();
    });
    thread.detach();
}

auto OnlineModule::update_online_list() -> void { 
    std::thread thread([&] () {
        mutex().lock();
        RECON_LOG(ONLINE) << "正在获取服务器重建列表...";
        mutex().unlock();
        
        if (!send(make_request("records"))) {
            BAIL("指令发送失败。");
        }
        auto records_opt = receive<online::ReconRecords>();
        if (!records_opt.has_value()) {
            BAIL("获取重建列表失败。");
        }
        online_records = *records_opt;
    });
    thread.detach();
}

auto OnlineModule::download() -> void {
    std::thread thread([&] () {
        mutex().lock();
        RECON_LOG(ONLINE) << "正在下载选定记录到本地...";
        state = State::CONNECTING;
        mutex().unlock();
        
        const auto &record = online_records.records(online_index);
        if (!send(make_request("download", std::to_string(record.id())))) {
            BAIL("无法发送下载指令到服务器。");
        }
        auto buffer = receive<online::ReconBuffer>();
        if (!buffer.has_value()) {
            BAIL("接受下载信息失败。");
        }
        
        if (buffer->has_file_base() && buffer->has_obj_content() &&
            buffer->has_mtl_content() && buffer->has_texture_content()) {
            mkdir_if_not_exists("recons");
            mkdir_if_not_exists("recons/" + record.owner());
            auto base = std::filesystem::path("recons") / buffer->file_base();
            std::ofstream obj_writer(base.string() + ".obj");
            std::ofstream mtl_writer(base.string() + ".mtl");
            std::ofstream tex_writer(base.string() + ".png");
            if (!obj_writer.good() || !mtl_writer.good() || !tex_writer.good()) {
                if (obj_writer.good()) {
                    obj_writer.close();
                }
                if (mtl_writer.good()) {
                    mtl_writer.close();
                }
                if (tex_writer.good()) {
                    tex_writer.close();
                }
                BAIL("尝试文件写入失败。");
                return;
            }
            obj_writer << buffer->obj_content();
            mtl_writer << buffer->mtl_content();
            tex_writer << buffer->texture_content();
            obj_writer.close();
            mtl_writer.close();
            tex_writer.close();
            
            mutex().lock();
            records = read_recon_records("recons/records.bin");
            auto file_name = base.filename().string();
            auto name = record.owner() + " 创作的 " + base.string();
            auto record_in_db = std::find_if(records.begin(), records.end(), [&] (auto &r) {
                return std::string(r.name) == name;
            });
            if (record_in_db == records.end()) {
                records.push_back(ReconRecord(name,
                                              record.owner() + "/" + file_name + ".obj"));
                write_recon_records(records, "recons/records.bin");
            }
            RECON_LOG(ONLINE) << "文件下载结束。";
            state = State::MAIN_INTERFACE;
            mutex().unlock();
        } else {
            mutex().lock();
            RECON_LOG(ONLINE) << "错误！服务端传输信息未齐全。";
            state = State::MAIN_INTERFACE;
            mutex().unlock();
        }
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

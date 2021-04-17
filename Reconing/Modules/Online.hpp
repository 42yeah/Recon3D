//
//  Online.hpp
//  Reconing
//
//  Created by apple on 17/04/2021.
//

#ifndef Online_hpp
#define Online_hpp

#include <optional>
#include "common.hpp"
#include "Module.hpp"

#define ONLINE "在线功能"
#define ONLINE_REMOTE "127.0.0.1"
#define ONLINE_PORT 17290

namespace OnlineNS {

enum class State {
    WELCOME = 0,
    CONNECTING = 1,
    LOGIN = 2,
    REGISTER = 3,
    MAIN_INTERFACE = 4
};

};


class OnlineModule : public Module {
public:
    OnlineModule() : Module(ONLINE), state(OnlineNS::State::WELCOME), sock(-1) {
        std::memset(username, 0, sizeof(username));
        std::memset(password, 0, sizeof(password));
    }

    virtual auto update(float delta_time) -> bool override;
    
    virtual auto update_ui() -> void override;
    
    auto connect() -> void;
    
    auto login() -> void;
    
    template<typename T>
    auto receive() -> std::optional<T>;
    
    template<typename T>
    auto send(T what) -> bool;
    
private:
    OnlineNS::State state;

    // I M G U I ///////////////////////////////////
    char username[512];
    char password[512];

    // O N L I N E /////////////////////////////////
    int sock;
};

#endif /* Online_hpp */

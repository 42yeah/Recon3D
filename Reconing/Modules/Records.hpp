//
//  Records.hpp
//  Reconing
//
//  Created by apple on 17/04/2021.
//

#ifndef Records_hpp
#define Records_hpp

#include "common.hpp"
#include "Module.hpp"

#define RECORDS "重建记录历史"

class RecordsModule : public Module {
public:
    RecordsModule() : Module(RECORDS), gl_ready(false) {
        records = read_recon_records("recons/records.bin");
        current_selected_index = 0;
    }
    
    virtual auto update(float delta_time) -> bool override;

    virtual auto update_ui() -> void override;
    
    auto load_record() -> bool;
    
private:
    std::vector<ReconRecord> records;
    int current_selected_index;
    
    // O P E N G L /////////////////////////////////////////
    bool gl_ready;
};

#endif /* Records_hpp */

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
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define RECORDS "重建记录历史"

class RecordsModule : public Module {
public:
    RecordsModule() : Module(RECORDS), gl_ready(false),
        mesh_texture(GL_NONE),
        current_selected_index(0),
        horizontal_rotation_target(0.0f) {
        records = read_recon_records("recons/records.bin");
    }
    
    virtual auto update(float delta_time) -> bool override;
    
    virtual auto render() -> void override;

    virtual auto update_ui() -> void override;
    
    auto load_record() -> bool;
    
private:
    auto setup_render(std::vector<Vertex> vertices) -> void;

    std::vector<ReconRecord> records;
    int current_selected_index;
    
    // O P E N G L /////////////////////////////////////////
    bool gl_ready;
    GLuint VAO, VBO, program;
    glm::vec3 eye, center;
    glm::mat4 model_mat, view_mat, perspective_mat;
    float time, radius, horizontal_rotation, horizontal_rotation_target;
    GLuint mesh_texture;
    int num_vertices;
};

#endif /* Records_hpp */

//
//  main.cpp
//  csfm
//
//  Created by apple on 15/03/2021.
//

#include <iostream>
#include <SfM.hpp>
#include <Hoppe.hpp>



int main(int argc, const char * argv[]) {
//    SfM sfm;
//
//    sfm.set_image_dir("assets/");
//    sfm.run_sfm();
//    sfm.export_to_ply("res.ply");
    
    Hoppe hoppe;
    hoppe.load_pointcloud("assets/christmas.obj");
    hoppe.run();
    
    return 0;
}

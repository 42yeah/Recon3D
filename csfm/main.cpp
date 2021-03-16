//
//  main.cpp
//  csfm
//
//  Created by apple on 15/03/2021.
//

#include <iostream>
#include <SfM.hpp>


int main(int argc, const char * argv[]) {
    SfM sfm;
    
    sfm.set_image_dir("assets/");
    sfm.run_sfm();
    sfm.export_to_ply("res.ply");
    
    return 0;
}

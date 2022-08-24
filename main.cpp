#include <vector>
#include <iostream>
#include <limits>

#include "core/tgaimage.h"
#include "core/model.h"
#include "core/geometry.h"
#include "core/our_gl.h"
#include "core/render.h"

const int width  = 800;
const int height = 800;

int main(int argc, char** argv) {
    rasterizer r(width, height);

    ////////////////////////////////////////////////////////////////////////
    // generate the light resourse

    auto l1 = Light{{15,5,-1},{500,500,500}};

    std::vector<Light> l;
    l.push_back(l1);

    ////////////////////////////////////////////////////////////////////////
    
    // render_object("../obj/spot/spot_triangulated_good.obj", r, l);
    // render_object("../obj/gun/Cerberus.obj", r, l);
    // render_object("../obj/helmet/helmet.obj", r, l);

    render_qiyana(r, l);

    return 0;
}

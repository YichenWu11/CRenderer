#include <vector>
#include <iostream>
#include <limits>

#include "core/tgaimage.h"
#include "core/model.h"
#include "core/geometry.h"
#include "core/our_gl.h"
#include "core/render.h"
#include "core/shaders.h"
#include "platform/win32.h"

const int width  = 800;
const int height = 800; 

int main(int argc, char** argv) {
    rasterizer r(width, height);

    window_init(width, height, "CRenderer");

    /* generate the light resourse
        qiyana   {-15,-5,1}
        cow      {-2,1,2}(-90.f) {2,-1,2}(90.f)
        helmet   {-1,1,1}
    */

    auto l1 = Light{{2,-1,2},{500,500,500}};
    // auto l2 = Light{{-1,3,-2},{500,500,500}};

    std::vector<Light> l;
    l.push_back(l1);
    // l.push_back(l2);

    // render_object("../obj/gun/Cerberus.obj", r, l);
    // render_object("../obj/helmet/helmet.obj", r, l);
    // render_cow_with_helmet(r, l);
    // render_qiyana(r, l);

    render_object("../obj/spot/spot_triangulated_good.obj", r, l);

    while (!window->is_close) {
        window_draw(&(r.image));
        msg_dispatch();
    }

    window_destroy();

    return 0;
}

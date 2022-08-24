#include <vector>
#include <iostream>
#include <limits>

#include "core/tgaimage.h"
#include "core/model.h"
#include "core/geometry.h"
#include "core/our_gl.h"

const int width  = 800;
const int height = 800;

Model *model = NULL;

Vec3f eye(2, 1, 3);
Vec3f center(0, 0, 0.5);
Vec3f up(0, 1, 0);

int main(int argc, char** argv) {
    // model = new Model("../obj/spot/spot_triangulated_good.obj");
    // model = new Model("../obj/gun/Cerberus.obj");
    model = new Model("../obj/helmet/helmet.obj");

    rasterizer r(width, height);

    ////////////////////////////////////////////////////////////////////////
    // generate the light resourse

    auto l1 = Light{{1,-1,-1},{500,500,500}};

    std::vector<Light> l;
    l.push_back(l1);

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer

    float *shadowbuffer   = new float[width*height];
    for (int i = width*height; --i; ) shadowbuffer[i] = -std::numeric_limits<float>::max();

    r.do_affine_transform_shadow(-90.f, Vec3f(1,-1,-1).normalize(), eye, center, up);

    DepthShader depthshader(model);
    r.draw(model, depthshader, shadowbuffer);

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering

    r.do_affine_transform(-90.f, eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    // displacement_Shader shader(model, l, mat, mit);
    Blinn_Phong_Shader shader(model, l, shadowbuffer, mat, mit, mshadow); // with shadow mapping
    // dump_Shader shader(model, l, mat, mit);
    // T_Shader shader(model, l, mat, mit);

    r.draw(model, shader);
    // r.draw_wire(model);
    
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // gc

    delete model;
    delete [] shadowbuffer;
    std::cout << "Done" << std::endl;

    ////////////////////////////////////////////////////////////////////////

    return 0;
}

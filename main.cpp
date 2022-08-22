#include <vector>
#include <iostream>
#include <limits>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);

const int width  = 800;
const int height = 800;

Model *model = NULL;

Vec3f eye(2, 1, 3);
Vec3f center(0, 0, 0.5);
Vec3f up(0, 1, 0);

int main(int argc, char** argv) {
    model = new Model("../obj/spot/spot_triangulated_good.obj");

    float *shadowbuffer   = new float[width*height];
    for (int i=width*height; --i; ) shadowbuffer[i] = -std::numeric_limits<float>::max();

    rasterizer r(width, height);

    auto l1 = Light{{1,-1,-1},{500,500,500}};

    std::vector<Light> l;
    l.push_back(l1);

    // rendering the shadow buffer
    r.set_model_matrix(-90.f, 1, Vec3f(0,0,0));
    r.lookat(Vec3f(1,-1,-1).normalize(), center, up);
    r.viewport(width/8, height/8, width*3/4, height*3/4);
    r.projection(0);

    DepthShader depthshader(model);
    r.draw(model, depthshader, shadowbuffer);
    // rendering the shadow buffer

    r.set_model_matrix(-90.f, 1, Vec3f(0,0,0));
    r.lookat(eye, center, up);
    r.projection(-1.f/(eye-center).norm());
    r.viewport(width/8, height/8, width*3/4, height*3/4);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    // displacement_Shader shader(model, l, mat, mit);
    Blinn_Phong_Shader shader(model, l, shadowbuffer, mat, mit, mshadow); // with shadow mapping
    // dump_Shader shader(model, l, mat, mit);
    // T_Shader shader(model, l, mat, mit);

    r.draw(model, shader);

    delete model;
    std::cout << "ok" << std::endl;
    return 0;
}

#ifndef __RENDER_H__
#define __RENDER_H__

#include <limits>
#include <string>

#include "./geometry.h"
#include "our_gl.h"

/*
    qiyana -20.f Vec3f(0.2f, -0.5f, 0.2f)
    helmet -90.f Vec3f(0.3f, 0.3f, 0.f)
    cow    -90.f Vec3f(0.f, 0.f, 0.f)
*/

Vec3f eye(2, 0, 3);
Vec3f center(0, 0, 0.5);
Vec3f up(0, 1, 0);

const Vec3f Blue(150, 210, 255);
const Vec3f White(255, 255, 255);
const Vec3f Yellow(255, 235, 181);
const Vec3f Light_Green(124, 252, 0);

// render ordinary object
void render_object(const char *filename, rasterizer r, std::vector<Light> l) {
    Model *model = new Model(filename);

    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer

    float *shadowbuffer   = new float[r.width*r.height];
    for (int i = r.width*r.height; --i; ) shadowbuffer[i] = -std::numeric_limits<float>::max();

    r.do_affine_transform_shadow(-90.f, l[0].light_dir.normalize(), eye, center, up);

    DepthShader depthshader(model);
    r.draw(model, depthshader, shadowbuffer);

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering

    r.do_affine_transform(-180.f, 0.8f, Vec3f(0.f, 0.f, 0.f), eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    // dump_Shader shader(model, l, mat, mit);
    // T_Shader shader(model, l, mat, mit);
    // displacement_Shader shader(model, l, mat, mit);
    Blinn_Phong_Shader shader(model, l, shadowbuffer, mat, mit, mshadow);
    // Toon_Shader shader(model, l ,mat, mit);

    // render the background color
    r.draw_background(Light_Green, White);

    r.draw(model, shader);
    // r.draw(model, shader2);
    // r.draw_wire(model);
    r.write_tga_file();

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // gc

    delete model;
    delete [] shadowbuffer;
    std::cout << "Done" << std::endl;

    ////////////////////////////////////////////////////////////////////////
}

void render_cow_with_helmet(rasterizer r, std::vector<Light> l) {
    Model *cow = new Model("../obj/spot/spot_triangulated_good.obj");
    Model *helmet = new Model("../obj/helmet/helmet.obj");

    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer

    float *shadowbuffer   = new float[r.width*r.height];
    for (int i = r.width*r.height; --i; ) shadowbuffer[i] = -std::numeric_limits<float>::max();

    r.do_affine_transform_shadow(-90.f, l[0].light_dir.normalize(), eye, center, up);

    DepthShader depthshader(cow);
    r.draw(cow, depthshader, shadowbuffer);

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering

    r.do_affine_transform(-90.f, 1.f, Vec3f(0.2f, 0.f, -0.2f), eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    // dump_Shader shader(model, l, mat, mit);
    // T_Shader shader(model, l, mat, mit);
    // displacement_Shader shader(model, l, mat, mit);
    Blinn_Phong_Shader shader_cow(cow, l, shadowbuffer, mat, mit, mshadow);

    r.draw(cow, shader_cow);

    depthshader.model = helmet;
    r.draw(helmet, depthshader, shadowbuffer);

    Blinn_Phong_Shader shader_helmet(helmet, l, shadowbuffer, mat, mit, mshadow);

    r.do_affine_transform(-100.f, 0.5f, Vec3f(0.7f, 0.57f, 0.f), eye, center, up);
    r.draw(helmet, shader_helmet);

    r.write_tga_file();

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // gc

    delete cow;
    delete helmet;
    delete [] shadowbuffer;
    std::cout << "Done" << std::endl;

    ////////////////////////////////////////////////////////////////////////    
}


void render_qiyana(rasterizer r, std::vector<Light> l) {
    Model *model_body = new Model("../obj/qiyana/qiyanabody.obj");
    Model *model_face = new Model("../obj/qiyana/qiyanaface.obj");
    Model *model_hair = new Model("../obj/qiyana/qiyanahair.obj");

    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer

    float *shadowbuffer   = new float[r.width*r.height];
    for (int i = r.width*r.height; --i; ) shadowbuffer[i] = -std::numeric_limits<float>::max();

    r.do_affine_transform_shadow(-90.f, Vec3f(15,5,-1).normalize(), eye, center, up);

    DepthShader depthshader(model_body);
    r.draw(model_body, depthshader, shadowbuffer);

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering

    r.do_affine_transform(-20.f, 1.f, Vec3f(0.2f, -0.8f, 0.2f), eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    Blinn_Phong_Shader shader_body(model_body, l, shadowbuffer, mat, mit, mshadow); // with shadow mapping
    Blinn_Phong_Shader shader_hair(model_hair, l, shadowbuffer, mat, mit, mshadow); // with shadow mapping
    Blinn_Phong_Shader shader_face(model_face, l, shadowbuffer, mat, mit, mshadow); // with shadow mapping

    r.draw(model_body, shader_body);
    r.draw(model_face, shader_face);
    r.draw(model_hair, shader_hair);
    r.write_tga_file();

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // gc

    delete model_body;
    delete model_face;
    delete model_hair;
    delete [] shadowbuffer;
    std::cout << "Done" << std::endl;

    ////////////////////////////////////////////////////////////////////////    
}




#endif

#ifndef __RENDER_H__
#define __RENDER_H__

#include <limits>
#include <string>
#include <memory>

#include "./geometry.h"
#include "our_gl.h"
#include "../platform/win32.h"

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
const Vec3f Light_Green(124, 150, 0);

// render ordinary object
void render_object(const char *filename, rasterizer &r, std::vector<Light> l) {
    auto model = std::make_shared<Model>(filename);
    auto sky   = std::make_shared<Model>("../obj/skybox2/box.obj", 1);

    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer (discard)

    auto shadowbuffer = std::make_shared<float[]>(r.width*r.height);
    for (int i = r.width * r.height; --i; ) shadowbuffer[i] = -9999.9f;

    r.do_affine_transform_shadow(-90.f, l[0].light_dir.normalize(), eye, center, up);

    DepthShader depthshader(model.get());
    r.draw(model.get(), depthshader, shadowbuffer.get());

    ////////////////////////////////////////////////////////////////////////


    ////////////////////////////////////////////////////////////////////////
    // rendering

    // skybox
    SkyBox_Shader shader_sky(sky.get());
    r.do_affine_transform(38.f, 5.f, Vec3f(-2.4f, -0.14f, -3.f), eye, center, up);
    r.draw(sky.get(), shader_sky);
    // skybox

    r.do_affine_transform(-180.f, 0.8f, Vec3f(-0.2f, -0.35f, 0.f), eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    // dump_Shader shader(model.get(), l, mat, mit);
    // T_Shader shader(model.get(), l, mat, mit);
    // displacement_Shader shader(model.get(), l, mat, mit);
    Blinn_Phong_Shader shader(model.get(), l, shadowbuffer.get(), mat, mit, mshadow);
    // Toon_Shader shader(model.get(), l ,mat, mit);

    // render the background color
    // r.draw_background(Light_Green, White);

    r.draw(model.get(), shader);
    r.do_color_grading();

    // r.draw_wire(model);

    // 不写入文件
    // r.write_tga_file();
}

void render_qiyana(rasterizer &r, std::vector<Light> l) {
    auto model_body = std::make_shared<Model>("../obj/qiyana/qiyanabody.obj");
    auto model_face = std::make_shared<Model>("../obj/qiyana/qiyanaface.obj");
    auto model_hair = std::make_shared<Model>("../obj/qiyana/qiyanahair.obj");

    ////////////////////////////////////////////////////////////////////////
    // rendering the shadow buffer

    auto shadowbuffer = std::make_shared<float[]>(r.width*r.height);
    for (int i = r.width*r.height; --i; ) shadowbuffer[i] = -9999.9f;

    r.do_affine_transform_shadow(-90.f, Vec3f(15,5,-1).normalize(), eye, center, up);

    DepthShader depthshader(model_body.get());
    r.draw(model_body.get(), depthshader, shadowbuffer.get());

    ////////////////////////////////////////////////////////////////////////

    r.do_affine_transform(-20.f, 1.f, Vec3f(0.2f, -0.8f, 0.2f), eye, center, up);

    Matrix M = r.Viewport*r.Projection*r.ModelView*r.Affine;
    Matrix mat   =  r.Projection*r.ModelView*r.Affine;
    Matrix mit = (r.Projection*r.ModelView*r.Affine).invert_transpose();
    Matrix mshadow = M*(r.Viewport*r.Projection*r.ModelView*r.Affine).invert();

    Blinn_Phong_Shader shader_body(model_body.get(), l, shadowbuffer.get(), mat, mit, mshadow); // with shadow mapping
    Blinn_Phong_Shader shader_hair(model_hair.get(), l, shadowbuffer.get(), mat, mit, mshadow); // with shadow mapping
    Blinn_Phong_Shader shader_face(model_face.get(), l, shadowbuffer.get(), mat, mit, mshadow); // with shadow mapping

    r.draw(model_body.get(), shader_body);
    r.draw(model_face.get(), shader_face);
    r.draw(model_hair.get(), shader_hair);
    // r.write_tga_file(); 
}

#endif
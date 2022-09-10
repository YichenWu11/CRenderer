#ifndef __SHADERS_H__
#define __SHADERS_H__

#include <vector>
#include "./tgaimage.h"
#include "./geometry.h"
#include "./model.h"
// #include "./sample.h"

const float depth = 2000.f;

struct Light {
    Vec3f light_dir;
    Vec3f light_intensity;
};

typedef struct cubemap 
{
	TGAImage faces[6];
} cubemap_t;

typedef struct iblmap 
{
	int mip_levels;
	cubemap_t *irradiance_map;
	cubemap_t *prefilter_maps[15];
	TGAImage *brdf_lut;
} iblmap_t;

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert, Matrix m) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};


////////////////////////////////////////////////////////////////////////
// Gouraud Shading

struct GouraudShader : public IShader {
    Model *model;
    std::vector<Light> lights;
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader

    GouraudShader(Model *m, std::vector<Light> l) : model(m), lights(l) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct T_Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    T_Shader(Model *m, std::vector<Light> l, Matrix mat, Matrix mit) : model(m), lights(l), uniform_M(mat), uniform_MIT(mit) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct DepthShader : public IShader {
    mat<3,3,float> varying_tri;
    Model *model;

    DepthShader(Model *m) : varying_tri(), model(m) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
// Blinn_Phong Shading

struct Blinn_Phong_Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    float *shadowbuffer;
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<3,3,float> world_pos;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()
    mat<4,4,float> uniform_Mshadow; // transform framebuffer screen coordinates to shadowbuffer screen coordinates
    mat<3,3,float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS

    Blinn_Phong_Shader(Model *m, std::vector<Light> l, float *s, Matrix mat, Matrix mit, Matrix mshadow) : 
                       model(m), lights(l), shadowbuffer(s), uniform_M(mat), uniform_MIT(mit), uniform_Mshadow(mshadow) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct dump_Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<3,3,float> world_pos;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    dump_Shader(Model *m, std::vector<Light> l, Matrix mat, Matrix mit) : 
                model(m), lights(l), uniform_M(mat), uniform_MIT(mit) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct displacement_Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<3,3,float> world_pos;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    displacement_Shader(Model *m, std::vector<Light> l, Matrix mat, Matrix mit) : 
                        model(m), lights(l), uniform_M(mat), uniform_MIT(mit) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct Toon_Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<3,3,float> world_pos;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()
    mat<3,3,float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS

    Toon_Shader(Model *m, std::vector<Light> l, Matrix mat, Matrix mit) : 
                model(m), lights(l), uniform_M(mat), uniform_MIT(mit) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);

};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct SkyBox_Shader {
    Model *model;
    cubemap_t env;
    Vec4f clip_coord[3];
    Vec3f world_coord[3];
    mat<3,3,float> world_pos;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    SkyBox_Shader() {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m);

    virtual bool fragment(Vec3f bar, TGAColor &color);
};

////////////////////////////////////////////////////////////////////////

#endif
#ifndef __OUR_GL_H__
#define __OUR_GL_H__

#include <vector>
#include "./shaders.h"

const float M_PI = 3.1415926f;

const TGAColor white = TGAColor(255, 255, 255, 255);

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);

// inline Vec3f world2screen(Vec3f v) { return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z); }

class rasterizer {
public:
    TGAImage image;
    TGAImage zbuffer;
    TGAImage depth;
    TGAImage colorgradingmap_;
    int width;
    int height;

    int msaa_w = 2;
    int msaa_h = 2;
    float darkness;

    Matrix Affine;
    Matrix ModelView;
    Matrix Viewport;
    Matrix Projection;

    rasterizer(int w = 800, int h = 800) : 
               image(TGAImage(w, h, TGAImage::RGB)), zbuffer(TGAImage(w, h, TGAImage::RGB)), 
               depth(TGAImage(w, h, TGAImage::RGB)), width(w), height(h) {
                    colorgradingmap_ = TGAImage();
                    colorgradingmap_.read_tga_file("../obj/lut/color_grading_lut_01.tga");
                    darkness = 1.f;
               }

    void viewport(int x, int y, int w, int h);
    void projection(float coeff=0.f); // coeff = -1/c
    void lookat(Vec3f eye, Vec3f center, Vec3f up);
    void set_model_matrix(float angle, float scale_cof = 1, Vec3f trans = Vec3f(0, 0, 0));

    void do_affine_transform(float angle, float scale, Vec3f trans, Vec3f eye, Vec3f center, Vec3f up);
    void do_affine_transform_shadow(float angle, Vec3f light_dir, Vec3f eye, Vec3f center, Vec3f up);

    void draw_background(Vec3f color1, Vec3f color2);
    void draw_wire(Model *model); // 只render出三角线框
    void draw(Model *model, IShader &shader);
    void draw(Model *model, IShader &shader, float *shadowbuffer);

    void do_color_grading();

    void write_tga_file();

    void renderShadow();

    void triangle(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer);
    void triangle_msaa(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer);
    void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *zbuffer);
};

////////////////////////////////////////////////////////////////////////

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P);

float fract(float);

#endif //__OUR_GL_H__


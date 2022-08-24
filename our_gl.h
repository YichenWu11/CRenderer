#ifndef __OUR_GL_H__
#define __OUR_GL_H__

#include <vector>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"

const float depth = 2000.f;
const TGAColor white = TGAColor(255, 255, 255, 255);

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);

// inline Vec3f world2screen(Vec3f v) { return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z); }

struct Light {
    Vec3f light_dir;
    Vec3f light_intensity;
};

struct IShader {
    virtual ~IShader();
    virtual Vec4f vertex(int iface, int nthvert, Matrix m) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

class rasterizer {
public:
    TGAImage image;
    TGAImage zbuffer;
    TGAImage depth;
    int width;
    int height;

    int msaa_w = 2;
    int msaa_h = 2;

    Matrix Affine;
    Matrix ModelView;
    Matrix Viewport;
    Matrix Projection;

    rasterizer(int w, int h) : 
               image(TGAImage(w, h, TGAImage::RGB)), zbuffer(TGAImage(w, h, TGAImage::GRAYSCALE)), width(w), height(h) {}

    void viewport(int x, int y, int w, int h);
    void projection(float coeff=0.f); // coeff = -1/c
    void lookat(Vec3f eye, Vec3f center, Vec3f up);
    void set_model_matrix(float angle, float scale_cof = 1, Vec3f trans = Vec3f(0, 0, 0));

    void do_affine_transform(float angle, Vec3f eye, Vec3f center, Vec3f up);
    void do_affine_transform_shadow(float angle, Vec3f light_dir, Vec3f eye, Vec3f center, Vec3f up);

    void draw_wire(Model *model); // 只render出三角线框
    void draw(Model *model, IShader &shader);
    void draw(Model *model, IShader &shader, float *shadowbuffer);
    void renderShadow();

    void triangle(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer);
    void triangle_msaa(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer);
    void triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *zbuffer);
};

////////////////////////////////////////////////////////////////////////
// Shader only rendering the triangle wire frame

struct WireFrameShader : public IShader {
    Model *model;

    WireFrameShader(Model *m) : model(m) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = m * gl_Vertex;
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        return false;
    }
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
// Gouraud Shading

struct GouraudShader : public IShader {
    Model *model;
    std::vector<Light> lights;
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader

    GouraudShader(Model *m, std::vector<Light> l) : model(m), lights(l) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = m * gl_Vertex;
        for (auto &light : lights) {
            varying_intensity[nthvert] = std::max(0.f,model->normal(iface, nthvert) * light.light_dir); // get diffuse lighting intensity
        }
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        float intensity = varying_intensity*bar;
        if (intensity>.85) intensity = 1;
        else if (intensity>.60) intensity = .80;
        else if (intensity>.45) intensity = .60;
        else if (intensity>.30) intensity = .45;
        else if (intensity>.15) intensity = .30;
        else intensity = 0;
        color = TGAColor(255, 155, 0)*intensity;
        return false;
    }
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct Shader : public IShader {
    Model *model;
    std::vector<Light> lights;
    Vec3f          varying_intensity; // written by vertex shader, read by fragment shader
    mat<2,3,float> varying_uv;        // same as above

    Shader(Model *m, std::vector<Light> l) : model(m), lights(l) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));

        for (auto &light : lights) {
            varying_intensity[nthvert] += std::max(0.f, model->normal(iface, nthvert)*(light.light_dir.normalize())); // get diffuse lighting intensity
        }
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return m*gl_Vertex; // transform it to screen coordinates
    }
    
    virtual bool fragment(Vec3f bar, TGAColor &color) {
        float intensity = varying_intensity*bar;   // interpolate intensity for the current pixel
        Vec2f uv = varying_uv*bar;                 // interpolate uv for the current pixel
        color = model->diffuse(uv)*intensity;      // well duh
        return false;                              // no, we do not discard this pixel
    }
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

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return m*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;
        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();

        for (auto &light : lights) {
            Vec3f l = proj<3>(uniform_M  *embed<4>(light.light_dir)).normalize();
            Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
            float spec = pow(std::max(r.z, 0.0f), 150);
            float diff = std::max(0.f, n*l);
            // float diff = 0.5;
            // TGAColor c = model->diffuse(uv);
            TGAColor c = TGAColor(rand()%255, rand()%255, rand()%255, 255);
            for (int i=0; i<3; i++) color[i] = std::min<float>(5 + c[i]*(diff + .3*spec), 255);
        }
        return false;
    }
};

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

struct DepthShader : public IShader {
    mat<3,3,float> varying_tri;
    Model *model;

    DepthShader(Model *m) : varying_tri(), model(m) {}

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = m*gl_Vertex;          // transform it to screen coordinates
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f p = varying_tri*bar;
        color = TGAColor(255, 255, 255)*(p.z/depth);
        return false;
    }
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

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return m*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec4f sb_p = uniform_Mshadow*embed<4>(varying_tri*bar); // corresponding point in the shadow buffer
        sb_p = sb_p/sb_p[3];
        int idx = int(sb_p[0]) + int(sb_p[1])*800; // index in the shadowbuffer array, 800为image的width
        float shadow = .3+.7*(shadowbuffer[idx]<sb_p[2]); // magic coeff to avoid z-fighting

        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;
        Vec3f point = world_pos*bar;

        int p = 150;

        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();

        Vec3f result_color;

        TGAColor c = model->diffuse(uv);
        // TGAColor c = TGAColor(120, 120, 120, 255);
        color = c;

        // printf("(%d,%d,%d)\n",c[0],c[1],c[2]);

        Vec3f ka = Vec3f(0.005, 0.005, 0.005);
        Vec3f ks = Vec3f(0.7937, 0.7937, 0.7937);        
        Vec3f kd = Vec3f(c[0], c[1], c[2]);

        for (auto &light : lights) {  
            Vec3f l = proj<3>(uniform_M  *embed<4>(light.light_dir)).normalize();

            Vec3f r = (n*(n*l*2.f) - l).normalize();   // the direction of reflected light 

            // float spec = pow(std::max(r.z, 0.0f), p);
            Vec3f specular = kd * pow(std::max(r.z, 0.0f), p);
            // float diff = std::max(0.f, n*l);
            Vec3f diffuse = kd * std::max(0.f, n*l);
            // float ambient = 5;
            Vec3f ambient = cwiseProduct(ka, light.light_intensity);
            // result_color = result_color + ambient + 0.5*specular + diffuse;   
            result_color = result_color + shadow * ambient + shadow * 0.5*specular + diffuse; // shadow mapping   
        }

        for (int i=0; i<3; i++) color[i] = std::min<float>(result_color[i], 255);

        return false;
    }
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

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return m*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;
        Vec3f point = world_pos*bar;

        int p = 150;

        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();

        float kh = 0.2, kn = 0.1;

        float x = n.x;
        float y = n.y;
        float z = n.z;

        Vec3f t = Vec3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
        Vec3f b = cross(normal, t);

        Mat3f TBN = {
            Vec3f(t.x, b.y, n.x),
            Vec3f(t.y, b.y, n.y),
            Vec3f(t.z, b.z, n.z),
        };

        Vec2f uv1 = Vec2f(uv.x + 1.0f / model->diffusemap_.get_width(), uv.y);
        Vec2f uv2 = Vec2f(uv.x, uv.y + 1.0f / model->diffusemap_.get_height());

        TGAColor c = model->diffuse(uv);
        TGAColor c1 = model->diffuse(uv1);
        TGAColor c2 = model->diffuse(uv2);
 
        float dU = kh * kn * (Vec3f(c1[0],c1[1],c1[2]).norm() - Vec3f(c[0],c[1],c[2]).norm());
        float dV = kh * kn * (Vec3f(c2[0],c2[1],c2[2]).norm() - Vec3f(c[0],c[1],c[2]).norm());

        Vec3f ln = Vec3f(-dU, -dV, 1.0f);

        point = point + (kn * n * Vec3f(c[0],c[1],c[2]).norm());
        n = (TBN * ln).normalize();
        Vec3f result_color = n;    
        for (int i=0; i<3; i++) color[i] = std::min<float>(result_color[i]*255.f, 255);
        return false;
    }
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

    virtual Vec4f vertex(int iface, int nthvert, Matrix m) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return m*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;
        Vec3f point = world_pos*bar;

        int p = 150;

        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();

        float kh = 0.2, kn = 0.1;

        float x = n.x;
        float y = n.y;
        float z = n.z;

        Vec3f t = Vec3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
        Vec3f b = cross(normal, t);

        Mat3f TBN = {
            Vec3f(t.x, b.y, n.x),
            Vec3f(t.y, b.y, n.y),
            Vec3f(t.z, b.z, n.z),
        };

        Vec2f uv1 = Vec2f(uv.x + 1.0f / model->diffusemap_.get_width(), uv.y);
        Vec2f uv2 = Vec2f(uv.x, uv.y + 1.0f / model->diffusemap_.get_height());

        TGAColor c = model->diffuse(uv);
        TGAColor c1 = model->diffuse(uv1);
        TGAColor c2 = model->diffuse(uv2);
 
        float dU = kh * kn * (Vec3f(c1[0],c1[1],c1[2]).norm() - Vec3f(c[0],c[1],c[2]).norm());
        float dV = kh * kn * (Vec3f(c2[0],c2[1],c2[2]).norm() - Vec3f(c[0],c[1],c[2]).norm());

        Vec3f ln = Vec3f(-dU, -dV, 1.0f);

        point = point + (kn * n * Vec3f(c[0],c[1],c[2]).norm());
        n = (TBN * ln).normalize();
        Vec3f result_color;

        for (auto &light : lights) {
            Vec3f l = proj<3>(uniform_M  *embed<4>(light.light_dir)).normalize();
            Vec3f r = (n*(n*l*2.f) - l).normalize();   // the direction of reflected light 
            Vec3f viewDir = (-point).normalize();
            Vec3f halfVector = (l + viewDir) / 2.0f;

            Vec3f ka = Vec3f(0.005, 0.005, 0.005);
            Vec3f ks = Vec3f(0.7937, 0.7937, 0.7937);        
            // Vec3f kd = Vec3f(c[0], c[1], c[2]);
            Vec3f kd = Vec3f(230, 230, 230);

            Vec3f specular = kd * pow(std::max(r.z, 0.0f), p);

            Vec3f diffuse = kd * std::max(0.0f, n.normalize()*l);

            Vec3f ambient = cwiseProduct(ka, light.light_intensity);

            result_color = result_color + ambient + 2.0*specular + diffuse;
        }
        for (int i=0; i<3; i++) color[i] = std::min<float>(result_color[i], 255);

        return false;
    }
};

////////////////////////////////////////////////////////////////////////

Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P);

#endif //__OUR_GL_H__


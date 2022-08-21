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
const int depth  = 255;

Model *model = NULL;
float *zbuffer = NULL;

Vec3f light_dir = Vec3f(1, -1, -1); // define light_dir
Vec3f eye(10, 3, 10);
Vec3f center(0, 0, 0);
Vec3f up(0,1,0);

float angle = -90.f * M_PI / 180.f;

Matrix rotation = {
    Vec4f(cos(angle), 0, sin(angle), 0), 
    Vec4f(0, 1, 0, 0), 
    Vec4f(-sin(angle), 0, cos(angle), 0),  
    Vec4f(0, 0, 0, 1),
};

struct GouraudShader : public IShader {
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader

    virtual Vec4f vertex(int iface, int nthvert) {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;
        varying_intensity[nthvert] = std::max(0.f,model->normal(iface, nthvert) * light_dir); // get diffuse lighting intensity
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

struct Shader : public IShader {
    Vec3f          varying_intensity; // written by vertex shader, read by fragment shader
    mat<2,3,float> varying_uv;        // same as above

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        // std::cout << model->normal(iface, nthvert).x << ","
        //           << model->normal(iface, nthvert).y << ","
        //           << model->normal(iface, nthvert).z << ","
        //           << std::endl;
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert)*(light_dir.normalize())); // get diffuse lighting intensity
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return Viewport*Projection*ModelView*rotation*gl_Vertex; // transform it to screen coordinates
    }
    
    virtual bool fragment(Vec3f bar, TGAColor &color) {
        float intensity = varying_intensity*bar;   // interpolate intensity for the current pixel
        Vec2f uv = varying_uv*bar;                 // interpolate uv for the current pixel
        color = model->diffuse(uv)*intensity;      // well duh
        return false;                              // no, we do not discard this pixel
    }
};

struct T_Shader : public IShader {
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return Viewport*Projection*ModelView*rotation*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;
        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();
        Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir        )).normalize();
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
        float spec = pow(std::max(r.z, 0.0f), 250);
        float diff = std::max(0.f, n*l);
        // float diff = 0.5;
        TGAColor c = model->diffuse(uv);
        // TGAColor c = TGAColor(120, 120, 120, 255);
        color = c;
        for (int i=0; i<3; i++) color[i] = std::min<float>(5 + c[i]*(diff + .3*spec), 255);
        return false;
    }
};

// struct ToonShader : public IShader {
//     mat<3,3,float> varying_tri;
//     Vec3f          varying_ity;

//     virtual ~ToonShader() {}

//     virtual Vec3i vertex(int iface, int nthvert) {
//         Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));
//         gl_Vertex = Projection*ModelView*gl_Vertex;
//         varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));

//         varying_ity[nthvert] = CLAMP(model->normal(iface, nthvert)*light_dir, 0.f, 1.f);

//         gl_Vertex = Viewport*gl_Vertex;
//         return proj<3>(gl_Vertex/gl_Vertex[3]);
//     }

//     virtual bool fragment(Vec3f bar, TGAColor &color) {
//         float intensity = varying_ity*bar;
//         if (intensity>.85) intensity = 1;
//         else if (intensity>.60) intensity = .80;
//         else if (intensity>.45) intensity = .60;
//         else if (intensity>.30) intensity = .45;
//         else if (intensity>.15) intensity = .30;
//         color = TGAColor(255, 155, 0)*intensity;
//         return false;
//     }
// };

struct Blinn_Phong_Shader : public IShader {
    mat<2,3,float> varying_uv;  // same as above
    mat<3,3,float> varying_normal; // 法线
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_normal.set_col(nthvert, model->normal(iface, nthvert));
        // point = embed<3>(model->vert(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return Viewport*Projection*ModelView*rotation*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f normal = varying_normal*bar;
        Vec2f uv = varying_uv*bar;

        Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();
        Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir        )).normalize();

        Vec3f r = (n*(n*l*2.f) - l).normalize();   // the direction of reflected light 
        float spec = pow(std::max(r.z, 0.0f), 250);
        float diff = std::max(0.f, n*l);
        // float diff = 0.5;
        TGAColor c = model->diffuse(uv);
        // TGAColor c = TGAColor(120, 120, 120, 255);
        color = c;
        for (int i=0; i<3; i++) color[i] = std::min<float>(5 + c[i]*(diff + .3*spec), 255);
        return false;
    }
};


void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1) < std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    
    int dx = x1 - x0;
    int dy = y1 - y0;
    int d_error = 2 * std::abs(dy);
    int error = 0;
    int y = y0;
    for (int x = x0; x <= x1; ++x) {
        if (steep) image.set(y, x, color); 
        else image.set(x, y, color);  
        error += d_error; 
        if (error > dx) { 
            y += (y1 > y0? 1 : -1); 
            error -= dx * 2; 
        } 
    } 
}



/*
初级版本
*/
Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

// void triangle(Vec3f *pts, Vec2f *texts, float *zbuffer, TGAImage &image) {
//     Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
//     Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
//     Vec2f clamp(image.get_width()-1, image.get_height()-1);
//     for (int i=0; i<3; i++) {
//         for (int j=0; j<2; j++) {
//             bboxmin[j] = std::max(0.f,      std::min(bboxmin[j], pts[i][j]));
//             bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
//         }
//     }
//     Vec3f P;
//     for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
//         for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
//             Vec3f bc_screen  = barycentric(pts[0], pts[1], pts[2], P);
//             if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
//             P.z = 0;
//             Vec2f Ptext(0, 0);
//             for (int i=0; i<3; i++) {
//                 P.z += pts[i][2] * bc_screen[i];
//                 Ptext[0] += texts[i][0] * bc_screen[i];
// 				Ptext[1] += texts[i][1] * bc_screen[i];
//             }
//             if (zbuffer[int(P.x+P.y*width)]<P.z) {
//                 TGAColor color = model->diffuse(Ptext);
//                 zbuffer[int(P.x+P.y*width)] = P.z;
//                 image.set(P.x, P.y, color);
//             }
//         }
//     }
// } 

// void triangle(Vec3i *t, float *intensity, float *zbuffer, TGAImage &image) {
//     if (t[0].y==t[1].y && t[0].y==t[2].y) return; // i dont care about degenerate triangles
//     if (t[0].y>t[1].y) { std::swap(t[0], t[1]); std::swap(intensity[0], intensity[1]); }
//     if (t[0].y>t[2].y) { std::swap(t[0], t[2]); std::swap(intensity[0], intensity[2]); }
//     if (t[1].y>t[2].y) { std::swap(t[1], t[2]); std::swap(intensity[1], intensity[2]); }

//     int total_height = t[2].y-t[0].y;
//     for (int i=0; i<total_height; i++) {
//         bool second_half = i>t[1].y-t[0].y || t[1].y==t[0].y;
//         int segment_height = second_half ? t[2].y-t[1].y : t[1].y-t[0].y;
//         float alpha = (float)i/total_height;
//         float beta  = (float)(i-(second_half ? t[1].y-t[0].y : 0))/segment_height; // be careful: with above conditions no division by zero here
//         Vec3i A    =               t[0]  + Vec3f(t[2]-t[0]  )*alpha;
//         Vec3i B    = second_half ? t[1]  + Vec3f(t[2]-t[1]  )*beta : t[0]  + Vec3f(t[1]-t[0]  )*beta;
//         float ityA =               intensity[0] +   (intensity[2]-intensity[0])*alpha;
//         float ityB = second_half ? intensity[1] +   (intensity[2]-intensity[1])*beta : intensity[0] +   (intensity[1]-intensity[0])*beta;
//         if (A.x>B.x) { std::swap(A, B); std::swap(ityA, ityB); }
//         for (int j=A.x; j<=B.x; j++) {
//             float phi = B.x==A.x ? 1. : (float)(j-A.x)/(B.x-A.x);
//             Vec3i    P = Vec3f(A) +  Vec3f(B-A)*phi;
//             float ityP =    ityA  + (ityB-ityA)*phi;
//             int idx = P.x+P.y*width;
//             if (P.x>=width||P.y>=height||P.x<0||P.y<0) continue;
//             if (zbuffer[idx]<P.z) {
//                 zbuffer[idx] = P.z;
//                 image.set(P.x, P.y, TGAColor(255, 255, 255)*ityP);
//             }
//         }
//     }
// }
/*
初级版本
*/

Vec3f world2screen(Vec3f v) {
    // v = v / (1 - 0.2 * v.z);
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

Vec4f toVector4(Vec3f v) {
    return Vec4f(v.x, v.y, v.z, 1.f);
}

Matrix vec2matrix(Vec3f v) {
    Matrix ret;
    ret[0][0] = v.x;
    ret[1][0] = v.y;
    ret[2][0] = v.z;
    return ret;
}

Vec3f matrix2vec(Matrix m) {
    return Vec3f(m[0][0]/m[3][0], m[1][0]/m[3][0], m[2][0]/m[3][0]);
}

int main(int argc, char** argv) {
    model = new Model("../obj/spot/spot_triangulated_good.obj");
    // model = new Model("../obj/fox/low-poly-fox-by-pixelmannen.obj");

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    // zbuffer = new float[width*height];
    // for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    lookat(eye, center, up);
    projection(-1.f/(eye-center).norm());
    viewport(width/8, height/8, width*3/4, height*3/4);

    // std::cerr << ModelView << std::endl;
    // std::cerr << Projection << std::endl;
    // std::cerr << Viewport << std::endl;
    Matrix mvp = (Viewport*Projection*ModelView);
    std::cerr << mvp << std::endl;


    // TODO: 添加纹理
    /*
    for (int i = 0; i < model->nfaces(); ++i) {
        std::vector<int> face = model->face(i);
        Vec3i screen_coords[3]; 
        Vec3f world_coords[3]; 
        float intensity[3];

        // std::cout << 1111 << std::endl;

        for (int j = 0; j < 3; ++j) { 
            Vec3f v = model->vert(face[j]); 
            Vec3f rotated_v = rotation * model->vert(face[j]); 
            // world_coords[j]  = rotated_v; 
            Vec4f trans = mvp*(toVector4(v));
            screen_coords[j] = Vec3f(trans.x, trans.y, trans.z);

            intensity[j] = model->normal(i, j) * light_dir;
            // std::cout << screen_coords[j] << std::endl;
        } 
        // Vec3f n = cross(world_coords[2]-world_coords[0], world_coords[1]-world_coords[0]); 
        // n.normalize(); 
        // float intensity = n * light_dir; 
        // if (intensity>0) triangle(pts, zbuffer, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        triangle(screen_coords, intensity, zbuffer, image);

        // 随机颜色（五彩小牛）
        // Vec2i screen_coords[3]; 
        // for (int j = 0; j < 3; ++j) {
        //     Vec3f world_coords = rotation * model->vert(face[j]);
        //     screen_coords[j] = Vec2i((world_coords.x+1.)*width/2., (world_coords.y+1.)*height/2.);
        // }
        // triangle(screen_coords, image, TGAColor(rand()%255, rand()%255, rand()%255, 255));
    }
    */

    T_Shader shader;
    shader.uniform_M   =  Projection*ModelView;
    shader.uniform_MIT = (Projection*ModelView).invert_transpose();

    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.flip_vertically();
    zbuffer.write_tga_file("zbuffer.tga");


    // TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
    // for (int i=0; i<width; i++) {
    //     for (int j=0; j<height; j++) {
    //         zbimage.set(i, j, TGAColor(zbuffer[i+j*width]));
    //     }
    // }
    // zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    // zbimage.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}

#include <cmath>
#include <limits>
#include <cstdlib>
#include "./our_gl.h"

IShader::~IShader() {}

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

///////////////////////////////////////////////////////

void rasterizer::set_model_matrix(float angle, float scale_cof, Vec3f trans) {
    angle = angle * M_PI / 180.f;
    Matrix rotation = {
        Vec4f(cos(angle), 0, sin(angle), 0), 
        Vec4f(0, 1, 0, 0), 
        Vec4f(-sin(angle), 0, cos(angle), 0),  
        Vec4f(0, 0, 0, 1),
    };
    Matrix translate = {
        Vec4f(1, 0, 0, trans.x), 
        Vec4f(0, 1, 0, trans.y), 
        Vec4f(0, 0, 1, trans.z),  
        Vec4f(0, 0, 0, 1),
    };
    Matrix scale = {
        Vec4f(scale_cof, 0, 0, 0), 
        Vec4f(0, scale_cof, 0, 0), 
        Vec4f(0, 0, scale_cof, 0),  
        Vec4f(0, 0, 0, 1),
    };

    Affine = translate * rotation * scale;
}

void rasterizer::viewport(int x, int y, int w, int h) {
    Viewport = Matrix::identity();
    Viewport[0][3] = x+w/2.f;
    Viewport[1][3] = y+h/2.f;
    Viewport[2][3] = 255.f/2.f;
    Viewport[0][0] = w/2.f;
    Viewport[1][1] = h/2.f;
    Viewport[2][2] = 255.f/2.f;
}

void rasterizer::projection(float coeff) {
    Projection = Matrix::identity();
    Projection[3][2] = coeff;
}

void rasterizer::lookat(Vec3f eye, Vec3f center, Vec3f up) {
    Vec3f z = (eye-center).normalize();
    Vec3f x = cross(up,z).normalize();
    Vec3f y = cross(z,x).normalize();
    ModelView = Matrix::identity();
    for (int i=0; i<3; i++) {
        ModelView[0][i] = x[i];
        ModelView[1][i] = y[i];
        ModelView[2][i] = z[i];
        ModelView[i][3] = -center[i];
    }
}

void rasterizer::do_affine_transform(float angle, Vec3f eye, Vec3f center, Vec3f up) {
    set_model_matrix(angle, 1.f, Vec3f(0.3f, 0.3f, 0));
    lookat(eye, center, up);
    projection(-1.f/(eye-center).norm());
    viewport(width/8, height/8, width*3/4, height*3/4);
}

void rasterizer::do_affine_transform_shadow(float angle, Vec3f light_dir, Vec3f eye, Vec3f center, Vec3f up) {
    set_model_matrix(angle);
    lookat(light_dir, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection(0);
}

void rasterizer::draw_wire(Model *model) {
    float angle = -40.f * M_PI / 180.f;
    Mat3f rotation = {
        Vec3f(cos(angle), 0, sin(angle)), 
        Vec3f(0, 1, 0), 
        Vec3f(-sin(angle), 0, cos(angle)),  
    };

    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        for (int j=0; j<3; j++) {
            Vec3f v0 = rotation * model->vert(face[j]);
            Vec3f v1 = rotation * model->vert(face[(j+1)%3]);
            int x0 = (v0.x+1.)*width/2.;
            int y0 = (v0.y+1.)*height/2.;
            int x1 = (v1.x+1.)*width/2.;
            int y1 = (v1.y+1.)*height/2.;
            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.flip_vertically();
    zbuffer.write_tga_file("zbuffer.tga");
}

// actual rendering
void rasterizer::draw(Model *model, IShader &shader) {
    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j, Viewport*Projection*ModelView*Affine);
        }
        // std::cout << screen_coords[0] << "," << screen_coords[1] << "," << screen_coords[2] << std::endl;
        triangle_msaa(screen_coords, shader, image, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.flip_vertically();
    zbuffer.write_tga_file("zbuffer.tga");
}

// render the shadow_mapping
void rasterizer::draw(Model *model, IShader &shader, float *shadowbuffer) {
    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j, Viewport*Projection*ModelView);
        }
        triangle(screen_coords, shader, depth, shadowbuffer);
    }

    depth.flip_vertically(); // to place the origin in the bottom left corner of the image
    depth.write_tga_file("depth.tga");
}

// void rasterizer::renderShadow() {

// }

void rasterizer::triangle(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer) {
    // axis-aligned bounding box (AABB)
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
            if (c.x<0 || c.y<0 || c.z<0 || zbuffer.get(P.x, P.y)[0]>frag_depth) continue;
            bool discard = shader.fragment(c, color);
            if (!discard) {
                zbuffer.set(P.x, P.y, TGAColor(frag_depth));
                image.set(P.x, P.y, color);
            }

            // std::cout << pts[0] << std::endl;
        }
    }
}


void rasterizer::triangle_msaa(Vec4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer) {
    // axis-aligned bounding box (AABB)
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            float count = 0.f;
            float step = 1.f / (msaa_h + msaa_w);
            Vec2f cur_pos = Vec2f(P.x, P.y);
            for (int i = 1; i <= msaa_w; ++i) {
                for (int j = 1; j <= msaa_h; ++j) {
                    Vec2f pos = Vec2f(cur_pos.x + i * step, cur_pos.y + i * step);
                    Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), pos);
                    if (c.x>=0 || c.y>=0 || c.z>=0) count += step;
                }
            }
            Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
            if (c.x<0 || c.y<0 || c.z<0 || zbuffer.get(P.x, P.y)[0]>frag_depth) continue;
            bool discard = shader.fragment(c, color);
            if (!discard) {
                // printf("(%d,%d,%d,%d)\n",color[0],color[1],color[2],color[3]);
                TGAColor act_color = color * count;
                zbuffer.set(P.x, P.y, TGAColor(frag_depth));
                image.set(P.x, P.y, act_color);
            }
        }
    }
}


// render the shadow_mapping
void rasterizer::triangle(Vec4f *pts, IShader &shader, TGAImage &image, float *shadowbuffer) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
        }
    }
    Vec2i P;
    TGAColor color;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
            float z = pts[0][2]*c.x + pts[1][2]*c.y + pts[2][2]*c.z;
            float w = pts[0][3]*c.x + pts[1][3]*c.y + pts[2][3]*c.z;
            int frag_depth = z/w;
            if (c.x<0 || c.y<0 || c.z<0 || shadowbuffer[P.x+P.y*image.get_width()]>frag_depth) continue;
            bool discard = shader.fragment(c, color);
            if (!discard) {
                shadowbuffer[P.x+P.y*image.get_width()] = frag_depth;
                image.set(P.x, P.y, color);
            }
        }
    }
}

///////////////////////////////////////////////////////


// return (alpha, beta, gamma)
Vec3f barycentric(Vec2f A, Vec2f B, Vec2f C, Vec2f P) {
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

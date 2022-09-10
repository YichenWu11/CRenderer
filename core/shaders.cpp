#include "./shaders.h"

IShader::~IShader() {}

Vec4f GouraudShader::vertex(int iface, int nthvert, Matrix m) {
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    gl_Vertex = m * gl_Vertex;
    for (auto &light : lights) {
        varying_intensity[nthvert] = std::max(0.f,model->normal(iface, nthvert) * light.light_dir); // get diffuse lighting intensity
    }
    return gl_Vertex;    
}

bool GouraudShader::fragment(Vec3f bar, TGAColor &color) {
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

Vec4f T_Shader::vertex(int iface, int nthvert, Matrix m) {
    varying_uv.set_col(nthvert, model->uv(iface, nthvert));
    varying_normal.set_col(nthvert, model->normal(iface, nthvert));
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    return m*gl_Vertex; // transform it to screen coordinates
}

bool T_Shader::fragment(Vec3f bar, TGAColor &color) {
    Vec3f normal = varying_normal*bar;
    Vec2f uv = varying_uv*bar;
    Vec3f n = proj<3>(uniform_MIT*embed<4>(normal)).normalize();

    for (auto &light : lights) {
        Vec3f l = proj<3>(uniform_M  *embed<4>(-light.light_dir)).normalize();
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

////////////////////////////////////////////////////////////////////////

Vec4f DepthShader::vertex(int iface, int nthvert, Matrix m) {
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    gl_Vertex = m*gl_Vertex;          // transform it to screen coordinates
    varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
    return gl_Vertex;    
}

bool DepthShader::fragment(Vec3f bar, TGAColor &color) {
    Vec3f p = varying_tri*bar;
    color = TGAColor(255, 255, 255)*(p.z/depth);
    return false;
}


////////////////////////////////////////////////////////////////////////
// Blinn_Phong_Shader

Vec4f Blinn_Phong_Shader::vertex(int iface, int nthvert, Matrix m) {
    varying_uv.set_col(nthvert, model->uv(iface, nthvert));
    varying_normal.set_col(nthvert, model->normal(iface, nthvert));
    world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
    return m*gl_Vertex; // transform it to screen coordinates
}

bool Blinn_Phong_Shader::fragment(Vec3f bar, TGAColor &color) {
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
        Vec3f l = proj<3>(uniform_M * embed<4>(point-light.light_dir)).normalize();

        Vec3f r = (n * (n * l * 2.f) - l).normalize();   // the direction of reflected light 

        // float spec = pow(std::max(r.z, 0.0f), p);
        Vec3f specular = kd * pow(std::max(r.z, 0.0f), p);
        // float diff = std::max(0.f, n*l);
        Vec3f diffuse = kd * std::max(0.f, n * l);
        // float ambient = 5;
        Vec3f ambient = cwiseProduct(ka, light.light_intensity);
        // result_color = result_color + ambient + 0.5*specular + diffuse;   
        result_color = result_color + shadow * ambient + shadow * 0.5 * specular + diffuse; // shadow mapping   
    }

    for (int i = 0; i < 3; ++i) color[i] = std::min<float>(result_color[i], 255);

    return false;
}

////////////////////////////////////////////////////////////////////////

Vec4f dump_Shader::vertex(int iface, int nthvert, Matrix m) {
    varying_uv.set_col(nthvert, model->uv(iface, nthvert));
    varying_normal.set_col(nthvert, model->normal(iface, nthvert));
    world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    return m*gl_Vertex; // transform it to screen coordinates
}

bool dump_Shader::fragment(Vec3f bar, TGAColor &color) {
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

////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////

Vec4f displacement_Shader::vertex(int iface, int nthvert, Matrix m) {
    varying_uv.set_col(nthvert, model->uv(iface, nthvert));
    varying_normal.set_col(nthvert, model->normal(iface, nthvert));
    world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    return m*gl_Vertex; // transform it to screen coordinates
}

bool displacement_Shader::fragment(Vec3f bar, TGAColor &color) {
    Vec3f normal = varying_normal*bar;
    Vec2f uv = varying_uv*bar;
    Vec3f point = world_pos*bar;

    int p = 80;

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
        Vec3f l = proj<3>(uniform_M  *embed<4>(-light.light_dir)).normalize();
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // the direction of reflected light 
        Vec3f viewDir = (-point).normalize();
        Vec3f halfVector = (l + viewDir) / 2.0f;

        Vec3f ka = Vec3f(0.005, 0.005, 0.005);
        Vec3f ks = Vec3f(0.7937, 0.7937, 0.7937);        
        // Vec3f kd = Vec3f(c[0], c[1], c[2]);
        Vec3f kd = Vec3f(148, 120, 92);

        Vec3f specular = kd * pow(std::max(r.z, 0.0f), p);

        Vec3f diffuse = kd * std::max(0.0f, n.normalize()*l);

        Vec3f ambient = cwiseProduct(ka, light.light_intensity);

        result_color = result_color + ambient + 2.0*specular + diffuse;
    }
    for (int i=0; i<3; i++) color[i] = std::min<float>(result_color[i], 255);

    return false;
}


////////////////////////////////////////////////////////////////////////
// Toon_Shader

Vec4f Toon_Shader::vertex(int iface, int nthvert, Matrix m) {
    varying_uv.set_col(nthvert, model->uv(iface, nthvert));
    varying_normal.set_col(nthvert, model->normal(iface, nthvert));
    world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
    Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
    varying_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
    return m*gl_Vertex; // transform it to screen coordinates
}

bool Toon_Shader::fragment(Vec3f bar, TGAColor &color) {
    Vec3f normal = varying_normal*bar;
    Vec2f uv = varying_uv*bar;
    Vec3f point = world_pos*bar;

    TGAColor c = model->diffuse(uv);
    Vec3f cs = Vec3f(c[0], c[1], c[2]);     // color_surface
    Vec3f cw = Vec3f(0.3*255, 0.3*255, 0);  // color_warm
    Vec3f cc = Vec3f(0, 0, 0.55*255);       // color_cool
    Vec3f ch = Vec3f(255, 255, 255);        // color_highlight

    Vec3f result_color;

    for (auto &light : lights) {
        Vec3f l = proj<3>(uniform_M  *embed<4>(-light.light_dir)).normalize();
        Vec3f viewDir = (-point).normalize();
        float t = (normal*l + 1) / 2;
        Vec3f r = 2 * (normal * l) * normal - l;
        // float s = 100*(r*viewDir) - 97;
        float s = 0.5;

        result_color = result_color + s*ch + (1-s)*(t*cw + (1-t)*cc);
    }

    for (int i=0; i<3; i++) color[i] = std::min<float>(result_color[i], 255);

    return false;
}

////////////////////////////////////////////////////////////////////////

// Vec4f SkyBox_Shader::vertex(int iface, int nthvert, Matrix m) {
//     world_coord[nthvert] = model->vert(iface, nthvert);
//     world_pos.set_col(nthvert,embed<3>(model->vert(iface, nthvert)));
//     Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
//     clip_coord[nthvert] = m*gl_Vertex;
//     return m*gl_Vertex; // transform it to screen coordinates
// }

// bool SkyBox_Shader::fragment(Vec3f bar, TGAColor &color) {
//     Vec3f point = world_pos*bar;

//     float Z = 1.0 / (bar[0]/clip_coord[0].w + bar[1]/clip_coord[1].w + bar[2]/clip_coord[2].w);
//     Vec3f w_pos = (bar[0]*world_coord[0]/clip_coord[0].w + bar[1]*world_coord[1]/clip_coord[1].w +
//         bar[2]*world_coord[2]/clip_coord[2].w) * Z;

//     // Vec3f result_color = cubemap_sampling(w_pos, model->environment_map);

//     for (int i = 0; i < 3; ++i) color[i] = 0;
// }

////////////////////////////////////////////////////////////////////////

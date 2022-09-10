#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "./geometry.h"
#include "./tgaimage.h"
#include "./sample.h"

class Model {
private:
    std::vector<Vec3f> verts_;
    std::vector<std::vector<Vec3i> > faces_; // attention, this Vec3i means vertex/uv/normal
    std::vector<Vec3f> norms_;
    std::vector<Vec2f> uv_;
    void load_texture(std::string filename, const char *suffix, TGAImage &img);
    void load_cubemap(const char *filename);

public:
    //skybox
	cubemap_t *environment_map;
	int is_skybox;

    TGAImage diffusemap_;
    TGAImage normalmap_;
    TGAImage specularmap_;
    TGAImage roughnessmap_;
    TGAImage metalnessmap_;
    TGAImage emissionmap_;
    Model(const char *filename, int is_sky = 0);
    ~Model();
    int nverts();
    int nfaces();
    Vec3f normal(int iface, int nthvert);
    Vec3f normal(Vec2f uv);
    Vec3f vert(int i);
    Vec3f vert(int iface, int nthvert);
    Vec2f uv(int iface, int nthvert);
    TGAColor diffuse(Vec2f uv);
    float specular(Vec2f uv);
    float metalness(Vec2f uv);
    float roughness(Vec2f uv);
    std::vector<int> face(int idx);

    // get_color
    Vec3f get_color(float u, float v);
};
#endif //__MODEL_H__


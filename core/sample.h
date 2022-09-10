#ifndef __SAMPLE_H__
#define __SAMPLE_H__

#include <cmath>

#include "./geometry.h"
#include "./tgaimage.h"

typedef struct cubemap 
{
	TGAImage faces[6];
} cubemap_t;

Vec3f cubemap_sampling(Vec3f direction, cubemap_t *cubemap);

Vec3f texture_sample(Vec2f uv, TGAImage *image);

#endif
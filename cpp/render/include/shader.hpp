#ifndef SHADER_H_
#define SHADER_H_

#include "geometry/vector.hpp"
#include "geometry/matrix.hpp"
#include "model.h"
#include "tga.h"

using namespace geometry;

void viewport(const int x, const int y, const int w, const int h);
void projection(const double coef=0);
void lookat(const Vector3d &eye, const Vector3d &center, const Vector3d &up);

struct IShader{
    static TGAColor sample2d(const TGAImage &img, const Vector2d &uv)
    {
        return img.getColor(uv[0] * img.width(), uv[1] * img.height());
    }
    virtual bool fragment(const Vector3d &bar, TGAColor &color) = 0;
}; // struct IShader

void triangle(const VectorNd<double, 4> clip_verts[3], IShader &shader, TGAImage &img, std::vector<double> &zbuffer);

struct Shader : public IShader
{
    const Model &model;
    Vector3d uniformLight;
    MatrixNd<2, 3> varyingUV;
    MatrixNd<3, 3> varyingNormal;
    MatrixNd<3, 3> viewTriangle;

    Shader(const Model &m) : model(m)
    {
        
    }
}; // struct Shader

#endif // SHADER_H_
#ifndef MODEL_H_
#define MODEL_H_

#include <vector>
#include <string>

#include "geometry/vector.hpp"
#include "tga.h"

using namespace geometry;

class Model
{
private:
    std::vector<Vector3d> vertices_;
    std::vector<Vector2d> textures_;
    std::vector<Vector3d> normals_;
    // per-triangle indices in the above arrays
    std::vector<int> vertex_indices_;
    std::vector<int> texture_indices_;
    std::vector<int> normal_indices_;
    TGAImage diffuseMap_;
    TGAImage normalMap_;
    TGAImage specularMap_;
    void loadTexture_(const std::string filename, const std::string suffix, TGAImage &img);

public:
    Model(const std::string filename);

    int numVertices() const;
    int numFaces() const;
    Vector3d getNormal(const int faceIdx, const int vertexIdx) const;
    Vector3d getNormal(const Vector2d &uv) const;
    Vector3d getVertex(const int idx) const;
    Vector3d getVertex(const int faceIdx, const int vertexIdx) const;
    Vector2d getUV(const int faceIdx, const int vertexIdx) const;
    const TGAImage& diffuse() const { return diffuseMap_; }
    const TGAImage& specular() const { return specularMap_; }
}; // class Model
#endif // MODEL_H_
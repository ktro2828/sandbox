#include <iostream>
#include <sstream>

#include "model.h"

Model::Model(const std::string filename)
{
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail())
        return;
    std::string line;
    while (!in.eof())
    {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v "))
        {
            iss >> trash;
            Vector3d v;
            for (int i = 0; i < 3; ++i)
                iss >> v[i];
            vertices_.push_back(v);
        }
        else if (!line.compare(0, 3, "vn "))
        {
            iss >> trash >> trash;
            Vector3d n;
            for (int i = 0; i < 3; ++i)
                iss >> n[i];
            normals_.push_back(n);
        }
        else if (!line.compare(0, 3, "vt "))
        {
            iss >> trash >> trash;
            Vector2d uv;
            for (int i = 0; i < 2; ++i)
                iss >> uv[i];
            textures_.push_back({uv[0], 1 - uv[1]});
        }
        else if (!line.compare(0, 2, "f "))
        {
            int f, t, n;
            iss >> trash;
            int cnt = 0;
            while (iss >> f >> trash >> t >> trash >> n)
            {
                vertex_indices_.push_back(--f);
                texture_indices_.push_back(--t);
                normal_indices_.push_back(--n);
                cnt++;
            }
            if (3 != cnt)
            {
                std::cerr << "Error: the obj file is supposed to be triangulated" << std::endl;
                in.close();
                return;
            }
        }
    }
    in.close();
    std::cerr << "# v# " << numVertices()
              << " f# " << numFaces()
              << " vt# " << textures_.size() << std::endl;
    loadTexture_(filename, "_diffuse.tga", diffuseMap_);
    loadTexture_(filename, "_nm_tangent.tga", normalMap_);
    loadTexture_(filename, "_spec.tga", specularMap_);
}

int Model::numVertices() const
{
    return vertices_.size();
}

int Model::numFaces() const
{
    return vertex_indices_.size();
}

Vector3d Model::getVertex(const int idx) const
{
    return vertices_[idx];
}

Vector3d Model::getVertex(const int faceIdx, const int vertexIdx) const
{
    return vertices_[vertex_indices_[faceIdx * 3 + vertexIdx]];
}

void Model::loadTexture_(std::string filename, const std::string suffix, TGAImage &img)
{
    size_t dot = filename.find_last_of(".");
    if (dot == std::string::npos)
        return;
    std::string texfile = filename.substr(0, dot) + suffix;
    std::cerr << "texture file " << texfile << " loading " << (img.loadFromFile(texfile.c_str()) ? "ok" : "failed") << std::endl;
}

Vector3d Model::getNormal(const Vector2d &uv) const
{
    TGAColor c = normalMap_.getColor(uv[0] * normalMap_.width(), uv[1] * normalMap_.height());
    return Vector3d{(double)c[2], (double)c[1], (double)c[0]} * 2. / 255. - Vector3d{1, 1, 1};
}

Vector3d Model::getNormal(const int faceIdx, const int vertexIdx) const
{
    return normals_[normal_indices_[faceIdx * 3 + vertexIdx]];
}

Vector2d Model::getUV(const int faceIdx, const int vertexIdx) const 
{
    return textures_[texture_indices_[faceIdx * 3 + vertexIdx]];
}
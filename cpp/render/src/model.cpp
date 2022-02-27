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
                // TODO
            }
        }
    }
}
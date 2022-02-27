#ifndef TGA_H_
#define TGA_H_

#include <cstdint>
#include <fstream>
#include <vector>

#pragma pack(push, 1)
struct TGAHeader
{
    std::uint8_t idLength;
    std::uint8_t colorMapType;
    std::uint8_t dataTypeCode;
    std::uint16_t colorMapOrigin;
    std::uint16_t colorMapLength;
    std::uint16_t xOrigin;
    std::uint16_t yOrigin;
    std::uint16_t width;
    std::uint16_t height;
    std::uint8_t bitsPerPixel;
    std::uint8_t imageDescriptor;
}; // struct TGAHeader
#pragma pack(pop)

struct TGAColor
{
    std::uint8_t bgra[4] = {0, 0, 0, 0};
    std::uint8_t bytespp = {0};

    TGAColor() = default;
    TGAColor(const std::uint8_t R, const std::uint8_t G, const std::uint8_t B, std::uint8_t A = 255) : bgra{B, G, R, A}, bytespp(4) {}
    TGAColor(const std::uint8_t *p, const std::uint8_t bpp) : bytespp(bpp)
    {
        for (int i = bpp; i--; bgra[i] = p[i])
            ;
    }
    std::uint8_t &operator[](const int i) { return bgra[i]; }
}; // struct TGAColor

struct TGAImage
{
public:
    enum format
    {
        GRAY = 1,
        RGB = 3,
        RGBA = 4
    };

    TGAImage() : w_(0), h_(0), bpp_(0), data_{} {}
    TGAImage(const int w, const int h, const int bpp) : w_(w), h_(h), bpp_(bpp), data_(w * h * bpp, 0) {}

    bool loadFromFile(const std::string filename);
    bool writeToFile(const std::string filename, const bool vfilp = true, const bool rle = true) const;

    void hFlip();
    void vFlip();

    TGAColor getColor(const int x, const int y) const;
    void setColor(const int x, const int y, const TGAColor &c);

    int width() const { return w_; }
    int height() const { return h_; }

private:
    bool loadRLE_(std::ifstream &in);
    bool unloadRLE_(std::ofstream &out) const;

    int w_;
    int h_;
    int bpp_;
    std::vector<std::uint8_t> data_;

}; // struct TGAImage

#endif // TGA_H_
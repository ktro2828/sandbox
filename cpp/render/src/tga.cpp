#include <iostream>
#include <cstring>

#include "tga.h"

bool TGAImage::loadFromFile(const std::string filename)
{
    std::ifstream in;
    in.open(filename, std::ios::binary);
    if (!in.is_open())
    {
        std::cerr << "cannot open file " << filename << std::endl;
        in.close();
        return false;
    }
    TGAHeader header;
    in.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (!in.good())
    {
        in.close();
        std::cerr << "an error occured while reading the header" << std::endl;
        return false;
    }
    w_ = header.width;
    h_ = header.height;
    bpp_ = header.bitsPerPixel >> 3;
    if (w_ <= 0 || h_ <= 0 || (bpp_ != GRAY && bpp_ != RGB && bpp_ != RGBA))
    {
        in.close();
        std::cerr << "bad bpp (or width/height) value" << std::endl;
        return false;
    }
    size_t nbytes = static_cast<size_t>(bpp_ * w_ * h_);
    data_ = std::vector<std::uint8_t>(nbytes, 0);
    if (3 == header.dataTypeCode || 2 == header.dataTypeCode)
    {
        in.read(reinterpret_cast<char *>(data_.data()), nbytes);
        if (!in.good())
        {
            in.close();
            std::cerr << "an error occured while reading the data" << std::endl;
            return false;
        }
    }
    else if (10 == header.dataTypeCode || 11 == header.dataTypeCode)
    {
        if (!loadRLE_(in))
        {
            in.close();
            std::cerr << "an error occured while reading the data" << std::endl;
            return false;
        }
    }
    else
    {
        in.close();
        std::cerr << "unknown file format" << (int)header.dataTypeCode << std::endl;
        return false;
    }

    if (!(header.imageDescriptor & 0x20))
        vFlip();
    if (header.imageDescriptor & 0x10)
        hFlip();
    std::cerr << w_ << "x" << h_ << "/" << bpp_ * 8 << std::endl;
    in.close();
    return true;
}

bool TGAImage::loadRLE_(std::ifstream &in)
{
    size_t pixelCnt = w_ * h_;
    size_t currPixel = 0;
    size_t currByte = 0;
    TGAColor colorBuffer;
    do
    {
        std::uint8_t chunkHeader = 0;
        chunkHeader = in.get();
        if (!in.good())
        {
            std::cerr << "an error occured while reading the data" << std::endl;
            return false;
        }
        if (chunkHeader < 128)
        {
            chunkHeader++;
            for (int i = 0; i < chunkHeader; ++i)
            {
                in.read(reinterpret_cast<char *>(colorBuffer.bgra), bpp_);
                if (!in.good())
                {
                    std::cerr << "an error occured while reading the header" << std::endl;
                    return false;
                }
                for (int t = 0; t < bpp_; ++t)
                {
                    data_[currByte++] = colorBuffer.bgra[t];
                }
                currPixel++;
                if (currPixel > pixelCnt)
                {
                    std::cerr << "Too many pixels read" << std::endl;
                    return false;
                }
            }
        }
        else
        {
            chunkHeader -= 127;
            in.read(reinterpret_cast<char *>(colorBuffer.bgra), bpp_);
            if (!in.good())
            {
                std::cerr << "an error occured while reading the header" << std::endl;
                return false;
            }
            for (int i = 0; i < chunkHeader; ++i)
            {
                for (int t = 0; t < bpp_; ++t)
                {
                    data_[currByte++] = colorBuffer.bgra[t];
                }
                currPixel++;
                if (currPixel > pixelCnt)
                {
                    std::cerr << "Too many pixels read" << std::endl;
                    return false;
                }
            }
        }
    } while (currPixel < pixelCnt);
    return true;
}

bool TGAImage::writeToFile(const std::string filename, const bool vflip, const bool rle) const
{
    constexpr std::uint8_t developerAreaRef[4] = {0};
    constexpr std::uint8_t extensionAreaRef[4] = {0};
    constexpr std::uint8_t footer[18] = {'T', 'R', 'U', 'E', 'V', 'I', 'S', 'I', 'O', 'N', '-', 'X', 'F', 'I', 'L', 'E', '.', '\0'};
    std::ofstream out;
    if (!out.is_open())
    {
        std::cerr << "cannot open file " << filename << std::endl;
        out.close();
        return false;
    }
    TGAHeader header;
    header.bitsPerPixel = bpp_ << 3;
    header.width = w_;
    header.height = h_;
    header.dataTypeCode = (bpp_ == GRAY ? (rle ? 11 : 3) : (rle ? 10 : 2));
    header.imageDescriptor = vflip ? 0x00 : 0x20;
    out.write(reinterpret_cast<const char *>(&header), sizeof(header));
    if (!out.good())
    {
        out.close();
        std::cerr << "cannot dump the tga file" << std::endl;
        return false;
    }
    if (!rle)
    {
        out.write(reinterpret_cast<const char *>(data_.data()), w_ * h_ * bpp_);
        if (!out.good())
        {
            std::cerr << "cannot unload raw data" << std::endl;
            out.close();
            return false;
        }
    }
    else if (!unloadRLE_(out))
    {
        out.close();
        std::cerr << "cannot unload raw data" << std::endl;
        return false;
    }
    out.write(reinterpret_cast<const char *>(developerAreaRef), sizeof(developerAreaRef));
    if (!out.good())
    {
        std::cerr << "cannout dump the tga file" << std::endl;
        out.close();
        return false;
    }
    out.write(reinterpret_cast<const char *>(extensionAreaRef), sizeof(extensionAreaRef));
    if (!out.good())
    {
        std::cerr << "cannot dump the tga file" << std::endl;
        out.close();
        return false;
    }
    out.write(reinterpret_cast<const char *>(footer), sizeof(footer));
    if (!out.good())
    {
        std::cerr << "cannot dump the tga file" << std::endl;
        out.close();
        return false;
    }
    out.close();
    return true;
}

bool TGAImage::unloadRLE_(std::ofstream &out) const
{
    const std::uint8_t maxChunkLength = 128;
    size_t numPixels = w_ * h_;
    size_t currPixel = 0;
    while (currPixel < numPixels)
    {
        size_t chunkStart = currPixel * bpp_;
        size_t currByte = currPixel * bpp_;
        std::uint8_t runLength = 1;
        bool raw = true;
        while (currPixel + runLength < numPixels && runLength < maxChunkLength)
        {
            bool successEq = true;
            for (int t = 0; successEq && t < bpp_; ++t)
            {
                successEq = (data_[currByte + t] == data_[currByte + t + bpp_]);
            }
            currByte += bpp_;
            if (1 == runLength)
                raw = !successEq;
            if (raw && successEq)
            {
                runLength--;
                break;
            }
            if (!raw && !successEq)
                break;
            runLength++;
        }
        currPixel += runLength;
        out.put(raw ? runLength - 1 : runLength + 127);
        if (!out.good())
        {
            std::cerr << "cannot dump the tga file" << std::endl;
            return false;
        }
        out.write(reinterpret_cast<const char *>(data_.data() + chunkStart), (raw ? runLength * bpp_ : bpp_));
        if (!out.good())
        {
            std::cerr << "cannot dump the tga file" << std::endl;
            return false;
        }
    }
    return true;
}

TGAColor TGAImage::getColor(int x, int y) const
{
    if (!data_.size() || x < 0 || y < 0 || x >= w_ || y >= h_)
        return {};
    return TGAColor(data_.data() + (x + y * w_) * bpp_, bpp_);
}

void TGAImage::setColor(int x, int y, const TGAColor &c)
{
    if (!data_.size() || x < 0 || y < 0 || x >= w_ || y >= h_)
        return;
    memcpy(data_.data() + (x + y * w_) * bpp_, c.bgra, bpp_);
}

void TGAImage::hFlip()
{
    int half = w_ >> 1;
    for (int i = 0; i < half; ++i)
    {
        for (int j = 0; j < h_; ++j)
        {
            for (int b = 0; b < bpp_; ++b)
                std::swap(data_[(i + j * w_) * bpp_ + b], data_[(w_ - 1 - i + j * w_) * bpp_ + b]);
        }
    }
}

void TGAImage::vFlip()
{
    int half = h_ >> 1;
    for (int i = 0; i < w_; ++i)
    {
        for (int j = 0; j < half; ++j)
        {
            for (int b = 0; b < bpp_; ++b)
                std::swap(data_[(i + j * w_) * bpp_ + b], data_[(i + (h_ - 1 - j) * w_) * bpp_ + b]);
        }
    }
}
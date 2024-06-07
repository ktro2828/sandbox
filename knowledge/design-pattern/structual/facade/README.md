# Facade

## 📝 Intent

Facade(ファサード)は、構造に関するデザインパターンの一つで、ライブラリー、フレームワーク、その他のクラスの複雑な組み合わせに対し、簡素化されたインターフェースを提供します。

## 💡 Key Concepts

ファサードは、大変入り組んで複雑なサブシステムへの単純なインターフェースを提供するクラスです。ファサードは、直接サブシステムとやりとりするのに比べると限られた機能しか提供しないかもしれません。 しかし、そこにはクライアントにとって本当に関心のある機能のみが含まれています。

<div align="center">
<img src="./img/structure.png">
</div>

## 💻 Pseudo Code

この例では、Facadeパターンによって、複雑なビデオ変換フレームワークとのやり取りを簡素化します。

<div align="center">
<img src="./img/example.png">
</div>

```cpp
// 以下は、外部作成のビデオ変換フレームワークのクラス。自分たちのものではないので、単純化できない。

class VideoFile { /* ... */}

class OggCompressionCodec { /* ... */}

class MPEG4CompressionCodec { /* ... */}

class CodecFactory { /* ... */}

class BitrateReader { /* ... */}

class AudioMixer { /* ... */}

// フレームわーくの複雑さを隠して、簡素なインターフェイスとするためのFacadeクラスを作成。
// 機能性とシンプルさとの間の取引。
class VideoConverter is
    method convert(filename, format): File is
        file = new VideoFile(filename)
        sourceCodec = (new CodecFactory).extract(file)
        if (format == "mp4")
            destinationCodec = new MPEG4CompressionCodec()
        else
            destinationCodec = new OggCompressionCodec()
        buffer = BitrateReader.read(filename, sourceCodec)
        result = BitrateReader.convert(buffer, destinationCodec)
        result = (new AudioMixer()).fix(result)
        return new File(result);

// アプリケーションのクラスは、複雑なフレームワークが提供する何億ものクラスに依存しない。
// フレームワークを違うものに切り替える場合は、Facadeクラスの書き換えだけで済む。
class Application is
    method main() is
        convertor = new VideoConverter()
        mp4 = convertor.convert("funny-cats-video.ogg", "mp4")
        mp4.save();
```
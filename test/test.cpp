#include <string>
#include <vector>
#include <jpeg.hpp>
#include <opencv2/opencv.hpp>

void print(const std::string name, std::uint64_t load, std::uint64_t save, float diff) {
    printf("%-16s", name.c_str());
    printf("%7d.%d ms ", int(load / 1000), int(load % 1000) / 100);
    printf("%7d.%d ms ", int(save / 1000), int(save % 1000) / 100);
    printf("%12.1f ", diff);
    printf("\n");
}

float get_diff(const std::string pathA, const std::string pathB) {
    cv::Mat imgA = cv::imread(pathA);
    cv::Mat imgB = cv::imread(pathB);
    return cv::norm(imgA, imgB);
}

int main() {
    const std::string root = "../../test/images/";
    const std::string tmp = "output-tmp.jpg";
    const std::vector<std::string> filenames = {
        "jpeg444.jpg",
        "jpeg400jfif.jpg",
        "jpeg420exif.jpg",
        "jpeg422jfif.jpg",
        "jpegCMYK.jpg",
    };

    printf("%s\n", mango::getSystemInfo().c_str());
    printf("-----------------------------------------------------------\n");
    printf("    test-case          load         save          diff     \n");
    printf("-----------------------------------------------------------\n");

    for (auto filename: filenames) {
        auto path = root + filename;

        std::uint64_t time0 = mango::Time::us();
        mango::image::Surface bitmap = mango_load_jpeg(path.c_str());

        std::uint64_t time1 = mango::Time::us();
        mango_save_jpeg(tmp.c_str(), bitmap);

        std::uint64_t time2 = mango::Time::us();
        print(filename, time1 - time0, time2 - time1, get_diff(path, tmp));
    }
}

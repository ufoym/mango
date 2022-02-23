#include <jpeg.hpp>

void print(const char* name, std::uint64_t load, std::uint64_t save)
{
    printf("%s", name);
    printf("%7d.%d ms ", int(load / 1000), int(load % 1000) / 100);
    printf("%7d.%d ms ", int(save / 1000), int(save % 1000) / 100);
    printf("\n");
}


int main(int argc, const char* argv[])
{
    const char* default_filename = "../../test/images/jpeg444.jpg";
    const char* filename = argc > 1 ? argv[1] : default_filename;

    printf("%s\n", mango::getSystemInfo().c_str());
    printf("----------------------------------------------\n");
    printf("                load         save             \n");
    printf("----------------------------------------------\n");

    std::uint64_t time0;
    std::uint64_t time1;
    std::uint64_t time2;

    time0 = mango::Time::us();
    mango::image::Surface bitmap = mango_load_jpeg(filename);

    time1 = mango::Time::us();
    mango_save_jpeg("output-mango.jpg", bitmap);

    time2 = mango::Time::us();
    print("mango:   ", time1 - time0, time2 - time1);
}

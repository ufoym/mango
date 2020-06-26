rd /S /Q ..\mango_build
meson setup ..\mango_build --buildtype=debug --backend=ninja -Dcpp_std=c++17
@rem meson setup ..\erhe_build --buildtype=debug --backend=vs -Dcpp_std=c++17

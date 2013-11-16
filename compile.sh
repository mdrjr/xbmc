#!/bin/bash

./bootstrap

./configure --enable-alsa --disable-pulse --enable-airtunes --enable-airplay --disable-libcec --enable-neon --disable-debug --enable-optimizations --enable-ccache --enable-gles --disable-x11 --disable-gl --enable-non-free --enable-exynos5 --disable-vdpau --disable-vaapi --disable-crystalhd --disable-openmax --disable-rsxs --disable-projectm --disable-fishbmc --disable-nfs --disable-afpclient --disable-dvdcss --disable-optical-drive --disable-libbluray --enable-texturepacker --enable-joystick --with-platform=hybris --with-cpu=cortex-a15 FFMPEG_CFLAGS="-mfloat-abi=hard -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad -ffast-math -mcpu=cortex-a15 -mtune=cortex-a15 -O3 -pipe -fstack-protector" CFLAGS="-mfloat-abi=hard -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad -ffast-math -mcpu=cortex-a15 -mtune=cortex-a15 -O3 -pipe -fstack-protector -DTARGET_HYBRIS=1 -I/usr/local/include/hybris" CXXFLAGS="-mfloat-abi=hard -mfpu=neon -ftree-vectorize -mvectorize-with-neon-quad -ffast-math -mcpu=cortex-a15 -mtune=cortex-a15 -O3 -pipe -fstack-protector -DTARGET_HYBRIS=1 -I/usr/local/include/hybris"

make -j5


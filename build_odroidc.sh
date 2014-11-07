#!/bin/bash
apt-fast build-dep xbmc
./bootstrap
LDFLAGS="-L/usr/local/lib -L/usr/lib/arm-linux-gnueabihf/mali-egl -L/usr/lib/aml_libs" ./configure --disable-static --enable-shared gl_cv_func_gettimeofday_clobber=no ac_cv_lib_bluetooth_hci_devid=no --disable-debug --disable-optimizations --disable-gl --enable-gles --disable-enable \
	--disable-vdpau --disable-vaapi --disable-vtbdecoder --disable-tegra --disable-profiling --disable-joystick --disable-libcec --enable-udev --disable-libusb --disable-goom --disable-rsxs --disable-projectm --enable-waveform --enable-spectrum --disable-fishbmc \
	--enable-x11 --enable-xrandr --enable-ccache --enable-alsa --enable-pulse --enable-rtmp --enable-samba --enable-nfs -enable-afpclient --enable-libvorbisenc --disable-libcap --enable-dvdcss --disable-mid --enable-avahi --enable-upnp --enable-mysql --enable-ssh \
	--enable-airplay --enable-airtunes --enable-non-free --disable-asap-codec --enable-webserver --enable-optical-drive --enable-libbluray --enable-neon --enable-texturepacker --with-ffmpeg=shared --enable-codec=amcodec

mkdir -p /tmp/downloads
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.0/sources/public_sources.tbz2 -O /tmp/downloads/public_sources.tbz2
tar -xf /tmp/downloads/public_sources.tbz2 -C /tmp/downloads Linux_for_Tegra/source/kernel_src.tbz2 
tar -xf /tmp/downloads/Linux_for_Tegra/source/kernel_src.tbz2 -C /tmp/downloads

# cleanup
rm /tmp/downloads/public_sources.tbz2
rm -r /tmp/downloads/Linux_for_Tegra

# patch kernel
cat <<EOF >> /tmp/downloads/kernel/kernel-jammy-src/arch/arm64/configs/defconfig
CONFIG_WLAN=y
CONFIG_WLAN_VENDOR_INTEL=y
CONFIG_IWLWIFI=m
CONFIG_IWLWIFI_LEDS=y
CONFIG_IWLDVM=m
CONFIG_IWLMVM=m
CONFIG_IWLWIFI_OPMODE_MODULAR=y
EOF


sudo apt install -y gcc-aarch64-linux-gnu bc binutils bison dwarves flex gcc git gnupg2 gzip libelf-dev libncurses5-dev libssl-dev make \
     openssl pahole perl-base rsync tar xz-utils

cd /tmp/downloads/kernel
make ARCH=arm64 -j4
cd ..
tar -czvf patched-kernel.tar.gz 
sha256sum patched-kernel.tar.gz > patched-kernel.tar.gz.sha256sum
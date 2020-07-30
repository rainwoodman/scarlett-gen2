version=5.7.11
./merge.pl config-rt kernel-${version}-i686.config > kernel-${version}-i686-rt.config
./merge.pl config-rt kernel-${version}-x86_64.config > kernel-${version}-x86_64-rt.config
# bash process_configs.sh kernel-${version}-x86_64-rt

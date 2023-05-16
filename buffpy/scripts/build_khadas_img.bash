mkdir -p ~/project/khadas
cd ~/project/khadas
git clone --depth 1 https://github.com/khadas/fenix && \
cd fenix && \
source env/setenv.sh && \
make

# write custom OS image into eMMC from removable storage

#     copy your OS image into any removable storage SD / USB Flash , etc (formatted by exfat or ext2/3/4 or fat)
#     start oowow by press FUNCTION + RESET
#     exit from wizard
#     plug removable storage with OS image
#     choose this image to write by menu -> Write image to eMMC and up to .. mounts

# NOTE: supported images

#     raw images dd suitable - .img
#     compressed images .img.xz lzma , .img.gz gzip, .img.zst zstd

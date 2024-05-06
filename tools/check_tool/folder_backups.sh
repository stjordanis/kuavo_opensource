#!/bin/bash


# 定义源目录和目标目录
source_offset_dir=~/.config/lejuconfig/offset.csv

source_config_dir=~/kuavo/src/biped_v2/config/kuavo_v3.4/
source_urdf_dir=~/kuavo/models/biped_gen3.4/urdf/


target_config_dir=~/kuavo_opensource/src/biped_v2/config/kuavo_v3.4/
target_urdf_dir=~/kuavo_opensource/models/biped_gen3.4/urdf/


# ----------------------------------- 打包备份 ----------------------------------- #
# 定义要打包的文件和目录列表
files=(
    $source_config_dir
    $source_urdf_dir
    $source_offset_dir
)

# 定义压缩文件的名称
zipfile=~/confirm_backups.zip

# 打包文件
zip -r $zipfile "${files[@]}"

# 提示打包完成
echo "文件已打包为 $zipfile"



# ----------------------------------同步文件 ----------------------------------- #

# 使用rsync命令替换目标目录
rsync -av --delete $source_config_dir $target_config_dir
rsync -av --delete $source_urdf_dir $target_urdf_dir

# 提示替换完成
echo "文件同步完成"

# 提示打包完成
echo "文件已打包为 $zipfile"





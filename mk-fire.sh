#!/bin/sh
#menu.sh


echo "---------------------------------------------------------"
echo "please enter your choise scripts number:"
echo "请输入要运行脚本的序号:"
echo "[0] Exit Menu"
echo "[1] Update feeds/config/download"
echo "[2] Save Defconfig"
echo "[3] Package Image"
echo "---------------------------------------------------------"
read input

case $input in
    0)
        exit;;
    1)
        SCRIPTS=Update
        ;;
    2)
        SCRIPTS=Defconfig
        ;;
    3)
        SCRIPTS=Package
        ;;
    *)
        echo 'input type number error, exit !'
        exit;;
esac

if [ $SCRIPTS = Update ] 
then
    echo '**************begining update and install feeds************'
    #更新升级软件包列表
    ./scripts/feeds update -a
    ./scripts/feeds install -a


    echo '**************application lubancat_defconfig ***************'
    #生成默认配置文件
    cat ./config/lubancat_defconfig > .config
    make defconfig


    echo '**************download all packages***********************'
    #下载软件包
    make download V=s

    ##如果自己在menuconfig中修改了配置，
    ##使用#把使用默认配置的命令注释掉再执行此脚本可一键更新软件包
fi



if [ $SCRIPTS = Defconfig ] 
then
    echo '**************begining write lubancat_defconfig************'
    #写入默认配置文件
    ./scripts/diffconfig.sh > config/lubancat_defconfig

    cat config/lubancat_defconfig

    echo '**********************write successful *********************'
fi

if [ $SCRIPTS = Package ] 
then
    echo '**************     begining package image       ************'


    DEVICE_DIR='bin/targets/rockchip/armv8/'

    echo "---------------------------------------------------------"
    echo "please enter your choise device number:"
    echo "[0] Exit Menu"
    echo "[1] DoorNet1"
    echo "[2] DoorNet2"
    echo "[3] LubanCat1"
    echo "[4] LubanCat1N"
    echo "[5] LubanCat2"
    echo "[6] LubanCat2N"
    echo "[7] LubanCatZeroN"
    echo "---------------------------------------------------------"
    read input

    case $input in
        0)
            exit;;
        1)
            DEVICE_NAME=DoorNet1
            OUTPUT_DIR=EmbedFire_DoorNet1/
            device_name=embedfire_doornet1
            ;;
        2)
            DEVICE_NAME=DoorNet2
            OUTPUT_DIR=EmbedFire_DoorNet2/
            device_name=embedfire_doornet2
            ;;
        3)
            DEVICE_NAME=LubanCat1
            OUTPUT_DIR=EmbedFire_LubanCat1/
            device_name=lubancat1
            ;;
        4)
            DEVICE_NAME=LubanCat1N
            OUTPUT_DIR=EmbedFire_LubanCat1N/
            device_name=lubancat1n
            ;;
        5)
            DEVICE_NAME=LubanCat2
            OUTPUT_DIR=EmbedFire_LubanCat2/
            device_name=lubancat2
            ;;
        6)
            DEVICE_NAME=LubanCat2N
            OUTPUT_DIR=EmbedFire_LubanCat2N/
            device_name=lubancat2n
            ;;
        7)
            DEVICE_NAME=LubanCatZeroN
            OUTPUT_DIR=EmbedFire_LubanCatZeroN/
            device_name=lubancatzeron
            ;;
        *)
            echo 'input device number error, exit !'
            exit;;
    esac


    echo '**********************config set info *********************'
    echo 'DEVICE_NAME=' $DEVICE_NAME
    echo 'DEVICE_DIR =' $DEVICE_DIR
    echo 'OUTPUT_DIR =' $OUTPUT_DIR
    echo 'Package file path : '${DEVICE_DIR}${OUTPUT_DIR}


    dirdate=`date '+%Y_%m_%d'`
    namedate=`date '+%Y%m%d'`

    mkdir -p ${DEVICE_DIR}${OUTPUT_DIR}${dirdate}

    cp ${DEVICE_DIR}openwrt-rockchip-armv8-*${device_name}-ext4-sysupgrade.img.gz ${DEVICE_DIR}${OUTPUT_DIR}${dirdate}/openwrt-${device_name}-ext4-${namedate}.img.gz
    cp ${DEVICE_DIR}openwrt-rockchip-armv8-*${device_name}-squashfs-sysupgrade.img.gz ${DEVICE_DIR}${OUTPUT_DIR}${dirdate}/openwrt-${device_name}-squashfs-${namedate}.img.gz
    cp ${DEVICE_DIR}openwrt-rockchip-armv8-*${device_name}.manifest ${DEVICE_DIR}${OUTPUT_DIR}${dirdate}/openwrt-${device_name}.manifest

    rm -f ${DEVICE_DIR}${OUTPUT_DIR}${dirdate}/sha256sum
    cd ${DEVICE_DIR}${OUTPUT_DIR}
    sha256sum ${dirdate}/* > ${dirdate}/sha256sum





    echo '**************** package image successful  *****************'
fi


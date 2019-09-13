#!/bin/bash

# ensure segmap homefolder exists
python -c "import segmappy"

SEGMAP_HOME="/media/david/8bee178c-7a37-4307-a49e-d7bfd8f9c563/Rozenberszki/SZTAKI/KITTI/CNN_Segmap/"
if [ -d "$SEGMAP_HOME" ]; then
    # ask about download kitti
    read -p "Do you wish to download kitti files (26GB)? (y/[n]) " yn
    case $yn in
        [Yy]* ) dl_kitti=1;;
        * ) dl_kitti=0;;
    esac

    # ask about downloading segments
    read -p "Do you wish to download segment datasets (3GB)? (y/[n]) " yn
    case $yn in
        [Yy]* ) dl_segments=1;;
        * ) dl_segments=0;;
    esac

    # ask about downloading models
    read -p "Do you wish to download model files (110MB)? (y/[n]) " yn
    case $yn in
        [Yy]* ) dl_models=1;;
        * ) dl_models=0;;
    esac

    # download kitti
    if [ $dl_kitti == 1 ]; then
        echo "Downloading kitti files"
        wget -c -r -np -R "index.html*" -nH --cut-dirs=3 -P $SEGMAP_HOME http://robotics.ethz.ch/~asl-datasets/segmap/segmap_data/kitti/
    fi

    # download segment datasets
    if [ $dl_segments == 1 ]; then
        echo "Downloading segment datasets"
        wget -c -r -np -R "index.html*" -nH --cut-dirs=3 -P $SEGMAP_HOME http://robotics.ethz.ch/~asl-datasets/segmap/segmap_data/training_datasets/
    fi

    # download models
    if [ $dl_models == 1 ]; then
        echo "Downloading model files"
        wget -c -r -np -R "index.html*" -nH --cut-dirs=3 -P $SEGMAP_HOME http://robotics.ethz.ch/~asl-datasets/segmap/segmap_data/trained_models/
    fi
else
    echo "Error: Could not find $SEGMAP_HOME"
fi

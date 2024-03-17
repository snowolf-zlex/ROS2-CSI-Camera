#!/bin/bash
# JUPYTER_PASSWORD=jetson

# 设置 Jupyter Notebook 密码
# jupyter lab --generate-config
# jupyter notebook password ${JUPYTER_PASSWORD}

cd $JUPYTER_WORKDIR 
jupyter lab --ip=0.0.0.0 --no-browser --allow-root &
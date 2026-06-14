#!/bin/zsh
# env_setup.zsh - Zsh environment setup for RADAR-LOCATION-LIDAR container

source ~/env_setup.bash

# Zsh-specific
if [ -d "${HOME}/.opencode/bin" ]; then
    fpath+=("${HOME}/.opencode/bin")
fi

#!/bin/bash

set -euo pipefail
ARCH=$(dpkg --print-architecture)
VERSION_ID=$(grep '^VERSION_ID=' /etc/os-release | cut -d'=' -f2 | tr -d '"')
MAVSDK_VERSION="3.14.0"

echo "Architecture: $ARCH"
echo "Version ID: $VERSION_ID"

pkgs=(
    python3-colcon-common-extensions
    libgeographic-dev
    geographiclib-tools
    wget
)

for pkg in "${pkgs[@]}"; do
    echo "Installing $pkg"
    apt-get install -y "$pkg"
done

echo "Installing MAVSDK version $MAVSDK_VERSION"
if [[ "$ARCH" == "amd64" ]]; then
    PACKAGE="libmavsdk-dev_${MAVSDK_VERSION}_ubuntu${VERSION_ID}_amd64.deb"
else
    PACKAGE="libmavsdk-dev_${MAVSDK_VERSION}_debian12_${ARCH}.deb"
fi

wget https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}/${PACKAGE}
sudo dpkg -i ${PACKAGE}
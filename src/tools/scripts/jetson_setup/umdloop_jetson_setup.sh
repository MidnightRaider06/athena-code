#!/bin/bash
set -euo pipefail

trap 'echo "[ERROR] Script failed at line $LINENO"' ERR

# having both of these are weird but they make sure you use sudo and you're not logged in as root (don't know the userdir if you're logged in as root)
if [[ $EUID -ne 0 ]]; then
    echo "[ERROR] This script must be run as root (use sudo)"
    exit 1
fi

if [[ -z "${SUDO_USER:-}" ]]; then
    echo "[ERROR] Please run with sudo, not as root directly"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ATHENA_REPO_DIR="${USER_HOME}/athena-code"
ATHENA_REPO_URL="https://github.com/umdloop/athena-code.git"

install_ros2_humble() {
    echo "[INFO] Installing ROS2 Humble Hawksbill..."

    echo "[INFO] Installing Git"
    apt-get install -y git-all

    if [[ -f /opt/ros/humble/setup.bash ]]; then
        echo "[WARN] ROS2 Humble already installed, skipping..."
        return 0
    fi

    apt-get update && apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    apt-get install -y software-properties-common curl
    add-apt-repository -y universe
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    apt-get update
    apt-get install -y ros-humble-desktop ros-dev-tools

    echo "[INFO] ROS2 Humble installed successfully"
}

setup_athena_code() {
    echo "[INFO] Setting up athena-code repository..."

    if [[ ! -d "$ATHENA_REPO_DIR" ]]; then
        echo "[INFO] Cloning athena-code repository..."
        sudo -u "$SUDO_USER" git clone "$ATHENA_REPO_URL" "$ATHENA_REPO_DIR"
    else
        echo "[WARN] athena-code directory already exists, skipping clone..."
    fi

    cd "$ATHENA_REPO_DIR" || { echo "[ERROR] Failed to cd to $ATHENA_REPO_DIR"; exit 1; }
    bash dependencies.sh

    echo "[INFO] Building athena-code workspace..."
    
    if ! grep -qxF 'source /opt/ros/humble/setup.bash' "$USER_HOME/.bashrc"; then
        echo 'source /opt/ros/humble/setup.bash' >> "$USER_HOME/.bashrc"
    fi
    
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install

    echo "[INFO] athena-code built successfully"
}

configure_can() {
    echo "[INFO] Configuring CAN interface..."

    while true; do
        echo "Select CAN interface type:"
        echo "1) Native Jetson CAN (can0/mttcan)"
        echo "2) CANable USB adapter (slcand)"
        echo "3) Skip CAN configuration"
        read -p "Enter choice [1-3]: " can_choice

        case $can_choice in
            1) configure_native_can; return ;;
            2) configure_canable; return ;;
            3) echo "[INFO] Skipping CAN configuration"; return ;;
            *)
                echo "[ERROR] Invalid choice. Please try again."
                ;;
        esac
    done
}

setup_can_common() {
    local setup_script=$1
    shift
    local packages=("$@")

    apt-get install -y "${packages[@]}"
    cp "${SCRIPT_DIR}/${setup_script}" /usr/local/bin/umdloop_can_setup.sh
    chmod +x /usr/local/bin/umdloop_can_setup.sh
    cp "${SCRIPT_DIR}/umdloop_can.service" /etc/systemd/system/umdloop_can.service
    setup_can_service
}

configure_native_can() {
    echo "[INFO] Configuring native Jetson CAN interface..."
    setup_can_common "umdloop_can_setup.sh" "can-utils" "busybox"
}

configure_canable() {
    echo "[INFO] Configuring CANable USB adapter..."
    setup_can_common "umdloop_canable_setup.sh" "can-utils"
}

setup_can_service() {
    echo "[INFO] Setting up CAN systemd service..."

    if systemctl is-enabled umdloop_can.service &>/dev/null; then
        echo "[WARN] CAN service already enabled"
        ip link set can0 down 2>/dev/null || true
        systemctl restart umdloop_can.service
    else
        systemctl daemon-reload
        systemctl enable umdloop_can.service
        systemctl start umdloop_can.service
    fi

    echo "[INFO] CAN service installed and enabled"
}

configure_networking() {
    echo "[INFO] Configuring network routes for radios..."

    echo "Available network interfaces:"
    ip link show | grep -E '^[0-9]+:' | awk -F': ' '{print $2}' | grep -v lo

    read -p "Enter the network interface name or press Enter to skip: " NETWORK_INTERFACE

    if [[ -z "$NETWORK_INTERFACE" ]]; then
        echo "[WARN] Skipping network configuration"
        return 0
    fi

    read -p "Enter the IP address to assign (e.g., 192.168.1.10): " NETWORK_IP

    if [[ -z "$NETWORK_IP" ]]; then
        echo "[WARN] No IP provided, skipping network configuration"
        return 0
    fi

    read -p "Enter subnet prefix length [24]: " SUBNET_PREFIX
    SUBNET_PREFIX="${SUBNET_PREFIX:-24}"

    local conn_name="athena-radio"

    nmcli connection delete "$conn_name" 2>/dev/null || true

    nmcli connection add type ethernet \
        con-name "$conn_name" \
        ifname "$NETWORK_INTERFACE" \
        ipv4.method manual \
        ipv4.addresses "${NETWORK_IP}/${SUBNET_PREFIX}" \
        connection.autoconnect yes

    nmcli connection up "$conn_name"

    echo "[INFO] Network configuration completed for $NETWORK_INTERFACE"
}

main() {
    echo "[INFO] Starting Jetson setup for athena-code..."
    echo

    install_ros2_humble
    echo

    setup_athena_code
    echo

    configure_can
    echo

    configure_networking
    echo

    echo "[INFO] Setup complete!"
    echo "[INFO] Please reboot the system to ensure all changes take effect."
}

main
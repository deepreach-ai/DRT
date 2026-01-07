#!/usr/bin/env bash
set -euo pipefail

TARGET_DOCKER="/opt/dlami/nvme/docker"
TARGET_CONTAINERD="/opt/dlami/nvme/containerd"
SRC_DOCKER="/var/lib/docker"
SRC_CONTAINERD="/var/lib/containerd"

echo "Detecting NVMe volume and preparing directories..."
sudo mkdir -p "$TARGET_DOCKER" "$TARGET_CONTAINERD"
sudo chown -R root:root /opt/dlami/nvme

echo "Stopping Docker and containerd services..."
sudo systemctl stop docker || true
sudo systemctl stop containerd || true

echo "Syncing existing Docker data to NVMe..."
if [ -d "$SRC_DOCKER" ]; then
  sudo rsync -aH "$SRC_DOCKER/" "$TARGET_DOCKER/"
fi

echo "Syncing existing containerd data to NVMe..."
if [ -d "$SRC_CONTAINERD" ]; then
  sudo rsync -aH "$SRC_CONTAINERD/" "$TARGET_CONTAINERD/"
fi

echo "Configuring Docker to use NVMe data-root..."
echo "{ \"data-root\": \"$TARGET_DOCKER\" }" | sudo tee /etc/docker/daemon.json >/dev/null

echo "Configuring containerd to use NVMe root..."
sudo mkdir -p /etc/containerd
sudo bash -c 'containerd config default > /etc/containerd/config.toml'
sudo sed -i "s#root = \"/var/lib/containerd\"#root = \"$TARGET_CONTAINERD\"#" /etc/containerd/config.toml

echo "Setting up bind mounts to ensure all writes go to NVMe..."
sudo mkdir -p "$SRC_DOCKER" "$SRC_CONTAINERD"
sudo mount --bind "$TARGET_DOCKER" "$SRC_DOCKER"
sudo mount --bind "$TARGET_CONTAINERD" "$SRC_CONTAINERD"

FSTAB_LINE_DOCKER="$TARGET_DOCKER $SRC_DOCKER none bind 0 0"
FSTAB_LINE_CONTAINERD="$TARGET_CONTAINERD $SRC_CONTAINERD none bind 0 0"
if ! grep -qs "$FSTAB_LINE_DOCKER" /etc/fstab; then
  echo "$FSTAB_LINE_DOCKER" | sudo tee -a /etc/fstab >/dev/null
fi
if ! grep -qs "$FSTAB_LINE_CONTAINERD" /etc/fstab; then
  echo "$FSTAB_LINE_CONTAINERD" | sudo tee -a /etc/fstab >/dev/null
fi

echo "Starting containerd and Docker..."
sudo systemctl restart containerd
sudo systemctl restart docker

echo "Verifying configuration..."
docker info | grep "Docker Root Dir" || true
mount | grep -E "/var/lib/docker|/var/lib/containerd" || true
sudo du -sh "$TARGET_DOCKER" || true
sudo du -sh "$TARGET_CONTAINERD" || true

echo "NVMe migration complete."

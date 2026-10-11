Quick reference cheat‑sheet (copy‑paste)
# 1. Edit files under: ros2_stack/third_party/mowglinext/gui/web/src/

# 2. Build ARM64 image (mower)
cd ros2_stack
ARCH=arm64 ./gui/build.sh

# 3. Save to tarball
docker save mower-gui:arm64 | gzip > /tmp/mower-gui-arm64.tar.gz

# 4. Copy to mower
sshpass -p "$MOWER_PASS" scp -o StrictHostKeyChecking=no /tmp/mower-gui-arm64.tar.gz airseekers@192.168.1.105:/tmp/

# 5. Load on mower
sshpass -p "$MOWER_PASS" ssh -o StrictHostKeyChecking=no airseekers@192.168.1.105 \
    "docker load < /tmp/mower-gui-arm64.tar.gz"

# 6. Replace container
sshpass -p "$MOWER_PASS" ssh -o StrictHostKeyChecking=no airseekers@192.168.1.105 <<'EOF'
docker rm -f mower_gui || true
cd /userdata/ros2_stack
docker compose -f docker/docker-compose.yml -f docker/docker-compose.gui.yml up -d mower_gui
EOF

# 7. Verify
sshpass -p "$MOWER_PASS" ssh -o StrictHostKeyChecking=no airseekers@192.168.1.105 \
    "docker inspect mower_gui --format '{{.Image}}'"

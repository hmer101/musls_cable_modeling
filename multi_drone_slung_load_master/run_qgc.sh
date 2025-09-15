#!/bin/bash

USERNAME="qgcuser"
APPIMAGE_PATH="/workspace_docker_build/QGroundControl.AppImage"
ALIAS_NAME="qgc"

# Create user if it doesn't exist
if ! id "$USERNAME" &>/dev/null; then
    echo "Creating user $USERNAME..."
    useradd -m -s /bin/bash "$USERNAME"
fi

# Add user to dialout group
usermod -a -G dialout "$USERNAME"

# Ensure alias exists in .bashrc
# BASHRC="/home/$USERNAME/.bashrc"
# ALIAS_CMD="alias $ALIAS_NAME=\"$APPIMAGE_PATH\""

# if ! grep -Fxq "$ALIAS_CMD" "$BASHRC"; then
#     echo "$ALIAS_CMD" >> "$BASHRC"
#     chown "$USERNAME:$USERNAME" "$BASHRC"
# fi

# Ensure AppImage is executable
chmod +x "$APPIMAGE_PATH"

# Launch QGroundControl as the non-root user
echo "Launching QGroundControl as $USERNAME..."
su - "$USERNAME" -c "$APPIMAGE_PATH"

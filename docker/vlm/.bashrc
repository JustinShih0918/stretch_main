# Interactive shell setup for the on-Thor VLM perception image.
# Trimmed copy of docker/sim/.bashrc: no Gazebo, no Isaac Sim, no rosdep, and
# the first-launch build is scoped to the semantic-navigation packages.

# Do nothing for non-interactive shells. Unlike the sim image, compose.yaml
# here runs its services as `bash -lc "..."` — a *login* shell, which sources
# ~/.profile, which sources this file. Without this guard every service would
# re-run the first-launch build before starting its node.
# Ref: the stock Ubuntu /etc/skel/.bashrc opens the same way.
case $- in
    *i*) ;;
      *) return;;
esac

# Setup paths in `~/.profile` to allow unified environment variable across login/non-login shells
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi
# pip installs torch and the LocateAnything runtime into ~/.local (the image
# runs pip as the unprivileged user), so this must come before any python use.
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Source colcon-argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

ROS2_WS="${ROS2_WS:-$HOME/stretch_main}"
# Dedicated colcon bases, NOT the default build/ install/ log/. The workspace
# is bind-mounted and is regularly built by the other images too (docker/deploy
# runs as root, docker/ci as root, this one as uid 1000): sharing one install
# tree means permission errors and artifacts from mixed ROS environments.
# Keep these in sync with compose.yaml.
VLM_BUILD_BASE="$ROS2_WS/build_vlm"
VLM_INSTALL_BASE="$ROS2_WS/install_vlm"
VLM_LOG_BASE="$ROS2_WS/log_vlm"

# Optionally build the workspace if it has not been built yet.
# This image only builds the semantic-navigation subset: the VLM perception
# nodes plus the projection node / costmap layer they feed. nav2 itself, the
# BT engine and the Stretch driver run on the robot (docker/deploy), not here.
if [ ! -f "$VLM_INSTALL_BASE/setup.bash" ]; then
    echo "Workspace has not been built yet. Building workspace..."
    cd "$ROS2_WS"
    # --log-base is a colcon *global* option: it must precede the verb.
    colcon --log-base "$VLM_LOG_BASE" build --symlink-install \
        --build-base "$VLM_BUILD_BASE" \
        --install-base "$VLM_INSTALL_BASE" \
        --packages-up-to semantic_perception semantic_traversability
    echo "Workspace built."
fi

# Source workspace environment
source "$VLM_INSTALL_BASE/setup.bash"
echo "Successfully built workspace and configured environment variables."

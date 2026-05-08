# /etc/profile.d/context.sh (read-only)
CTX="$WS_PATH/tools/dev/context_env.sh"

if [ -f "$CTX" ]; then
  . "$CTX"
fi

# Librealsense buildeada desde source (v2.57.6, RSUSB backend) debe tener
# precedencia sobre la version de apt (/opt/ros/humble/bin) que usa el
# backend de kernel y no funciona sin permisos de root.
export PATH="/usr/local/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

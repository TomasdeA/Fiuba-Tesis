# /etc/profile.d/context.sh (read-only)
WS="${WS_PATH:-/home/tomasdea/Tesis/develop/nav_mapper}"
CTX="$WS/tools/dev/context_env.sh"

if [ -f "$CTX" ]; then
  . "$CTX"
fi


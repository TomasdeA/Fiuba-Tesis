# /etc/profile.d/context.sh (read-only)
CTX="$WS_PATH/tools/dev/context_env.sh"

if [ -f "$CTX" ]; then
  . "$CTX"
fi



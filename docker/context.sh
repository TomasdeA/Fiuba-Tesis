# /etc/profile.d/context.sh (se monta read-only desde el host)
# Se ejecuta en shells de login y también lo toma BASH_ENV para no-interactivas

WS=/workspaces/tesis_nav_assistant_ws
CTX="$WS/tools/dev/context_env.sh"

# Sorcear si existe
if [ -f "$CTX" ]; then
  # shellcheck source=/dev/null
  . "$CTX"
fi

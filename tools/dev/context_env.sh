# shellcheck shell=bash
# Contexto host vs container + helpers
# Debe ser SOURCEADO desde .bashrc/.bash_profile (no ejecutado)

# -------- Detección de container --------
_is_container() {
  if [ -f "/.dockerenv" ]; then
    return 0
  fi
  if grep -E -q '(docker|podman|containerd|kubepods|libpod)' /proc/1/cgroup 2>/dev/null; then
    return 0
  fi
  if [ -n "${RUNNING_IN_CONTAINER:-}" ] || [ -n "${IS_DOCKER:-}" ]; then
    return 0
  fi
  return 1
}

if _is_container; then
  export SHELL_CONTEXT="container"
else
  export SHELL_CONTEXT="host"
fi

# -------- Forzar tag en PS1 al final (evitando que ~/.bashrc lo pise) --------
__ctx_apply_ps1() {
  [[ $- == *i* ]] || return 0
  [ -z "${CTX_KEEP_PS1:-}" ] || return 0

  local tag
  if [ "$SHELL_CONTEXT" = "container" ]; then
    tag="\[\e[1;33m\][docker]\[\e[0m\] "
  else
    tag="\[\e[1;36m\][local]\[\e[0m\] "
  fi

  # Evitar duplicar
  case "$PS1" in
    *"[docker]"*|*"[local]"*) : ;;
    *) PS1="${tag}${PS1}" ;;
  esac
}

__ctx_prompt_hook() {
  # solo si existe la función (en ESTE shell)
  if declare -F __ctx_apply_ps1 >/dev/null 2>&1; then
    __ctx_apply_ps1
  fi
}

if [[ $- == *i* ]]; then
  if [ -n "${PROMPT_COMMAND:-}" ]; then
    PROMPT_COMMAND="__ctx_prompt_hook; $PROMPT_COMMAND"
  else
    PROMPT_COMMAND="__ctx_prompt_hook"
  fi
  export PROMPT_COMMAND
fi


# -------- Banner 1 sola vez por sesión interactiva --------
if [[ $- == *i* ]] && [ -z "${_CTX_BANNER_SHOWN:-}" ] && [ -z "${CTX_SKIP_BANNER:-}" ]; then
  if [ "$SHELL_CONTEXT" = "container" ]; then
    printf "\e[1;33m[INFO]\e[0m corriendo desde \e[1mDOCKER\e[0m\n"
  else
    printf "\e[1;36m[INFO]\e[0m corriendo \e[1mLOCAL\e[0m\n"
  fi
  export _CTX_BANNER_SHOWN=1
fi

# -------- Evitar doble carga del init pesado (pero permitir forzar re-aplicar) --------
if [[ -n "${__CTX_INIT_DONE:-}" ]] && [[ -z "${CTX_FORCE:-}" ]]; then
  return 0
fi
__CTX_INIT_DONE=1

# -------- Helpers de contexto --------
_require_container() {
  if [ "$SHELL_CONTEXT" != "container" ]; then
    echo "Este comando solo corre dentro de docker. Entrá al contenedor e intentá de nuevo." >&2
    return 1
  fi
}
_require_host() {
  if [ "$SHELL_CONTEXT" != "host" ]; then
    echo "Este comando solo corre en el host (no dentro del contenedor)." >&2
    return 1
  fi
}

_is_raspi_host() {
  # True si estamos en una Raspberry Pi (host), sin depender del OS exacto
  # /proc/device-tree/model existe en Raspi. En PCs normalmente no.
  if [ -r /proc/device-tree/model ]; then
    if tr -d '\0' </proc/device-tree/model | grep -qi 'raspberry pi'; then
      return 0
    fi
  fi
  return 1
}
# -------- Root del repo (nav_mapper) --------
_CTX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$_CTX_DIR/../.." && pwd)"
export WS_ROOT

# -------- Sourcing condicionado --------
if [ "$SHELL_CONTEXT" = "container" ]; then
  [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
else
  [ -f /usr/share/bash-completion/bash_completion ] && source /usr/share/bash-completion/bash_completion
  [ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
fi

# -------- Comandos del usuario (con guardas de contexto) --------

# 1) Solo dentro del contenedor: Realsense
rs-start() {
  _require_container || return 1
  ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=false enable_accel:=false
}

# 2) Solo en el host: correr tesis
tesis-run() {
  _require_host || return 1

  if [ ! -d "$WS_ROOT" ]; then
    echo "No encuentro WS_ROOT=$WS_ROOT. ¿Se sourceó context_env.sh?" >&2
    return 1
  fi

  cd "$WS_ROOT" || return 1

  local compose_file=""
  if _is_raspi_host; then
    compose_file="docker-compose.raspi.yml"
    echo "[tesis-run] Host detectado: RASPI -> usando $compose_file"
    docker compose -f "$compose_file" up -d || return 1
  else
    compose_file="docker-compose.yml"
    echo "[tesis-run] Host detectado: PC -> usando $compose_file"
    docker compose -f "$compose_file" up -d || return 1
  fi

  echo "[tesis-run] Entrando al contenedor tesis_nav_dev..."
  docker exec -it tesis_nav_dev bash -l
}

nav-start()        { ros2 launch nav_bringup nav.launch.py; }
nav-start-hw()     { ros2 launch nav_bringup nav.launch.py use_hw:=true; }
nav-start-viz()    { ros2 launch nav_bringup nav.launch.py use_viz:=true; }
nav-start-hw-viz() { ros2 launch nav_bringup nav.launch.py use_hw:=true use_viz:=true; }

# shellcheck shell=bash
# Contexto host vs container + helpers
# Debe ser SOURCEADO desde .bashrc/.bash_profile (no ejecutado)

# Evitar doble carga (pero permitir forzar re-aplicar)
if [[ -n "${__CTX_INIT_DONE:-}" ]] && [[ -z "${CTX_FORCE:-}" ]]; then
  return 0
fi
__CTX_INIT_DONE=1


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

# -------- Banner 1 sola vez por sesión interactiva --------
if [[ $- == *i* ]] && [ -z "${_CTX_BANNER_SHOWN:-}" ] && [ -z "${CTX_SKIP_BANNER:-}" ]; then
  if [ "$SHELL_CONTEXT" = "container" ]; then
    printf "\e[1;33m[INFO]\e[0m corriendo desde \e[1mDOCKER\e[0m\n"
  else
    printf "\e[1;36m[INFO]\e[0m corriendo \e[1mLOCAL\e[0m\n"
  fi
  export _CTX_BANNER_SHOWN=1
fi

# -------- Prompt con etiqueta --------
if [ -z "${CTX_KEEP_PS1:-}" ]; then
  if [ "$SHELL_CONTEXT" = "container" ]; then
    export PS1="\[\e[1;33m\][docker]\[\e[0m\] \u@\h:\w\$ "
  else
    export PS1="\[\e[1;36m\][local]\[\e[0m\] \u@\h:\w\$ "
  fi
fi

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

# Macros
only_in_container() { _require_container || return 1; "$@"; }
only_on_host()      { _require_host || return 1;      "$@"; }

# -------- Aliases / funciones de ejemplo --------
dk-logs()       { _require_container || return 1; tail -F /var/log/app/*.log 2>/dev/null || tail -F /var/log/*.log; }
dk-enter()      { _require_host || return 1; docker exec -it "${1:?container_name}" bash; }
rosdep-update() { _require_container || return 1; sudo rosdep update; }

# -------- Sourcing condicionado (editá a gusto) --------
if [ "$SHELL_CONTEXT" = "container" ]; then
  # ROS adentro del contenedor
  [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/humble/setup.bash
  [ -f ~/ws/install/setup.bash ]   && source ~/workspace/tesis_nav_assistant_ws/install/setup.bash
else
  # Cosas del host
  [ -f /usr/share/bash-completion/bash_completion ] && source /usr/share/bash-completion/bash_completion
fi

# -------- Comandos del usuario (con guardas de contexto) --------

# 1) Solo dentro del contenedor: Realsense
rs-start() {
  _require_container || return 1
  # Si necesitás activar ROS dentro del contenedor, debería estar ya en la sección de sourcing.
  ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true enable_accel:=true align_depth:=true
}

# 2) Solo en el host: correr tesis
tesis-run() {
  _require_host || return 1
  local WS="$HOME/tesis_nav_assistant_ws"
  if [ ! -d "$WS" ]; then
    echo "No encuentro $WS. Ajustá la ruta en context_env.sh." >&2
    return 1
  fi
  cd "$WS" || return 1
  make run
}

# (Opcional) si igual querés tener *aliases*, los apuntamos a las funciones:
alias rs-start='rs-start'
alias tesis-run='tesis-run'


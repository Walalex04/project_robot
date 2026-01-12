#!/bin/bash
# Limpiar completamente el entorno de Snap
unset SNAP
unset SNAP_NAME
unset SNAP_ARCH
unset SNAP_VERSION
unset SNAP_REVISION
unset SNAP_LIBRARY_PATH
unset SNAP_DATA

# Crear LD_LIBRARY_PATH limpio
CLEAN_LD_PATH=""
for path in $(echo $LD_LIBRARY_PATH | tr ':' '\n'); do
    if [[ ! "$path" =~ /snap/ ]]; then
        CLEAN_LD_PATH="${CLEAN_LD_PATH}:$path"
    fi
done

# Añadir rutas del sistema primero
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu${CLEAN_LD_PATH}"

# Forzar X11
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11

# Ejecutar launch file
ros2 launch robot display.launch.py
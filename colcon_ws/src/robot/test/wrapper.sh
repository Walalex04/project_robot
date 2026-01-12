#!/bin/bash
# Wrapper corregido para RViz

# Desmontar Snap si existe
if mount | grep -q "/snap/core"; then
    echo "Desmontando Snap mounts..."
    sudo umount -l /snap/core20/current 2>/dev/null || true
fi

# PATH limpio
CLEAN_PATH=""
for p in $(echo $PATH | tr ':' ' '); do
    if [[ ! "$p" =~ /snap/ ]]; then
        CLEAN_PATH="$CLEAN_PATH:$p"
    fi
done
export PATH="${CLEAN_PATH:1}"

# LD_LIBRARY_PATH con TODAS las librerías necesarias
export LD_LIBRARY_PATH="\
/usr/lib/x86_64-linux-gnu:\
/lib/x86_64-linux-gnu:\
/opt/ros/humble/lib:\
/opt/ros/humble/lib/x86_64-linux-gnu:\
/usr/lib"

# Forzar X11
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY

# Verificar Ogre
if [ ! -f /usr/lib/x86_64-linux-gnu/libOgreMain.so.1.12.1 ] && \
   [ ! -f /opt/ros/humble/lib/libOgreMain.so.1.12.1 ]; then
    echo "ERROR: Ogre no encontrado. Instalando..."
    sudo apt install libogre-1.12.0 -y
fi

# Ejecutar RViz
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "Ejecutando RViz..."
exec /opt/ros/humble/lib/rviz2/rviz2 "$@"
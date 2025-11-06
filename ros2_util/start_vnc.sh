x11vnc -create -localhost \
    -env X11VNC_CREATE_GEOM=${1:-2200x1000x16} \
    -env X11VNC_FINDDISPLAY_ALWAYS_FAILS=1 \
    -gone 'killall Xvfb' \
    -nopw
    #-env FD_PROG=/usr/bin/fluxbox \

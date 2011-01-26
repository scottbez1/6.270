#!/bin/bash
/bin/bash -c "guvcview --control_only --device=/dev/video$1 -l ~/6.270/vision/uvcSettings.gpfl & "; sleep 1 && killall guvcview
make && ./vision $@


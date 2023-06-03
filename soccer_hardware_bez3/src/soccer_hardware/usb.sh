#!/usr/bin/env bash
set -e
setserial /dev/ttyUSB* low_latency && setserial /dev/ttyACM* low_latency
SUDOERD_BASENAME="sudo_setserial"
SUDOERD_FILE="/etc/sudoers.d/$SUDOERD_BASENAME"
SUDOER_LINE="$(logname) ALL=(root) NOPASSWD: $(realpath "$0")"
EDIT_CMD="echo '$SUDOER_LINE' >> $SUDOERD_FILE"
if [ -e SUDOERD_FILE ]; then
	[ -z "$(grep -F EDIT_CMD SUDOERD_FILE)" ] && exit 0;
else
	touch SUDOERD_FILE
fi

echo "$SUDOER_LINE" | sudo EDITOR='tee -a' visudo -f "$SUDOERD_FILE"

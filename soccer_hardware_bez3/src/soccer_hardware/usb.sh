#!/usr/bin/env bash
set -e
SUDOERD_BASENAME="sudo_setserial"
SUDOERD_FILE="/etc/sudoers.d/$SUDOERD_BASENAME"
SUDOER_LINE="$(logname) ALL=(root) NOPASSWD: $(realpath "$0")"
EDIT_CMD="echo '$SUDOER_LINE' >> $SUDOERD_FILE"
if ! [ -e "$SUDOERD_FILE" ]; then
	echo "!!!"
	sudo touch "$SUDOERD_FILE"
fi

if [ -z "$(grep -F "$SUDOER_LINE" "$SUDOERD_FILE")" ]; then
	echo "$SUDOER_LINE" | sudo EDITOR='tee -a' visudo -f "$SUDOERD_FILE"
fi

set +e
setserial /dev/ttyUSB* low_latency
setserial /dev/ttyACM* low_latency
setserial /dev/ttyTHS* low_latency
echo "Set serials to low latency"

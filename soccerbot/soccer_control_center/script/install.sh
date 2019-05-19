#!/bin/bash

## Installation script for ROS Monitor
if [ "$1" == "-h" -o "$1" == "--help" ]; then
  echo "Installation of ROS monitoring services, execute this once on machines that want monitoring service"
  echo "Usage: `install $0` [map_name:=(map to use)]"
  exit 0
fi

# Get bash source directory
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTPATH="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

# Variables
USER=${SUDO_USER:-$USER}
HOME=/home/$USER
ROOT_DIR=$HOME/soccer

# Prompt for sudo
[ "$UID" -eq 0 ] || exec sudo -E bash "$0" "$@"

# Install and Start FileServer
systemctl stop lighttpd
cat > /etc/lighttpd/lighttpd.conf <<- EOM
server.modules = (
  "mod_access",
  "mod_redirect",
  "mod_setenv",
  "mod_dirlisting",
)

dir-listing.activate = "enable"
setenv.add-response-header = ( "Access-Control-Allow-Origin" => "*" )

server.document-root        = "$ROOT_DIR"
server.upload-dirs          = ( "/var/cache/lighttpd/uploads" )
server.errorlog             = "/var/log/lighttpd/error.log"
server.pid-file             = "/var/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = "8000"
EOM
systemctl start lighttpd

rm -rf /var/log/soccer/supervisor
mkdir -p /var/log/soccer/supervisor
chown -R $USER:$USER /var/log/soccer/supervisor
rm -rf $HOME/soccer/supervisor
mkdir -p $HOME/soccer/supervisor
chown -R $USER:$USER $HOME/soccer/
rm -rf /var/log/soccer_supervisord.log
rm -rf /tmp/supervisor.sock
rm -rf /tmp/soccer_supervisord.pid

# Copy supervisor.conf
cat > /etc/soccer_supervisor.conf <<- EOM
; supervisor config file for control center.

[unix_http_server]
file = /tmp/supervisor.sock
chown = $USER

[inet_http_server]
port = *:6001

[supervisord]
logfile = /var/log/soccer_supervisord.log ; (main log file;default $CWD/supervisord.log)
logfile_maxbytes = 50MB        ; (max main logfile bytes b4 rotation;default 50MB)
logfile_backups = 10           ; (num of main logfile rotation backups;default 10)
loglevel = info                ; (log level;default info; others: debug,warn,trace)
pidfile = /tmp/soccer_supervisord.pid ; (supervisord pidfile;default supervisord.pid)
nodaemon = false               ; (start in foreground if true;default false)
minfds = 1024                  ; (min. avail startup file descriptors;default 1024)
minprocs = 200                 ; (min. avail process descriptors;default 200)
user = $USER

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[supervisorctl]
serverurl = unix:///tmp/supervisor.sock

[include]
files = $HOME/soccer/supervisor/*.conf
EOM

# Copy supervisor systemd service
cat > /etc/systemd/system/soccer_supervisor.service <<- EOM
[Unit]
Description=Supervisor process control system for UNIX
Documentation=http://supervisord.org
After=network.target

[Service]
ExecStart=/usr/bin/supervisord -n -c /etc/soccer_supervisor.conf
ExecStop=/usr/bin/supervisorctl -c /etc/soccer_supervisor.conf shutdown
ExecReload=/usr/bin/supervisorctl -c /etc/soccer_supervisor.conf reload
KillMode=process
Restart=on-failure
RestartSec=50s

[Install]
WantedBy=multi-user.target
EOM

# Reset supervisor log file
rm /var/log/soccer_supervisord.log
touch /var/log/soccer_supervisord.log
chown -R $USER:$USER /var/log/soccer_supervisord.log

# Restart daemon
systemctl daemon-reload
systemctl restart soccer_supervisor
systemctl enable soccer_supervisor

# Create link to the soccer directory
rm -f ${ROOT_DIR}/config.yaml
sudo -u $USER ln -s ${SCRIPTPATH}/../config/config.yaml ${ROOT_DIR}/config.yaml

# Profile script
touch /etc/profile.d/soccer.sh
chmod ug+rwx /etc/profile.d/soccer.sh
chown $USER:$USER /etc/profile.d/soccer.sh

cat > /etc/profile.d/soccer.sh <<- EOM
export HOSTNAME=$(echo $(hostname) | tr '-' '_') # Remove underscore characters from the hostname because this will cause a problem with node names that use the hostname in ROS
export ROS1_WS_PATH=${ROS1_WS_PATH:-${HOME}/ws}
export ROS1_DISTRO=${ROS1_DISTRO:-kinetic}
EOM

# Do not need to login to own computer
if [ ! -f "$HOME/.ssh/id_rsa.pub" ]; then
    echo -ne '\ny\n\n' | sudo -u $USER ssh-keygen
fi
grep "$(cat $HOME/.ssh/id_rsa.pub)" $HOME/.ssh/authorized_keys  > /dev/null || cat $HOME/.ssh/id_rsa.pub >> $HOME/.ssh/authorized_keys

# set alias for supervisorctl
if ! (grep -q "alias controlctl" $HOME/.bashrc 2> /dev/null); then
      echo "alias controlctl='supervisorctl -c /etc/soccer_supervisor.conf'" >> $HOME/.bashrc
fi

echo -ne "Device setup successfully.\nSource bashrc and do 'sudo controlctl status' to see running processes.\n"

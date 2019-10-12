#!/bin/bash

## Installation script for ROS Monitor
if [ "$1" == "-h" -o "$1" == "--help" ]; then
  echo "Installation of ROS monitoring services, execute this once on machines that want monitoring service"
  echo "Usage: `install $0` [map_name:=(map to use)]"
  echo "Example: ./install nittsu"
  exit 0
fi

# Get bash source directory
SCRIPTPATH="$(dirname "$(realpath "$0")")"

# Variables
RR_USER=${SUDO_USER:-$USER}
RR_HOME=/home/$RR_USER
ROOT_DIR=$RR_HOME/sootballs

# Prompt for sudo
[ "$UID" -eq 0 ] || exec sudo -E bash "$0" "$@"

###### Install and Start FileServer ######

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
systemctl start lighttpd || sudo service lighttpd start

rm -rf /var/log/sootballs/supervisor
mkdir -p /var/log/sootballs/supervisor
chown -R $RR_USER:$RR_USER /var/log/sootballs/supervisor
rm -rf $RR_HOME/sootballs/supervisor
mkdir -p $RR_HOME/sootballs/supervisor
chown -R $RR_USER:$RR_USER $RR_HOME/sootballs/
rm -rf /var/log/sootballs_supervisord.log
rm -rf /tmp/supervisor.sock
rm -rf /tmp/sootballs_supervisord.pid

###### Install and Start Sootballs Supervisor ######

cat > /etc/sootballs_supervisor.conf <<- EOM
; supervisor config file for control center.

[unix_http_server]
file = /tmp/supervisor.sock
chown = $RR_USER

[inet_http_server]
port = *:6001

[supervisord]
logfile = /var/log/sootballs_supervisord.log ; (main log file;default $CWD/supervisord.log)
logfile_maxbytes = 50MB        ; (max main logfile bytes b4 rotation;default 50MB)
logfile_backups = 10           ; (num of main logfile rotation backups;default 10)
loglevel = info                ; (log level;default info; others: debug,warn,trace)
pidfile = /tmp/sootballs_supervisord.pid ; (supervisord pidfile;default supervisord.pid)
nodaemon = false               ; (start in foreground if true;default false)
minfds = 1024                  ; (min. avail startup file descriptors;default 1024)
minprocs = 200                 ; (min. avail process descriptors;default 200)
user = $RR_USER

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[supervisorctl]
serverurl = unix:///tmp/supervisor.sock

[include]
files = $RR_HOME/sootballs/supervisor/*.conf
EOM

# Copy supervisor systemd service
cat > /etc/systemd/system/sootballs_supervisor.service <<- EOM
[Unit]
Description=Supervisor process control system for UNIX
Documentation=http://supervisord.org
After=network.target

[Service]
ExecStart=/usr/bin/supervisord -n -c /etc/sootballs_supervisor.conf
ExecStop=/usr/bin/supervisorctl -c /etc/sootballs_supervisor.conf shutdown
ExecReload=/usr/bin/supervisorctl -c /etc/sootballs_supervisor.conf reload
KillMode=process
Restart=on-failure
RestartSec=50s

[Install]
WantedBy=multi-user.target
EOM

# Reset supervisor log file
rm -f /var/log/sootballs_supervisord.log
touch /var/log/sootballs_supervisord.log
chown -R $RR_USER:$RR_USER /var/log/sootballs_supervisord.log

# Restart daemon
systemctl daemon-reload
systemctl enable sootballs_supervisor
systemctl restart sootballs_supervisor || /usr/bin/supervisord -n -c /etc/sootballs_supervisor.conf &

# Create link to the sootballs directory
rm -f ${ROOT_DIR}/config.yaml
sudo -u $RR_USER ln -s ${SCRIPTPATH}/../config/config.yaml ${ROOT_DIR}/config.yaml

###### Install and Robot Announcement ######

cp -f ${SCRIPTPATH}/robot_announcement.py /usr/local/bin/robot_announcement.py
chmod +x /usr/local/bin/robot_announcement.py

cat > /etc/systemd/system/sootballs_robot_announcement.service <<- EOM
[Unit]
Description=Sootballs Robot Annoucement Service
After=multi-user.target
Conflicts=getty@tty1.service

[Service]
Type=simple
ExecStart=/usr/bin/python2.7 /usr/local/bin/robot_announcement.py
StandardInput=tty-force

[Install]
WantedBy=multi-user.target
EOM
systemctl daemon-reload
sudo systemctl enable sootballs_robot_announcement.service
sudo systemctl restart sootballs_robot_announcement.service || /usr/bin/python2.7 /usr/local/bin/robot_announcement.py &

###### Other Stuff ######

touch /etc/profile.d/sootball.sh
chmod ug+rwx /etc/profile.d/sootball.sh
chown $RR_USER:$RR_USER /etc/profile.d/sootball.sh

cat > /etc/profile.d/sootball.sh <<- EOM
export HOSTNAME=$(echo $(hostname) | tr '-' '_') # Remove underscore characters from the hostname because this will cause a problem with node names that use the hostname in ROS
export SOOTBALLS_MAP=${SOOTBALLS_MAP:-${1:-sagawa}}
export ROS1_WS_PATH=${ROS1_WS_PATH:-${RR_HOME}/ws}
export ROS1_DISTRO=${ROS1_DISTRO:-melodic}
EOM

# Do not need to login to own computer
systemctl start ssh || service ssh start

# Fix ownership
chown $RR_USER:$RR_USER /home/$RR_USER /home/$RR_USER/.ssh /home/$RR_USER/.ssh/authorized_keys >> /dev/null
chmod 755 /home/$RR_USER && chmod 700 /home/$RR_USER/.ssh && chmod 600 /home/$RR_USER/.ssh/authorized_keys >> /dev/null

if [ ! -f "$RR_HOME/.ssh/id_rsa.pub" ]; then
    yes y | sudo -u $RR_USER ssh-keygen -f /home/$RR_USER/.ssh/id_rsa -q -t rsa -N '' > /dev/null
fi
# TODO remove hardcoded password (many places in this repo)
sudo -u $RR_USER sshpass -p "airborne" ssh-copy-id -o StrictHostKeyChecking=no localhost

# set alias for supervisorctl
if ! (grep -q "alias controlctl" $RR_HOME/.bashrc 2> /dev/null); then
      echo "alias controlctl='supervisorctl -c /etc/sootballs_supervisor.conf'" >> $RR_HOME/.bashrc
fi

echo -ne "Device setup successfully for sootballs control.\nSource bashrc and do 'sudo controlctl status' to see running processes.\n"

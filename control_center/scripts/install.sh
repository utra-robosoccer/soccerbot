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
ROBOT_USER=${SUDO_USER:-$USER}
ROBOT_HOME=/home/$ROBOT_USER
ROOT_DIR=$ROBOT_HOME/soccer

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
server.eROBOTorlog             = "/var/log/lighttpd/eROBOTor.log"
server.pid-file             = "/var/run/lighttpd.pid"
server.username             = "www-data"
server.groupname            = "www-data"
server.port                 = "8000"
EOM
systemctl start lighttpd || sudo service lighttpd start

rm -rf /var/log/soccer/supervisor
mkdir -p /var/log/soccer/supervisor
chown -R $ROBOT_USER:$ROBOT_USER /var/log/soccer/supervisor
rm -rf $ROBOT_HOME/soccer/supervisor
mkdir -p $ROBOT_HOME/soccer/supervisor
chown -R $ROBOT_USER:$ROBOT_USER $ROBOT_HOME/soccer/
rm -rf /var/log/soccer_supervisord.log
rm -rf /tmp/supervisor.sock
rm -rf /tmp/soccer_supervisord.pid

###### Install and Start soccer Supervisor ######

cat > /etc/soccer_supervisor.conf <<- EOM
; supervisor config file for control center.

[unix_http_server]
file = /tmp/supervisor.sock
chown = $ROBOT_USER

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
user = $ROBOT_USER

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[supervisorctl]
serverurl = unix:///tmp/supervisor.sock

[include]
files = $ROBOT_HOME/soccer/supervisor/*.conf
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
rm -f /var/log/soccer_supervisord.log
touch /var/log/soccer_supervisord.log
chown -R $ROBOT_USER:$ROBOT_USER /var/log/soccer_supervisord.log

# Restart daemon
systemctl daemon-reload
systemctl enable soccer_supervisor
systemctl restart soccer_supervisor || /usr/bin/supervisord -n -c /etc/soccer_supervisor.conf &

# Create link to the soccer directory
rm -f ${ROOT_DIR}/config.yaml
sudo -u $ROBOT_USER ln -s ${SCRIPTPATH}/../config/config.yaml ${ROOT_DIR}/config.yaml

###### Install and Robot Announcement ######

cp -f ${SCRIPTPATH}/robot_announcement.py /usr/local/bin/robot_announcement.py
chmod +x /usr/local/bin/robot_announcement.py

cat > /etc/systemd/system/soccer_robot_announcement.service <<- EOM
[Unit]
Description=soccer Robot Annoucement Service
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
sudo systemctl enable soccer_robot_announcement.service
sudo systemctl restart soccer_robot_announcement.service || /usr/bin/python2.7 /usr/local/bin/robot_announcement.py &

###### Other Stuff ######

touch /etc/profile.d/sootball.sh
chmod ug+rwx /etc/profile.d/sootball.sh
chown $ROBOT_USER:$ROBOT_USER /etc/profile.d/sootball.sh

# Do not need to login to own computer
systemctl start ssh || service ssh start

# Fix ownership
chown $ROBOT_USER:$ROBOT_USER /home/$ROBOT_USER /home/$ROBOT_USER/.ssh /home/$ROBOT_USER/.ssh/authorized_keys >> /dev/null
chmod 755 /home/$ROBOT_USER && chmod 700 /home/$ROBOT_USER/.ssh && chmod 600 /home/$ROBOT_USER/.ssh/authorized_keys >> /dev/null

if [ ! -f "$ROBOT_HOME/.ssh/id_rsa.pub" ]; then
    yes y | sudo -u $ROBOT_USER ssh-keygen -f /home/$ROBOT_USER/.ssh/id_rsa -q -t rsa -N '' > /dev/null
fi
# TODO remove hardcoded password (many places in this repo)
sudo -u $ROBOT_USER sshpass -p "soccer" ssh-copy-id -o StrictHostKeyChecking=no localhost

# set alias for supervisorctl
if ! (grep -q "alias controlctl" $ROBOT_HOME/.bashrc 2> /dev/null); then
      echo "alias controlctl='supervisorctl -c /etc/soccer_supervisor.conf'" >> $ROBOT_HOME/.bashrc
fi

echo -ne "Device setup successfully for soccer control.\nSource bashrc and do 'sudo controlctl status' to see running processes.\n"

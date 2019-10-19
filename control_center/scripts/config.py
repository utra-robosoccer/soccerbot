import os
import sys
import subprocess

PATH = os.path.join(os.getcwd() + "/../../soccerbot/launch/modules/")
#PATH = "home/anthony/catkin_ws/src/soccer_ws/soccerbot/launch/modules/"

# Access directory with sudo
#uid = int(os.environ.get("SUDO_UID"))
#gid = int(os.environ.get('SUDO_GID'))
#os.chown("/etc/supervisor/conf.d", 1000, 1000)

#subprocess.call(['sudo','python'] + sys.argv)
#sys.exit()
#os.execvp('sudo', ['sudo','python'] + sys.argv)

if not os.path.exists("/etc/supervisor/conf.d/" + "logfiles"):
    os.chdir("/etc/supervisor/conf.d")
    #os.mkdir("logfiles")
    os.system("sudo mkdir logfiles")
#i = 0

#print(os.listdir(PATH))
print(PATH)
for file in os.listdir(PATH):
    print(PATH)
#    i += 1
    filename = (str(file).split("."))
    #print("FILE:" + file +"\n")
    #print(os.getcwd())
    #print(PATH)
    print(filename)
    extension = filename[1]
    if extension == "launch":
        name = filename[0]
        if not os.path.exists("/etc/supervisor/conf.d/logfiles/" + name + "/_log"):
            os.chdir("/etc/supervisor/conf.d/logfiles")
            os.system("sudo mkdir " + name + "_log")
            #os.system("sudo touch /etc/supervisor/conf.d/" + name + ".conf")                                                                                                                                                                                                             "user=www-data" > "/etc/supervisor/conf.d/" + name + ".conf"'')
            #os.system("sudo echo '[program:' + name + ']\ncommand='"
            #          "+ PATH + str(file) + '\ndirectory=' + PATH "
            #          "+ '\nautostart=false\nautorestart=true\nstartretires=3\nstderr_logfile='"
            #          "+ PATH + 'logfiles' + name + '_log' + name + '.err.log\nstdout_logfile=' "
            #          "+ PATH + 'logfiles' + name + '_log' + name + '.out.log\nuser=www-data\n' > /etc/supervisor/conf.d/logfiles/" + name + ".conf")
            with open("/etc/supervisor/conf.d/" + name + ".conf", "w+") as f:
                f.write("[program:" + name + "]\ncommand="
                + PATH + str(file) + "\ndirectory=" + PATH
                + "\nautostart=false\nautorestart=true\nstartretires=3\nstderr_logfile="
                + "/etc/supervisor/conf.d/" + "logfiles/" + name + "_log/" + name + ".err.log\nstdout_logfile="
                + "/etc/supervisoonf.d/" + "logfiles/" + name + "_log/" + name + ".out.log\nuser=www-data\n")

with open("/etc/supervisor/supervisord.conf", "a") as f:
    server = False
    
    for line in f:
        if line == "[inet_http_server]":
            server = True
    if server == False:
        f.write("\n[inet_http_server]\nport = 127.0.0.1:9001")
        print("Access server at: 127.0.0.1:9001")
import os

PATH = os.path.join(os.getcwd() + "/../../soccerbot/launch/modules/")

if not os.path.exists("/etc/supervisor/conf.d/" + "logfiles"):
    os.chdir("/etc/supervisor/conf.d")
    os.system("sudo mkdir logfiles")

print(PATH)
for file in os.listdir(PATH):
    filename = (str(file).split("."))
    extension = filename[1]
    if extension == "launch":
        name = filename[0]
        if not os.path.exists("/etc/supervisor/conf.d/logfiles/" + name + "/_log"):
            os.chdir("/etc/supervisor/conf.d/logfiles")
            os.system("sudo mkdir " + name + "_log")
            with open("/etc/supervisor/conf.d/" + name + ".conf", "w+") as f:
                f.write("[program:" + name + "]\ncommand="
                + PATH + str(file) + "\ndirectory=" + PATH
                + "\nautostart=false\nautorestart=true\nstartretires=3\nstderr_logfile="
                + "/etc/supervisor/conf.d/" + "logfiles/" + name + "_log/" + name + ".err.log\nstdout_logfile="
                + "/etc/supervisor/conf.d/" + "logfiles/" + name + "_log/" + name + ".out.log\nuser=www-data\n")
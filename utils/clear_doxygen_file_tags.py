#!/usr/bin/python3

import fileinput
import os
import re
import sys

def print_log(log):
    for line in log:
        print(line)

walkdir = sys.argv[1]

print("Clearing @file tags found under directory tree {}...".format(walkdir))

for root, dirs, files in os.walk(walkdir):
    for f in files:
        fp = root + os.sep + f
        print("Reading path {}:".format(fp))
        replace_log = []
        with fileinput.input(fp, inplace=1, backup=".bak") as f:
            for line in f:
                line_prev = line.rstrip()
                line = re.sub(r"^ *\* *@file.*", "  * @file", line.rstrip())
                print(line)
                if line_prev != line:
                    replace_log.append("  Replaced \"{}\" with \"{}\".".format(line_prev, line))
        print_log(replace_log)
        print("Wrote backup at: {}.bak".format(fp))


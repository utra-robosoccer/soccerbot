import argparse
import glob
import os

parser = argparse.ArgumentParser()
parser.add_argument("--folder", type=str, default="./")
args = parser.parse_args()

os.chdir(args.folder)

for stl_fileName in glob.glob("*.stl"):
    conversion_command = "meshlabserver -i " + stl_fileName + " -o " + stl_fileName[:-3] + "dae"
    os.system(conversion_command)

for stl_fileName in glob.glob("*.STL"):
    conversion_command = "meshlabserver -i " + stl_fileName + " -o " + stl_fileName[:-3] + "dae"
    os.system(conversion_command)

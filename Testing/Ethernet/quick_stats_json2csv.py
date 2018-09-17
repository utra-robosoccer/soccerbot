import json
import sys

# Convert a quick_stats json file to csv. A header labels the fields.
# May be useful for analysis in Excel.

filenames = sys.argv[1:]

for filename in filenames:
    print("Converting file {} to csv...".format(filename))
    json_data = None
    csv_filename = filename + ".csv"
    print("Output file: {}".format(csv_filename))
    with open(filename, "r") as json_file:
        json_data = json.load(json_file)
    with open(csv_filename, "w") as csv_file:
        csv_file.write("size,sum,average,median,std_dev,max,min,times\n")
    with open(csv_filename, "a") as csv_file:
        for size, results in json_data.items():
            csv_file.write("{},{},{},{},{},{},{},{}\n".format(size, results["sum"], results["average"], results["median"], results["std_dev"], results["max"], results["min"], ",".join(results["the_times"]))
    print("Done for file {}".format(csv_filename))


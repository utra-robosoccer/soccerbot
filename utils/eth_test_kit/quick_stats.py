import json
import numpy
import sys

data_filenames = sys.argv[1:]

def print_quick_results(quick_results, filename):
    print("Quick stats for: {}".format(filename))
    for size, results in quick_results.items():
        print("-- {} bytes message size:\n".format(size)
                + "     sum: {:.2e},     ".format(results["sum"])
                + " average: {:.2e}, ".format(results["average"])
                + " median: {:.2e},\n".format(results["median"])
                + "     std_dev: {:.2e}, ".format(results["std_dev"])
                + " max: {:.2e},     ".format(results["max"])
                + " min: {:.2e}    ".format(results["min"]))

for data_filename in data_filenames:
    with open(data_filename, "r") as data_file:
        json_data = json.load(data_file)

    test_name = json_data["name"]
    message_sizes = [int(s) for s in json.loads(json_data["config"]["message_sizes"])]

    results = {}
    for size in message_sizes:
        results[str(size)] = [[float(time) for time in test["times"].split(",")] for test in json_data["tests"] if test["msg_size"] == size]

    # Flatten the results arrays - don't care if messages sent one after the other immediately without yielding (num_in_sequence), just aggregate the data
    results_flat = {}
    for size, times_list_of_lists in results.items():
        results_flat[size] = [i for j in times_list_of_lists for i in j]

    quick_results = {}
    for size, times in results_flat.items():
        times_array = numpy.array(times)
        quick_result = {"the_times": times,
                        "sum": numpy.sum(times_array),
                        "average": numpy.average(times_array),
                        "median": numpy.median(times_array),
                        "std_dev": numpy.std(times_array),
                        "max": numpy.amax(times_array),
                        "min": numpy.amin(times_array)}
        quick_results[size] = quick_result

    with open(data_filename + "-quick_result.json", "w") as results_file:
        json.dump(quick_results, results_file)
    
    print_quick_results(quick_results, data_filename)

import matplotlib.pyplot as plt
import numpy
import json
import sys

# TODO: use re to properly format filenames

filenames = sys.argv[1:]

def nearest_decimal_up(num):
    decimal_up = pow(10, -15) # Unlikely to get values lower than this
    while decimal_up < num:
        decimal_up *= 10
    return decimal_up

for filename in filenames:
    json_data = {}
    with open(filename, "r") as json_file:
        json_data = json.load(json_file)
    
    # Collect the x values there is a measurement for
    y_collected = {}
    x_collected = numpy.array([int(k) for k, v in json_data.items()])
    y_collected["average"] = numpy.array([v["average"] for k, v in json_data.items()])
    y_collected["max"] = numpy.array([v["max"] for k, v in json_data.items()])

    # Normalize the x axis according to lowest unit
    x_normalized = []
    unit = 0
    while unit < numpy.amax(x_collected) + 1:
        x_normalized.append(int(unit))
        unit += 1

    # Generate the y values based on the x coordinate existing in measured values
    y = {}
    y["average"] = numpy.array([json_data[str(k)]["average"] if k in x_collected else numpy.nan for k in x_normalized])
    y["std_dev"] = ([json_data[str(k)]["std_dev"] if k in x_collected else numpy.nan for k in x_normalized])
    y["max"] = numpy.array([json_data[str(k)]["max"] if k in x_collected else numpy.nan for k in x_normalized])

    plt.figure()

    plt.subplot(211)

    lines = plt.plot(x_normalized, y["average"])
    plt.errorbar(x_normalized, y["average"], y["std_dev"], ecolor='k', elinewidth=1.5, capsize=3)
    plt.setp(lines, color='b', marker='.', ms=5)
    plt.xlabel('message size (bytes)')
    plt.ylabel('average echo time (seconds)')
    plt.title('Average echo time - ' + filename)
    plt.axis([0, numpy.amax(x_collected), 0, nearest_decimal_up(numpy.amax(y_collected["average"]))])

    plt.subplot(212)

    lines = plt.plot(x_normalized, y["max"])
    plt.setp(lines, color='r', marker='.', ms=5)
    plt.xlabel('message size (bytes)')
    plt.ylabel('max echo time (seconds)')
    plt.title('Average echo time - ' + filename)
    plt.axis([0, numpy.amax(x_collected), 0, nearest_decimal_up(numpy.amax(y_collected["max"]))])

    print (numpy.amax(y_collected["max"]))
    plt.suptitle('Ethernet echo test statistics')
    plt.show()

    # TODO: add subplot for max, subplot for distribution


import matplotlib.pyplot as plt
import math
import numpy
import json
import sys

# TODO: use re to properly format filenames
# TODO: use another window drawing tool instead of matplotlib.pyplot to have scrollable window of plots

filenames = sys.argv[1:]

def nearest_decimal_up(num):
    return round(num, -int(math.ceil(math.log10(abs(num)))))

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
    y["times"] = [numpy.array(json_data[str(k)]["the_times"]) for k in x_collected]

    plt.figure(figsize=(4, 8), dpi=80)

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
    
    plt.subplots_adjust(left  = 0.125, right = 0.9, bottom = 0.1, top = 0.9, wspace = 0.2, hspace = 0.360)
    plt.suptitle('Ethernet echo test statistics')
    
    plt.figure(figsize=(4, x_collected.size), dpi=80)
    for i in range(x_collected.size):
        plt.subplot(x_collected.size, 1, i + 1)
        plt.hist(y["times"][i], 50, density=False, facecolor='g', alpha=0.75)
        plt.title('Histogram of echo times for message size ' + str(x_collected[i]) + ' bytes')
        plt.xlabel('time (seconds)')
        plt.ylabel('count')

    plt.subplots_adjust(left  = 0.125, right = 0.9, bottom = 0.1, top = 0.9, wspace = 0.2, hspace = 0.360)
    plt.suptitle('Echo times histograms')

plt.show()

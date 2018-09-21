import matplotlib.pyplot as plt
import numpy
import json
import sys

# TODO: use re to properly format filenames

filenames = sys.argv[1:]

for filename in filenames:
    json_data = {}
    with open(filename, "r") as json_file:
        json_data = json.load(json_file)

    x = [int(k) for k, v in json_data.items()]
    y = [v["average"] for k, v in json_data.items()]
    x_a = numpy.array(x)
    y_a = numpy.array(y)

    # Normalize the x axis
    xi = range(numpy.amax(x_a) + 1)
    y_n = [json_data[str(k)]["average"] if k in x else numpy.nan for k in xi]
    y_err = [json_data[str(k)]["std_dev"] if k in x else numpy.nan for k in xi]

    xi_a = numpy.array(xi)
    y_n_a = numpy.array(y_n)
    y_err_a = numpy.array(y_err)

    
    lines = plt.plot(xi_a, y_n_a)
    plt.errorbar(xi_a, y_n_a, y_err_a, None, '', 'k', 1.5, 3)
    plt.setp(lines, color='b', marker='.', ms=5)
    plt.xlabel('message size (bytes)')
    plt.ylabel('average echo time (seconds)')
    plt.title(filename)
    plt.axis([0, numpy.amax(x_a), 0, numpy.amax(y_a) * 1.1])
    plt.show()

    # TODO: add subplot for max, subplot for distribution


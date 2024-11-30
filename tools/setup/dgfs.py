# with is like your try .. finally block in this case
with open("rosdep_2.txt", "r") as file:
    # read a list of lines into data
    data = file.readlines()

for idx, line in enumerate(data):
    print(line.split(" "))
    if len(line.split(" ")) > 1:
        data[idx] = line.split(" ")[0] + "\n"
print(data)
# print "Your name: " + data[0]
#
# # now change the 2nd line, note that you have to add a newline
# data[1] = 'Mage\n'
#
# # and write everything back
with open("rosdep_2.txt", "w") as file:
    file.writelines(data)

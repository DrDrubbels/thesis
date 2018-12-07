import json
import numpy as np
import matplotlib.pyplot as plt

def interpret(filename):
	my_file = open(filename,"r")
	raw_data = my_file.read()
	parsed_data = json.loads(raw_data)
	extension_data = parsed_data[1]
	meta_data = parsed_data[0]
	x = range(len(extension_data))
	beta_F = meta_data[0]
	print(str(beta_F))
	plt.plot(x,extension_data)
	plt.show()

interpret("31.dat")
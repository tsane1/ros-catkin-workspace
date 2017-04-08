'''
Author: Matthew Lamare
Verision: 1.0
Params:
	- theMap: The input map that will be padded
	- padding: the width that needs to be padded in length units
'''
def dilateByK(theMap, padding):
	paddingSquares = padding / theMap.mapMetaData.resolution
	for i in range(0, paddingSquares):
		theMap = dilate(theMap)
	return theMap

def dilate(theMap):
	for row in len(theMap):
		for node in len(row):
			if node.data == 100:
				#Four connect
				if (row > 0 and theMap[row - 1][node] == 0):
					theMap[row - 1][node] = 100
				if (row > 0 and theMap[row][node - 1] == 0):
					theMap[row][node - 1] = 100
				if (row > 0 and theMap[row + 1][node] == 0):
					theMap[row + 1][node] = 100
				if (row > 0 and theMap[row][node + 1] == 0):
					theMap[row][node + 1] = 100
				#Eight Connect
				if (row > 0 and theMap[row - 1][node - 1] == 0):
					theMap[row - 1][node - 1] = 100
				if (row > 0 and theMap[row + 1][node - 1] == 0):
					theMap[row + 1][node - 1] = 100
				if (row > 0 and theMap[row + 1][node + 1] == 0):
					theMap[row + 1][node + 1] = 100
				if (row > 0 and theMap[row - 1][node + 1] == 0):
					theMap[row - 1][node + 1] = 100

	return theMap

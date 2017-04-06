"""
A neat little wrapper for callback values

Author:
	- Tanuj Sane

Since:
	- 4/5/2017

Version:
	- 1.0 Initial commit
"""

"""
A wrapper for a variable populated by a callback function
"""
class AsyncValue:
	def __init__(self, value):
		self.__value = value
		self.__locked = True
	
	def lock(self):
		self.__locked = True	

	def unlock(self):
		self.__locked = False

	def unwrap(self):
		if not self.__locked: return self.__value
		else: raise NameError 
		

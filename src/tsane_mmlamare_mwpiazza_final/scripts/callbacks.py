#!/usr/bin/env python

"""
A neat little encapsulation of callback-populated values

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
	"""
	Constructor
	"""
	def __init__(self):
		self.__value = None
		self.__locked = True
	
	"""
	Test if the value is ready to use

	Returns:
		- False if the value is usable, True otherwise
	"""	
	def is_locked(self):
		return self.__locked

	"""
	Release the lock, allow the value to be accessed
	"""
	def unlock(self):
		self.__locked = False

	"""
	Releases the value if the lock has been released

	Raises:
		- NameError if the value is locked
	"""
	def unwrap(self):
		if not self.__locked: return self.__value
		else: raise NameError

	"""
	Replace the inner value

	Paramters:
		- value {UNTYPED} New value to wrap
	"""
	def wrap(self, value):
		self.__value = value

"""
Sample usage
"""

def callback(msg):
	av.wrap(msg)
	av.unlock()

def main():
	global av
	av = AsyncValue()

	try:
		print 'Trying to unwrap...'
		av.unwrap()
	except NameError:
		print 'Unwrap failed, unlocking.'
		callback('Unwrap worked!')
		print av.unwrap()

# main()

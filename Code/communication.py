import serial
import struct
import numpy as np
import time

class DownLink:

	def __init__(self):
		self.port=serial.Serial(port='/dev/ttyUSB0',baudrate=115200)		# Define blocking serial port @ 115200 baudrate
		#self.port=serial.Serial(port='COM21',baudrate=115200)
		print("connected to: " + self.port.portstr)
		self.message=None
		

	@staticmethod
	def isValid(byteArray,checkSum):
		check=0
		for ch in byteArray[0:8]:
			check=check+ord(ch)
		check = check&0xFF
		if ord(checkSum)==check:
			return True
		else:
			return False

	def sendValues(self,value1=0,value2=0,value3=0,value4=0):
		#self.port.write(DownLink.byteMap["START_TX"])
		#print "F",value1,value2,value3,value4
		txMessage=struct.pack('<f',value1)+struct.pack('<f',value2)+struct.pack('<f',value3)+struct.pack('<f',value4)
		time.sleep(0.1)
		for bytes in txMessage:
			self.port.write(chr(ord(bytes)))
		chk=DownLink.calcCheckSum(txMessage)
		self.port.write(chk)

	@staticmethod
	def calcCheckSum(byteArray):
		check=0
		for bytes in byteArray:
			check=check+ord(bytes)
		return chr(check&0xFF)

	def release(self):
		self.port.close()


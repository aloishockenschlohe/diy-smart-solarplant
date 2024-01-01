#! /usr/bin/python3

# Program: steering.py
# Purpose: Controlling a small diy smart solar power plant (see https://github.com/aloishockenschlohe/diy-smart-solarplant)

#
# Should work with:
#
# - DPM8605 (see f. e. https://joy-it.net/en/products/JT-DPM8605)
# - DPM8608 (search on ebay)
# - DPM8616 (search on ebay)
# - DPM8624 (see f. e. https://joy-it.net/en/products/JT-DPM8624)

import sys
import serial, time
import os, stat
from os.path import exists
from os import access, R_OK, W_OK

from subprocess import Popen, PIPE
import subprocess
import decimal

#----- variables

ver="240101"			# version of this script

# DPM86 communication
# ===================
# The DPM86 power supply unit is controlled via the "RS485" protocol over a physical connection
# between the computer and the power supply unit (cable with two wires, e.g. bell wire).
#
# On the RPi, the corresponding pins on the GPIO strip can be used or a corresponding interface can be
# added with a USB-to-RS485 adapter (these adapters cost a few €). Either way - at the end of the day,
# the device via which the power supply unit can be accessed must be entered here.
#
# -> /dev/ttyUSBx - if you use a USB-to-RS485 adapter 
# -> /dev/ttyAMA0 - if you use the PL011-UART from the Raspberry (untested!)
# -> /dev/ttyS0 - the mini UART from the Raspberry (untested!)
#
# Good sources (in German):
# - https://www.raspberry-pi-geek.de/ausgaben/rpg/2020/02/serielle-kommunikation-ueber-rs-485-mit-dem-raspberry-pi/
# - https://www.raspberry-pi-geek.de/ausgaben/rpg/2019/12/serielle-kommunikation-ueber-rs-232-rs-485-teil-1/
#
tty="/dev/ttyUSB1"		# The device via this script is talking to the power supply.

# We read the power consumption from a volkszaehler.org instance in the local network via the external tool 'wget'.
#
VZ_IP="192.168.178.20"								# IP address of the instance - please adopt this setting to your needs
VZ_UUID="94f88820-261f-11ec-b8ce-9de20c49f0db"					# UUID of the cannel "power consumption" - please adopt this setting to your needs
VZ_URL="http://" + VZ_IP + "/middleware/data/" + VZ_UUID + ".txt?from=now"	# URL

SRVFILE="/tmp/dpm86_current_power.txt"

ok = 0
error = 1

C_INIT =  2.55

# A 24 V lithium-ion battery actually delivers between 28.8 V (fully charged) and 20.0 V (fully discharged, last gasp, could/will damage the battery)
# The values above apply when no power is drawn from the battery.
# The output voltage V_INIT is set to 25.5 V, so power drain ends when the battery delivers less than 26 V (which corresponds to a state of charge of about 30%).
#
V_INIT = 25.50

# Only the current is permanently adjusted to retrieve the desired power.
# In practice 25.5 volts proved to be a good value for the inverter I used (MI600 from Bosswerk).
# At other voltages I have observed strong and unmotivated fluctuations of the current intensity
# (probably has something to do with the interaction between MPPT algorithm of the inverter and the reaction time of the power supply).
#
# Note: I defined the upper limit of the current to 6 A (which corresponds to a maximum power drain of 6 A * 25.5 V = 152 watts).
# I did this because this value corresponds quite exactly to the basic demand of my apartment during the night and so I can quite relaxed drain 1.7 kWh over 12 hours.
#
C_MAX  =  6.0


# parameter for dpm functions (just for a better reading)
	
			# function "f_output"
p_off="0"		# parameter "off"
p_on="1"		# parameter "on"

			# function "f_const"
p_voltage="0"		# parameter "voltage"
p_current="1"		# parameter "current"

#----- class definition

class dpm86(serial.Serial):

	# supported dpm functions -- see the document "dpm86xx-series-power-supply_simple-communication-protocol.odt/pdf" in this repository
	F_VOLTAGE_SETTING="10"		# R/W: output voltage target
	F_CURRENT_SETTING="11"		# R/W :output current target
	F_OUTPUT="12"			# R/W: output on/off
	F_VOLTAGE="30"			# R/-: output voltage
	F_CURRENT="31"			# R/-: output current
	F_CONST="32"			# R/W: constant current or constant voltage status
	F_TEMPERATURE="33"		# R/-: temperature

	V_MIN = 0			# 00,00 Volt
	V_MAX = 2900		# 29,00 Volt --> should be set automatically, depending on the type (maybe next release)
	CURRENT_MIN = 0			# 0,000 Ampere
	CURRENT_MAX = 9000		# 9,000 Ampere --> should be set automatically, depending on the type (maybe next release)

	READ = "r"			# marks a read access
	WRITE = "w"			# marks a write access

	RETRIES = 5
	RTR_SLEEP = 0.2

	addr = "01"			# Adress of the power supply. Default: 01. Check manual of the power suppy for more information.
	
	def __init__(self, addr="01", port=None, baudrate=9600, timeout=None, inter_byte_timeout=None):
	
		self.addr = addr
		super().__init__(
			port = port,
			baudrate = baudrate,
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			bytesize = serial.EIGHTBITS,
			timeout = timeout,
			inter_byte_timeout = inter_byte_timeout
		)


	# lowlevel - read a value
	#	vread(opcode) -> returns an integer between 0 and 65535 if successful
	#	vread(opcode) -> returns "ERROR..." if unsuccessful
	#
	def vread(self, opcode):
	
		opcode=str(opcode)
		self.cmd=":" + self.addr + self.READ + opcode + "=0" + ",,\n"
		self.bcmd = self.cmd.encode()
		
		# send the command
		success = False
		for cnt in range(self.RETRIES):
#			time.sleep(cnt * self.RTR_SLEEP)
			written = self.write(self.bcmd)
			if written == len(self.bcmd):
				success = True
				break
			time.sleep(cnt * self.RTR_SLEEP)
		if not success: return("ERROR -- vread: Writing '" + self.cmd + "' to dpm86 failed: only " + written + " bytes written.")

		# read the response
		for cnt in range(self.RETRIES):
#			time.sleep(cnt * self.RTR_SLEEP)
			bresponse = self.readline()
			# strip answer
			response = bresponse.decode(errors='replace')
			value = response[7:-3]
			if value.isdigit(): return int(value)
			time.sleep(cnt * self.RTR_SLEEP)

		return ("ERROR -- vread: Communication with dpm86 failed. No or strange response read: " + response)


	# lowlevel - write a value
	#	vwrite(self, opcode) -> returns "ok" on success
	#	vwrite(self, opcode) -> returns "ERROR..." if unsuccessful
	#
	def vwrite(self, opcode, value):

		opcode=str(opcode)
		value=str(value)
		self.cmd=":" + self.addr + self.WRITE + opcode + "=" + value + ",,\n"
		self.bcmd = self.cmd.encode()
	
		# send the command
		success = False
		for cnt in range(self.RETRIES):
#			time.sleep(cnt * self.RTR_SLEEP)
			written = self.write(self.bcmd)
			if written == len(self.bcmd):
				success = True
				break
			time.sleep(cnt * self.RTR_SLEEP)
		if not success: return("ERROR -- vwrite: Writing '" + self.cmd + "' to dpm86 failed: only " + written + " bytes written.")

		# read the response
		for cnt in range(self.RETRIES):
			bresponse = self.readline()
			# strip answer
			response = bresponse.decode(errors='replace')
			response = response[:-2]
			if response == ":" + self.addr + "ok": return("ok")
			time.sleep(cnt * self.RTR_SLEEP)

		return ("ERROR -- vwrite: Communication with dpm86 failed. No or strange response read: " + response)


	# return state or turn the output on or off
	#	output()	-> return if output is on (1) or off (0)
	#	output(1)	-> turn output on
	#	output(0)	-> turn output off
	#
	def output(self, state=None):
	
		if state == None: return(self.vread(self.F_OUTPUT))
		elif state in [p_on, p_off]: return(self.vwrite(self.F_OUTPUT, state))
		else: return("ERROR -- dpm.output: Invalid parameter.")


	# return const, set the const to current
	# 	const() -> return 0 if constant voltage is set
	#	const() -> return 1 if constant current is set
	# 	const(p_voltage) -> set constant voltage
	# 	const(p_current) -> set constant current
	#
	def const(self, state=None):
	
		if state == None: return(self.vread(self.F_CONST))
		elif state in [p_voltage, p_current]: return(self.vwrite(self.F_CONST, state))
		else: return("ERROR -- dpm.const: Invalid parameter.")


	# return temperature
	#	temperature()	-> return temperature (Celsius)
	#
	def temperature(self):
	
		return(self.vread(self.F_TEMPERATURE))


	# transform ticks -> volt
	#	volt(ticks)	-> return voltage (Volt)
	#
	def volt(self, v):
		if type(v) is not int: return("ERROR -- dpm.volt: reading values.")
		return(float(v) / 100)


	# transform ticks -> ampere
	#	ampere(ticks)	-> return current (Ampere)
	#
	def ampere(self, a):
		if type(a) is not int: return("ERROR -- dpm.ampere: reading values.")
		return(float(a) / 1000)


	# return delivered current (Ampere)
	#	current()	-> return delivered current (Ampere)
	#
	def current(self):

		return(self.ampere(self.vread(self.F_CURRENT)))


	# return delivered voltage (Volt)
	#	voltage()	-> return volatage (Volt)
	#
	def voltage(self):
	
		return(self.volt(self.vread(self.F_VOLTAGE)))


	# return delivered power (Watt)
	#
	def power(self):
	
		v = self.volt(self.vread(self.F_VOLTAGE))
		time.sleep(0.2)
		c = self.ampere(self.vread(self.F_CURRENT))
		if type(v) is str or type(c) is str: return("ERROR -- dpm.power: reading values.")
		return(round(float(v) * float(c),2))

	# read and set current and voltage setting, read the power setting
	def setting(self, name, value=None):

		if type(name) is not str: return("ERROR -- dpm.setting: Parameter 'name' is not a string.")
		if name in ['voltage', 'volt', 'v']:
			if value == None: return(self.volt(self.vread(self.F_VOLTAGE_SETTING)))
			if type(value) == str: return("ERROR -- dpm.setting(v): Value for voltage is not an number.")
			value = int(value * 100)
			if value < self.V_MIN or value > self.V_MAX: return("ERROR -- dpm.setting(v): Value for voltage out of range")
			return(self.vwrite(self.F_VOLTAGE_SETTING, value))
		elif name in ['current', 'c', 'ampere', 'amp', 'a']:
			if value == None: return(self.ampere(self.vread(self.F_CURRENT_SETTING)))
			if type(value) == str: return("ERROR -- dpm.setting(a): Value for current is not a number.")
			value = int(value * 1000)
			if value < self.CURRENT_MIN or value > self.CURRENT_MAX: return("ERROR -- dpm.setting(a): Value for current out of range")
			return(self.vwrite(self.F_CURRENT_SETTING, value))
		elif name in ['power', 'p', 'watt', 'w']:
			if value == None:
				v = self.volt(self.vread(self.F_VOLTAGE_SETTING))
				c = self.ampere(self.vread(self.F_CURRENT_SETTING))
				if type(v) is str or type(c) is str: return("ERROR -- dpm.setting(p): Reading values.")
#				print("v = " + str(v) + "--- c = " + str(c))
				return(round(float(v) * float(c),2))
			return("ERROR -- dpm.setting(p): Power expects no additional parameter")
		else:
			return("ERROR -- dpm.setting: Parameter 'name' unknown value.")


#----- Code starts here :-)

# Init of the dpm86 power supply.
def dpm_init():

	print("dpm_init")
	print("========")
	print("Turning dpm off.")
	print(dpm.output(p_off))
	print("Setting voltage to " + str(V_INIT) + " V.")
	print(dpm.setting("v", V_INIT))
	print("Setting current to " + str(C_INIT) + " A.")
	print(dpm.setting("c", C_INIT))
	print("Setting constant current.")
	print(dpm.const(p_voltage))
	print("Turning dpm on.")
	print(dpm.output(p_on))


# Provide information about the delivered and the requested power in file '$srv_file'. Volkszaehler.org may read it.
def statistics():

	print("Writing statistics.")
	my_string = "dpm86powerout " + str(dpm.power()) + "\n" + "dpm86powersetting " + str(dpm.setting("power")) + "\n"

	srvfile = open(SRVFILE, "w")
	srvfile.write(my_string)
	srvfile.close()


# Get the actual power consumption from a volkszaehler.org instance via wget. (int) Watt
def get_vz_power_consumption():

	try:
		output = subprocess.check_output(["wget", "-O", "-", "-q", VZ_URL])
	except subprocess.CalledProcessError as e:
		print("ERROR -- get_vz_power_consumption: wget returned a non-zero exitcode.")
		return(-1000)
	except subprocess.TimeoutExpired as e:
	 	print("ERROR -- get_vz_power_consumption: wget timeout.")
	 	return(-1000)
	
	# The response from volkszaehler.org looks like this: "-34.56 W". We return just the integer part ("-34").
	output = output.decode()
	output = output.replace(" W","")
	return int(float(output))


# Return the power correction. (int) Watt
def get_delta(rounds):

	MAX_DELTA=900

	cnt=1
	while cnt <= rounds:
		# 1st round: Init
		if cnt == 1:
			pc_old = get_vz_power_consumption()	# read the current power consumption from volkszaehler.org
			pc_sum = pc_old				# initialise of the sum of all readings (= current power consumption)
			cnt += 1
			print("Tracking " + str(rounds) + " rounds -- '" + str(pc_old) + "' ", end ="")

		time.sleep(0.2)

		# Calculate the change in power consumption.
		pc_now = get_vz_power_consumption()			# Get current power consumption...
		my_delta = pc_now - pc_old				# ...calculate the delta between current and last power consumption..
		print("(" + str(my_delta) + ") '" + str(pc_now) + "' ", end ="", flush=True)
		
		# If the change is too big...
		if my_delta > MAX_DELTA:
			# ...start a new series,
			print("--- Delta bigger than " + str(MAX_DELTA) + " - restart tracking. ---")
			cnt = 1
		else:
			# ...otherwise add up the measured power consumption
			pc_sum = pc_sum + ( pc_now * cnt )	# Every measurment gets multiplied by its number in the series.
								# In this way, the younger the measured values, the higher they are weighted.
			pc_old = pc_now
			cnt += 1
	
	# To build a kind of average, we have to sum the positions of the measurments.
	divisor = 0
	for x in range(1, cnt):
		divisor = divisor + x

	delta = int(pc_sum / divisor)
	print("= " + str(delta))

	# return the average
	return(delta)


# Set a new current in the dpm86 power supply.
def dpm_adapt(p_from_grid):

	# Read the actual deliverd voltage and current (v, c)...
	v = dpm.voltage()
	c = dpm.current()
	p = round(v * c,2)

	# Read the actual settings 
	v_set = dpm.setting("v")
	c_set = dpm.setting("c")
	p_set = int(v_set) * int(c_set)

	# calculate the new settings (voltage v_target, power p_target and current c_target)
	v_target = V_INIT				# The voltage remains the initial setting.
#	p_target = round((p + p_from_grid) * 1.1, 2)	# The new power is the delivered power plus the power from grid plus 10%
	p_target = round((p + p_from_grid), 2)	# The new power is the delivered power plus the power from grid plus 10%
	c_target = round(p_target / v_target, 2)	# The new current is the new power divided by the new voltage.

	# check p_target
	if p_target < 10: p_target = 10			# If we deliver we deliver at least 10 W.

	# Print some info.
	print("power to/from grid      : %7.2f W" % (p_from_grid))
	print("actual battery supply   : %7.2f W [ %5.2f V with %6.3f A ]" % (p, v, c))
	print("target battery supply   : %7.2f W [ %5.2f V with %6.3f A ]" % (p_target, v_target, c_target))

	# Calculate and check the new current setting for the new power.
	if c_target > C_MAX: c_target = C_MAX		# Check upper limit.
	if c_target < 0: c_target = 0			# Check lower limit.

	# Calculate and print the targets in a human readable format.
	p_target = round(v_target * c_target,2)
	print("corrected battery supply: %7.2f W [ %5.2f V with %6.3f A ]" % (p_target, v_target, c_target))

	# Calculate the difference between the old and the new current settings and check if it is too small.
	c_delta = abs(c_target - c_set)
	if c_delta < 0:
		print("The change in the current setting would be too small (" + str(round(c_delta,2)) + " A) - ignored.")
	else:
		dpm.setting("c", c_target)		# Set the new current 
		# dpm_stable_c(c_target)		# Wait for a stable current.

### Main Code

try:
	dpm = dpm86(
		port = tty,
		baudrate = 9600,
		timeout=0.5, 
		inter_byte_timeout=0.1
	)
	
except dpm.SerialException:
	write("Error opening serial connection wit power supply. Exit.")
	sys.exit()

dpm_init()

measurements=3
while True:
	print("\n--- New cycle ---")
	my_delta = get_delta(measurements)
	if abs(my_delta) < 5: 
		measurements = 20
		continue
	elif abs(my_delta) < 20:
		measurements = 20
	else:
		measurements = 20

	dpm_adapt(my_delta)
	statistics()


dpm.close()

# this is the last line :)

#!/usr/bin/env python3

# IlluminatIR
# Copyright (C) 2021  zwostein
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import argparse
import hid
import os
import sys
import ctypes
import threading
import collections
import queue
import time
import signal


################################################################
# Configuration
default_reportId    =  1
default_channels    =  4
default_updateFreq  = 20

supported_devices = [
	{
		'vendor_id': 0x16c0,
		'product_id': 0x05df,
		'manufacturer_strings': {
			'DesWahnsinnsRan.de': {
				'product_strings': {
					'IlluminatIR': {
						'reportId': 1,
						'channels': 64,
						'updateFreq': 30,
					}
				}
			}
		}
	}
]
################################################################


################################################################
# Autodetected optional functionality
autoconfig_messages = ''
has_uleds = sys.platform.startswith('linux')
try:
	import sacn
	has_sacn = True
except ImportError:
	has_sacn = False
	autoconfig_messages += 'For sACN/E1.31 support install sacn ("pip install --user sacn")\n'
	pass
################################################################


def print_available_devices(infos):
	if not infos:
		print('No devices found!')
		sys.exit(1)
	print('Available devices:')
	for info in infos:
		print('\tDevice @', info['path'], ':')
		print('\t\tManufacturer:', info['manufacturer_string'])
		print('\t\tProduct     :', info['product_string'])
		print('\t\tSerial      :', info['serial_number'])
		print('\t\tExtra Config:', info['extra_config'])


def enumerate_devices(filter_serial=None):
	infos = []
	for supported_device in supported_devices:
		for info in hid.enumerate(supported_device['vendor_id'], supported_device['product_id']):
			if info['manufacturer_string'] not in supported_device['manufacturer_strings']:
				continue
			supported_manufacturer = supported_device['manufacturer_strings'][info['manufacturer_string']]
			if info['product_string'] not in supported_manufacturer['product_strings']:
				continue
			if filter_serial and (info['serial_number'] != filter_serial):
				continue
			info['extra_config'] = supported_manufacturer['product_strings'][info['product_string']]
			infos.append(info)
	return infos


def enumerate_single_device(serial=None):
	infos = enumerate_devices(serial)
	if not infos:
		print('No device found!')
		sys.exit(1)
	if len(infos) != 1:
		print('Multiple devices found!')
		if not serial:
			print('Select a device by adding the "--serial" argument and the device\'s serial number.')
			print_available_devices(infos)
		sys.exit(1)
	return infos[0]


class LedDevice:
	UpdateSingle = collections.namedtuple('UpdateSingle', ['channel', 'value'])
	UpdateRange = collections.namedtuple('Updaterange', ['offset', 'values'])

	def __init__(self, serial):
		self.info = enumerate_single_device(serial)
		self.device = hid.Device(path=self.info['path'])
		extraConfig = self.info.get('extra_config', {})
		self.channels = extraConfig.get('channels', default_channels)
		self.reportId = extraConfig.get('reportId', default_reportId)
		self.updateFreq = extraConfig.get('updateFreq', default_updateFreq)
		self.queue = queue.Queue()
	
	def start(self):
		self.enabled = True
		self._updaterThread = threading.Thread(target=self.__updater)
		self._updaterThread.daemon = True
		self._updaterThread.start()
	
	def stop(self):
		self.enabled = False

	def __updater(self):
		values = bytearray(b'\0'*self.channels)
		lastUpdate = time.perf_counter()
		changed = True
		while self.enabled:
			timeToUpdate = False
			try:
				item = self.queue.get(timeout=1.0/self.updateFreq)
				if type(item) is LedDevice.UpdateSingle:
					if item.channel < self.channels:
						values[item.channel] = item.value
				elif type(item) is LedDevice.UpdateRange:
					to = min(self.channels, item.offset+len(item.values))
					values[item.offset:to] = item.values[:to]
				changed = True
				self.queue.task_done()
			except queue.Empty:
				timeToUpdate = True

			now = time.perf_counter()
			elapsed = now - lastUpdate
			if elapsed > 1.0/self.updateFreq:
				timeToUpdate = True

			if changed and timeToUpdate:
#				print(values)
				report = bytes().join([bytes([self.reportId]), bytes(values)])
				self.device.send_feature_report(report)
				lastUpdate = now
				changed = False

	def set_single(self, channel:int, value:int):
		self.queue.put(LedDevice.UpdateSingle(channel,value))

	def set_range(self, offset:int, values:bytes):
		self.queue.put(LedDevice.UpdateRange(offset,values))

	@property
	def manufacturer(self):
		return self.device.manufacturer

	@property
	def product(self):
		return self.device.product

	@property
	def serial(self):
		return self.device.serial


if has_uleds:
	class Uled:
		LED_MAX_NAME_SIZE = 64
		def __init__(self, device:LedDevice, index:int, color:str=None, function:str=None):
			if not color:
				color=''
			if not function:
				function=''
			self.index = int(index)
			self.device = device
			
			self.name = b'PyLuminate:'+color.encode('ascii')+b':'+function.encode('ascii')
			max_brightness = ctypes.c_uint32(255)
			uleds_user_dev = self.name + b'\0' * (Uled.LED_MAX_NAME_SIZE - len(self.name)) + bytes(max_brightness)

			self.fd = os.open('/dev/uleds', os.O_RDWR|os.O_CLOEXEC)
			os.write(self.fd, uleds_user_dev)

		def start(self):
			self.enabled = True
			self._updaterThread = threading.Thread(target=self.__updater)
			self._updaterThread.daemon = True
			self._updaterThread.start()

		def stop(self):
			self.enabled = False

		def __updater(self):
			print('Uled:', (self.device.product)+' ('+(self.device.serial)+'):', 'Channel ' + str(self.index) + ':', self.name)
			while self.enabled:
				brightness_raw = os.read(self.fd, ctypes.sizeof(ctypes.c_uint32))
				brightness = int.from_bytes(brightness_raw, byteorder=sys.byteorder)
				self.device.set_single(self.index, brightness)


def main():
	global has_uleds
	global has_sacn
	parser = argparse.ArgumentParser(
		description = 'PyLuminate - A device handler for IlluminatIR and similar controllers.',
		epilog = autoconfig_messages
	)
	parser.add_argument('action', choices=['list', 'serve'])
	parser.add_argument('--serial', help='Use device with given serial number when multiple supported devices are connected. Use "list" action to get a list of devices.')
	if has_uleds:
		parser.add_argument('--uled', action='append', metavar='INDEX:COLOR:FUNCTION', help='Provide a Linux Userspace LED named PyLuminate:COLOR:FUNCTION for given device channel index. Can be used multiple times.')
	if has_sacn:
		parser.add_argument('--sacn', action='store_true', help='Start sACN receiver.')
		parser.add_argument('--sacn_bind', metavar='ADDR:PORT', help='Optional address and port (default: :5568) to bind sACN receiver.')
		parser.add_argument('--sacn_universe', metavar='INT', type=int, default=1, help='Universe (default: 1).')
		parser.add_argument('--sacn_multicast', action='store_true', help='Joins multicast group. Might also need something like "ip addr add 239.255.0.1/32 dev eno1 autojoin" in case it doesnt work aut of the box.')
		parser.add_argument('--sacn_map', action='append', metavar='INDEX:DMXCHANNEL[:LENGTH]', help='Map device channels to DMX channels. Can be used multiple times. If not used, every device channel is mapped to the same DMX channel.')

	args = parser.parse_args()

	if args.action == 'list':
		infos = enumerate_devices()
		print_available_devices(infos)
		sys.exit()
	elif args.action == 'serve':
		device = LedDevice(args.serial)
		
		uleds = []
		if has_uleds and args.uled:
			try:
				for uled_def in args.uled:
					index, color, function = (uled_def.split(':', 2) + [None, None])[:3]
					uled = Uled(device, index, color, function)
					uleds.append(uled)
			except Exception as e:
				has_uleds = False
				print('Could not create Linux Userspace LEDs:', e)
		
		if has_sacn and args.sacn:
			if args.sacn_bind:
				address, port = (args.sacn_bind.split(':', 1) + [5568])[:2]
				sacn_receiver = sacn.sACNreceiver(bind_address=address, bind_port=int(port))
			else:
				sacn_receiver = sacn.sACNreceiver()
			if args.sacn_multicast:
				sacn_receiver.join_multicast(args.sacn_universe)
			sacn_map = None
			if args.sacn_map:
				sacn_map = {}
				for element in args.sacn_map:
					deviceIndex, dmxIndex, length = (element.split(':', 2) + [1])[:3]
					sacn_map[int(deviceIndex)] = int(dmxIndex), int(length)

			def sacn_callback(packet):
				if sacn_map:
					for deviceIndex, (dmxIndex, length) in sacn_map.items():
						device.set_range(deviceIndex, packet.dmxData[dmxIndex:dmxIndex+length])
				else:
					device.set_range(0, packet.dmxData[:device.channels])
			sacn_receiver.register_listener('universe', sacn_callback, universe=args.sacn_universe)

			def sacn_callback_availability(universe, changed):
				print('sACN DMX universe ' + str(universe) + ':', changed)
			sacn_receiver.register_listener('availability', sacn_callback_availability)

		device.start()
		if has_uleds:
			for uled in uleds:
				uled.start()
		if has_sacn and args.sacn:
			sacn_receiver.start()

		def signal_handler(sig, frame):
			print('Exiting on received signal', sig)
			if has_sacn and args.sacn:
				sacn_receiver.stop()
			if has_uleds:
				for uled in uleds:
					uled.stop()
			device.stop()
			sys.exit(0)
		signal.signal(signal.SIGINT, signal_handler)

		while True:
			signal.pause()
	
	sys.exit(1)


if __name__ == "__main__":
	main()

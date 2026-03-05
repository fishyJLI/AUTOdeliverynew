'''
  ******************************************************************************
  * Copyright (c)  2016 - 2022, Wuhan Yesense Co.,Ltd .  http://www.yesense.com
  * @file    Yesense_Decode.py
  * @version V1.0.0
  * @date    2022
  * @author  Yesense Technical Support Team  
  * @brief   decode yesense output data with python3.
  ******************************************************************************    
/*******************************************************************************
*
* Code license and exemption information
* Wuhan Yuansheng Innovation Technology Co., Ltd. grants you a non-exclusive copyright license to use all programming code examples. You can do so here.
* Generate similar functions customized to your specific needs. According to any statutory guarantees that cannot be excluded, Wuhan Yuansheng Innovation
* Technology Co., Ltd. and its program developers and suppliers do not provide any express or implied program or technical support (if any).
* The guarantees or conditions contained, including but not limited to the implied guarantees of marketability, fitness for a particular purpose, and non-infringement
* or conditions.
* In any case, Wuhan Yuansheng Innovation Technology Co., Ltd. and its program developers or suppliers are not responsible for the following，
* This is true even when you are told of the possibility of its occurrence: loss or damage of data; direct, special, and collateral
* Or indirect damage, or any consequential economic damage; or profits, business, income, goodwill, or expected savings
* Loss.
* Some jurisdictions do not allow any exclusion or limitation of direct, indirect, or consequential damages, so some or
* All of the above exclusions or limitations may not apply to you.
*
*******************************************************************************/
'''
# -*- encoding:utf-8 -*-
#!/usr/bin/env python3

import serial
import time

#class yesense_product:
#	def __init__(self, model):
yis_out = {'tid':1, 'roll':0.0, 'pitch':0.0, 'yaw':0.0, \
			'q0':1.0, 'q1':1.0, 'q2':0.0, 'q3':0.0, \
			'sensor_temp':25.0, 'acc_x':0.0, 'acc_y':0.0, 'acc_z':1.0, \
			'gyro_x':0.0, 'gyro_y':0.0, 'gyro_z':0.0,	\
			'norm_mag_x':0.0, 'norm_mag_y':0.0, 'norm_mag_z':0.0,	\
			'raw_mag_x':0.0, 'raw_mag_y':0.0, 'raw_mag_z':0.0,	\
			'lat':0.0, 'longt':0.0, 'alt':0.0,	\
			'vel_e':0.0, 'vel_n':0.0, 'vel_u':0.0, \
			'ms':0, 'year': 2022, 'month':8, 'day': 31, \
			'hour':12, 'minute':0, 'second':0,	\
			'samp_timestamp':0, 'dataready_timestamp':0 ,'status':0
			}

#little-endian
YIS_HEADER_1ST	= 0x59
YIS_HEADER_2ND 	= 0x53					

#header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)
PROTOCOL_MIN_LEN 			= 7	

PROTOCOL_TID_LEN 			= 2
PROTOCOL_PAYLOAD_LEN		= 1
PROTOCOL_CHECKSUM_LEN		= 2
PROTOCOL_TID_POS 			= 2
PROTOCOL_PAYLOAD_LEN_POS	= 4
CRC_CALC_START_POS	        = 2
PAYLOAD_POS					= 5

TLV_HEADER_LEN				= 2		#type(1B) + len(1B)

#Definition of the data ID of the output data
sensor_temp_id 	= 0x01				#data_id
acc_id 			= 0x10				
gyro_id 		= 0x20
norm_mag_id		= 0x30
raw_mag_id		= 0x31
euler_id		= 0x40
quaternion_id	= 0x41
utc_id			= 0x50
location_id		= 0x68
speed_id		= 0x70
status_id		= 0x80

#Definition of the len of the output data
sensor_temp_len	= 0x02				#data_id
acc_len			= 0x0C				
gyro_len 		= 0x0C
norm_mag_len	= 0x0C
raw_mag_len		= 0x0C
euler_len		= 0x0C
quaternion_len	= 0x10
utc_len			= 0x0B
location_len	= 0x14
speed_len		= 0x0C
status_len		= 0x01

data_factor_not_raw_mag 	= 0.000001	#The conversion factor of the non-raw magnetic output data from the real data
data_factor_raw_mag			= 0.001		#The conversion factor between the raw magnetic output data and the real data
data_factor_sensor_temp 	= 0.01
data_factor_high_res_loc 	= 0.0000000001
data_factor_alt				= 0.001
data_factor_speed			= 0.001

buf 	= [0] * 512					#Parse buffer
buf_len = 0;						#Parse buffer data length

##--------------------------------------------------------------------------------------##
##                                    Parse the main function                                         	##
##   Input: data --the currently acquired sensor data, num-the length of the currently acquired sensor data                			##
##   Input: info --store the dictionary of real data after the analysis is complete, debug_flg--Debug flag, True--open        			##
##   Return value: True--the resolution was successful, False-the resolution was unsuccessful, including crc check errors, etc.               			##
##--------------------------------------------------------------------------------------##		
def decode_data(data, num, info, debug_flg):
	global buf
	global buf_len
	
	pos = 0	#The location of the data that is currently parsed is recorded
	cnt = 0
	data_len = 0
	check_sum = 0
	
	#Update the data of this read to the parse buffer
	buf[buf_len : buf_len + num] = data[0 : num]
	buf_len += num
	if(debug_flg):
		print('cur_num = ', num)
		print('buf_len = ', buf_len)

	#The total length of the data to be parsed is less than the minimum frame
	if (buf_len < PROTOCOL_MIN_LEN):
		if(debug_flg):
			print('len not enough')		
		return False
		
	cnt = buf_len
	
	#Step 1 -- Find the frame header
	while(cnt > 0):
		if (YIS_HEADER_1ST == buf[pos] and YIS_HEADER_2ND == buf[pos + 1]):
			break;	#Find the header.
		else:
			cnt -= 1
			pos += 1
			
	if(debug_flg):
		print('start pos = ', pos)	
		
	if (cnt < PROTOCOL_MIN_LEN):
		if(debug_flg):
			print('clear_data')	
		clear_data(pos)
		return False
	
	#Find the header of the frame, and the remaining frame length is longer than the length of the smallest frame data, start parsing
	data_len = buf[pos + PROTOCOL_PAYLOAD_LEN_POS]
	if(debug_flg):
		print('payload_len = ', data_len)			
	
	#Comparison between the total length of valid bytes and the length of protocol bytes
	if (PROTOCOL_MIN_LEN + data_len > cnt):
		if(debug_flg):		
			print('protocol len is not enough')	
		clear_data(pos)	
		return False

	if(debug_flg):		
		print('start calc checksum, pos = ', pos)

	#Calculate the checksum
	check_sum = calc_checksum(buf[pos + CRC_CALC_START_POS : buf_len], CRC_CALC_LEN(data_len))
	if (check_sum != (buf[pos + PROTOCOL_CRC_DATA_POS(data_len)] + (buf[pos + PROTOCOL_CRC_DATA_POS(data_len) + 1] << 8))):
		clear_data(pos + data_len + PROTOCOL_MIN_LEN)
		return False
	
	if(debug_flg):
		print('checksum done')		
	info['tid'] = buf[pos + PROTOCOL_TID_POS] + (buf[pos + PROTOCOL_TID_POS + 1] << 8)
	cnt = data_len
	
	#Parse the data
	pos += PAYLOAD_POS;	#payload The data start location
	tlv = {'id':0x10, 'len':0x0C}		#Each packet consists of data_id + data_len + data
	while (data_len > 0 and pos <= buf_len):
		tlv['id'] = buf[pos]
		tlv['len'] = buf[pos + 1]
		ret = parse_data_by_id(tlv, buf[pos + TLV_HEADER_LEN: buf_len], info, debug_flg)
		if(debug_flg):
			print(tlv)
			print('ret = ', ret)
			print('parse data_len %d, pos' %data_len, pos)			
		if(True == ret):
			pos += tlv['len'] + TLV_HEADER_LEN
			data_len -= tlv['len'] + TLV_HEADER_LEN
		else:
			pos += 1
			data_len -= 1
			
	if(debug_flg):			
		print('total len : ', buf_len)
		
	clear_data(pos + PROTOCOL_CHECKSUM_LEN)
	if(debug_flg):
		print('anlysis done, pos = %d, buf_len left %d' %(pos, buf_len));
	
	return True

##--------------------------------------------------------------------------------------##
##                                  Buffer processing function                                      	##
##   Input: clr_len --The length of the data to be cleared, starting from the first data in the buffe算                        			##
##   Return value: none                                                                         	##
##--------------------------------------------------------------------------------------##			
def clear_data(clr_len):
	global buf
	global buf_len
	
	if(0 == clr_len):
		return
		
	buf[0:clr_len] = [0]	
	if (buf_len > clr_len):
		buf[0 : buf_len - clr_len] = buf[clr_len : buf_len]
		buf[clr_len : buf_len] = [0]
		buf_len -= clr_len		
	else:
		buf_len = 0;	

##--------------------------------------------------------------------------------------##
##                                   Data analysis function                                       	##
##   Input: tlv --the header information of the current data to be parsed, and payload-the payload of the current data to be parsed             			##
##   Input: store the dictionary of real data after the analysis is complete, debug_flg--Debug flag, True--open                			##
##   Return value: True --resolved successfully, False --unrecognized data                                   		##
##--------------------------------------------------------------------------------------##				
def parse_data_by_id(tlv, payload, info, debug_flg):
	ret = True
	if (sensor_temp_id == tlv['id'] and sensor_temp_len == tlv['len']):
		if(debug_flg):
			print('data temp')
		info['sensor_temp'] = get_int16_lit(payload) * data_factor_sensor_temp
	elif (acc_id == tlv['id'] and acc_len == tlv['len']):
		if(debug_flg):
			print('data acc')
		info['acc_x'] = get_int32_lit(payload) * data_factor_not_raw_mag		
		info['acc_y'] = get_int32_lit(payload[4:8]) * data_factor_not_raw_mag	
		info['acc_z'] = get_int32_lit(payload[8:12]) * data_factor_not_raw_mag			
	elif (gyro_id == tlv['id'] and gyro_len == tlv['len']):
		if(debug_flg):
			print('data gyro')
		info['gyro_x'] = get_int32_lit(payload) * data_factor_not_raw_mag		
		info['gyro_y'] = get_int32_lit(payload[4:8]) * data_factor_not_raw_mag	
		info['gyro_z'] = get_int32_lit(payload[8:12]) * data_factor_not_raw_mag		
	elif (euler_id == tlv['id'] and euler_len == tlv['len']):
		if(debug_flg):
			print('data euler')
		info['pitch'] = get_int32_lit(payload) * data_factor_not_raw_mag		
		info['roll'] = get_int32_lit(payload[4:8]) * data_factor_not_raw_mag	
		info['yaw'] = get_int32_lit(payload[8:12]) * data_factor_not_raw_mag		
	elif (quaternion_id == tlv['id'] and quaternion_len == tlv['len']):
		if(debug_flg):	
			print('data quaternion')
		info['q0'] = get_int32_lit(payload) * data_factor_not_raw_mag		
		info['q1'] = get_int32_lit(payload[4:8]) * data_factor_not_raw_mag	
		info['q2'] = get_int32_lit(payload[8:12]) * data_factor_not_raw_mag		
		info['q3'] = get_int32_lit(payload[12:16]) * data_factor_not_raw_mag				
	elif (norm_mag_id == tlv['id'] and norm_mag_len == tlv['len']):
		if(debug_flg):	
			print('data norm mag')
		info['norm_mag_x'] = get_int32_lit(payload) * data_factor_not_raw_mag		
		info['norm_mag_y'] = get_int32_lit(payload[4:8]) * data_factor_not_raw_mag	
		info['norm_mag_z'] = get_int32_lit(payload[8:12]) * data_factor_not_raw_mag		
	elif (raw_mag_id == tlv['id'] and raw_mag_len == tlv['len']):
		if(debug_flg):	
			print('data raw mag')
		info['raw_mag_x'] = get_int32_lit(payload) * data_factor_raw_mag		
		info['raw_mag_y'] = get_int32_lit(payload[4:8]) * data_factor_raw_mag	
		info['raw_mag_z'] = get_int32_lit(payload[8:12]) * data_factor_raw_mag		
	elif (location_id == tlv['id'] and location_len == tlv['len']):
		if(debug_flg):	
			print('data location')
		info['alt'] = get_int64_lit(payload) * data_factor_high_res_loc		
		info['longt'] = get_int64_lit(payload[8:16]) * data_factor_high_res_loc	
		info['alt'] = get_int32_lit(payload[16:20]) * data_factor_alt		
	elif (utc_id == tlv['id'] and utc_len == tlv['len']):
		if(debug_flg):	
			print('data utc')
		info['ms'] = get_int32_lit(payload)		
		info['year'] = get_int32_lit(payload)	
		info['month'] = get_int32_lit(payload)		
		info['day'] = get_int32_lit(payload)	
		info['hour'] = get_int32_lit(payload)		
		info['minute'] = get_int32_lit(payload)	
		info['second'] = get_int32_lit(payload)				
	elif (speed_id == tlv['id'] and speed_len == tlv['len']):
		if(debug_flg):	
			print('data speed')
		info['vel_e'] = get_int32_lit(payload) * data_factor_speed		
		info['vel_n'] = get_int32_lit(payload[4:8]) * data_factor_speed	
		info['vel_u'] = get_int32_lit(payload[8:12]) * data_factor_speed		
	elif (status_id == tlv['id'] and status_len == tlv['len']):
		if(debug_flg):	
			print('data fusion status')
		info['status'] = payload[0]			
	else:
		print('unknown data id && len')
		ret = False
		
	return ret

##--------------------------------------------------------------------------------------##
##                       Function to calculate the data length of the CRC to be calculated                                  		##
##    Input: payload_len --the payload length of the current data frame                                       		##
##   Return value: the total data length of the CRC to be calculated                                                      		##
##--------------------------------------------------------------------------------------##		
def CRC_CALC_LEN(payload_len):	#3 = tid(2B) + len(1B) 	
	return (payload_len + PROTOCOL_TID_LEN + PROTOCOL_PAYLOAD_LEN)	

##--------------------------------------------------------------------------------------##
##                       Function to calculate the position of the CRC in the output message data                              		##
##   Input: payload_len --the payload length of the current data frame                                      		##
##   Return value: the starting position of CRC in the current output data message                                          		##
##--------------------------------------------------------------------------------------##			
def PROTOCOL_CRC_DATA_POS(payload_len):
	return (CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))
	
##--------------------------------------------------------------------------------------##
##                                   CRC calculation function                                        	##
##   Inputs: data --data to be CRC calculated, len --length of the CRC data to be calculated                           		##
##   Return value: CRC calculation result, occupies 2 bytes length                                                   		##
##--------------------------------------------------------------------------------------##			
def calc_checksum(data, len):
	check_a = 0x00
	check_b = 0x00

	for i in range(0, len):
		check_a += data[i]
		check_b += check_a
	return ((check_b % 256) << 8) + (check_a % 256)

##--------------------------------------------------------------------------------------##
##                      Convert 8bit data stream to signed 16bit data function                         		##
##   Input: data --the 8bit data stream to be calculated                                                   	##
##   Return value: the converted 16bit conversion result                                                    		##
##--------------------------------------------------------------------------------------##			
def get_int16_lit(data):
	temp = 0

	temp = data[0]
	temp += data[1] << 8
	
	if(temp & 0x8000):
		temp -= 1
		temp = ~temp
		temp &= 0x7FFF
		temp = 0 - temp
	
	return temp

##--------------------------------------------------------------------------------------##
##                      Convert 8bit data stream to signed 32bit data function                           	##
##   Input: data --the 8bit data stream to be calculated                                                   	##
##   Return value: the converted 32bit conversion result                                                    		##
##--------------------------------------------------------------------------------------##			
def get_int32_lit(data):
	temp = 0

	for i in range(0, 4):
		temp += data[i] << (i * 8)

	if(temp & 0x8000_0000):
		temp -= 1
		temp = ~temp
		temp &= 0x7FFFFFFF
		temp = 0 - temp
			
	return temp

##--------------------------------------------------------------------------------------##
##                      Convert 8bit data stream to signed 64bit data function                         		##
##   Input: data --the 8bit data stream to be calculated                                                   	##
##   Return value: the converted 64bit conversion result                                                    		##
##--------------------------------------------------------------------------------------##			
def get_int64_lit(data):
	temp = 0

	for i in range(0, 8):
		temp += data[i] << (i * 8)

	if(temp & 0x8000_0000_0000_0000):
		temp -= 1
		temp = ~temp
		temp &= 0x7FFF_FFFF_FFFF_FFFF
		temp = 0 - temp
			
	return temp
	
##--------------------------------------------------------------------------------------##
##                                   Open serial port function                                          ##
##   Input: port --the serial port number to be opened, string, baud --the baud rate of the serial port to be opened, integer                       ##
##   Return value: serial port example                                                                      ##
##--------------------------------------------------------------------------------------##			
def open_serial(port, baud):
	ser = serial.Serial(port, baud, timeout = 0.002)
	if (ser.is_open):
		print("open %s with baudrate %d suc!" %(port, baud))
	else:
		print("open %s with baudrate %d fail!" %(port, baud))
	return ser
	
##--------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:
		#conn = open_serial('com16', 460800)		#comxx on windows，/dev/ttySCx or /dev/ttyUSBx on linux
		conn=open_serial('/dev/ttyUSB0', 460800)
	except Exception as e:
		print('-------Exception------: ', e)

	while(1):
		data = conn.read_all()
		#data = conn.read(conn.in_waiting)
		num = len(data)
		if(num > 0):
			ret = decode_data(data, num, yis_out, False)
			if(True == ret):
				print('tid %d, pitch %f, roll %f, yaw %f, acc_x %f, acc_y %f, acc_z %f, gyro_x %f, gyro_y %f, gyro_z %f' \
				%(yis_out['tid'], yis_out['pitch'], yis_out['roll'], yis_out['yaw'], \
				yis_out['acc_x'], yis_out['acc_y'], yis_out['acc_z'],\
				yis_out['gyro_x'], yis_out['gyro_y'], yis_out['gyro_z']))
		time.sleep(0.005)   #You need to remove it for windows, otherwise it won't run correctly.

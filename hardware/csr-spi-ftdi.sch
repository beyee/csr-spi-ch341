EESchema Schematic File Version 2  date Вт. 08 июля 2014 00:38:08
LIBS:power
LIBS:device
LIBS:conn
LIBS:components
LIBS:csr-spi-ftdi-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title ""
Date "7 jul 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_B_5S USB1
U 1 1 53BAD805
P 2400 2000
F 0 "USB1" H 2400 1650 60  0000 C CNN
F 1 "USB_B_5S" H 2400 2400 60  0000 C CNN
F 2 "~" H 2400 2000 60  0000 C CNN
F 3 "~" H 2400 2000 60  0000 C CNN
	1    2400 2000
	1    0    0    -1  
$EndComp
$Comp
L FT232RL U3
U 1 1 53BAD13F
P 3500 3600
F 0 "U3" H 3500 4500 60  0000 C CNN
F 1 "FT232RL" H 3900 2600 60  0000 L CNN
	1    3500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4100 7800 4100
Wire Wire Line
	7800 3900 7700 3900
Wire Wire Line
	7700 3500 7800 3500
Wire Wire Line
	7800 3700 7700 3700
Wire Wire Line
	6700 3800 6800 3800
Wire Wire Line
	6700 3600 6800 3600
Connection ~ 2700 2250
Wire Wire Line
	2700 2350 2700 2150
Wire Wire Line
	3150 1650 3800 1650
Wire Wire Line
	2700 1750 2700 1650
Wire Wire Line
	2700 1750 2600 1750
Wire Wire Line
	2600 1950 2700 1950
Connection ~ 7950 1650
Wire Wire Line
	7950 1650 7950 1750
Connection ~ 7650 2250
Wire Wire Line
	7050 2250 7950 2250
Wire Wire Line
	7950 2250 7950 2150
Connection ~ 5850 1650
Wire Wire Line
	5950 1650 5300 1650
Wire Wire Line
	5850 1650 5850 1750
Wire Wire Line
	7700 2950 7700 3050
Wire Wire Line
	5600 3350 5600 3250
Wire Wire Line
	5600 3950 5600 3850
Wire Wire Line
	4500 3300 4400 3300
Wire Wire Line
	4500 3000 4400 3000
Wire Wire Line
	3350 4800 3350 4900
Connection ~ 3350 4900
Connection ~ 3500 4900
Connection ~ 7050 2250
Wire Wire Line
	7650 2250 7650 2150
Connection ~ 4900 2250
Wire Wire Line
	5500 2250 5500 2150
Wire Wire Line
	2450 3300 2550 3300
Wire Wire Line
	2550 2850 2450 2850
Wire Wire Line
	3650 2150 3650 2250
Wire Wire Line
	3650 2250 3300 2250
Connection ~ 7650 1650
Connection ~ 5500 1650
Connection ~ 5600 2800
Wire Wire Line
	5900 2850 5900 2800
Connection ~ 3650 1650
Wire Wire Line
	3650 1650 3650 1750
Wire Wire Line
	8050 2850 7950 2850
Wire Wire Line
	5500 1750 5500 1650
Wire Wire Line
	7650 1750 7650 1650
Wire Wire Line
	7450 2850 7350 2850
Wire Wire Line
	3300 1650 3300 1750
Connection ~ 3300 1650
Wire Wire Line
	2450 2950 2550 2950
Wire Wire Line
	4450 1650 4500 1650
Wire Wire Line
	6600 1650 6650 1650
Wire Wire Line
	3300 2350 3300 2150
Connection ~ 3300 2250
Wire Wire Line
	2450 3200 2550 3200
Wire Wire Line
	4900 2350 4900 2150
Wire Wire Line
	7050 2150 7050 2350
Wire Wire Line
	3200 4800 3200 4900
Wire Wire Line
	3200 4900 3800 4900
Wire Wire Line
	3800 4900 3800 4800
Wire Wire Line
	3500 4800 3500 5000
Wire Wire Line
	3650 4800 3650 4900
Connection ~ 3650 4900
Wire Wire Line
	4400 2900 4500 2900
Wire Wire Line
	4400 3400 4500 3400
Wire Wire Line
	5900 3950 5900 3850
Wire Wire Line
	5900 3350 5900 3250
Wire Wire Line
	7700 3050 7600 3050
Wire Wire Line
	5850 2150 5850 2250
Wire Wire Line
	5850 2250 4900 2250
Connection ~ 5500 2250
Wire Wire Line
	8050 1650 7450 1650
Wire Wire Line
	2700 1850 2600 1850
Wire Wire Line
	2700 2150 2600 2150
Wire Wire Line
	2700 1650 2850 1650
Wire Wire Line
	5600 2800 5600 2850
Wire Wire Line
	2700 2250 2600 2250
Wire Wire Line
	6900 3500 7200 3500
Wire Wire Line
	6900 3700 7200 3700
Wire Wire Line
	7300 3800 7800 3800
Wire Wire Line
	7300 3600 7800 3600
Wire Wire Line
	7800 4200 7700 4200
Wire Wire Line
	7700 4200 7700 4300
Wire Wire Line
	4500 3500 4400 3500
Wire Wire Line
	4400 3200 4500 3200
Wire Wire Line
	6200 3350 6200 3250
Wire Wire Line
	6200 3950 6200 3850
Wire Wire Line
	6200 2850 6200 2800
Wire Wire Line
	6200 2800 6250 2800
Wire Wire Line
	5900 2800 5500 2800
Wire Wire Line
	7400 4000 7800 4000
Text GLabel 7700 3900 0    60   Input ~ 0
VIO
$Comp
L CONN_8 P1
U 1 1 53BACE96
P 8150 3850
F 0 "P1" V 8100 3850 60  0000 C CNN
F 1 "CONN_8" V 8200 3850 60  0000 C CNN
	1    8150 3850
	1    0    0    -1  
$EndComp
Text GLabel 6250 2800 2    60   Input ~ 0
VCC
$Comp
L GND #PWR01
U 1 1 53BACD05
P 6200 3950
F 0 "#PWR01" H 6200 3950 30  0001 C CNN
F 1 "GND" H 6200 3880 30  0001 C CNN
	1    6200 3950
	1    0    0    -1  
$EndComp
$Comp
L LED D3
U 1 1 53BACCC7
P 6200 3050
F 0 "D3" H 6200 3150 50  0000 C CNN
F 1 "LED_PWR" H 6200 2950 50  0000 C CNN
	1    6200 3050
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 53BACCC6
P 6200 3600
F 0 "R7" V 6280 3600 50  0000 C CNN
F 1 "1k" V 6200 3600 50  0000 C CNN
	1    6200 3600
	-1   0    0    1   
$EndComp
Text GLabel 7700 4100 0    60   Input ~ 0
3V3
Text GLabel 7400 4000 0    60   Input ~ 0
1V8
$Comp
L PTC F1
U 1 1 53B543D0
P 3000 1650
F 0 "F1" H 2950 1750 60  0000 C CNN
F 1 "500mA" H 3000 1540 60  0000 C CNN
	1    3000 1650
	1    0    0    -1  
$EndComp
$Comp
L CP1 C6
U 1 1 53B42B3D
P 7950 1950
F 0 "C6" H 8000 2050 50  0000 L CNN
F 1 "10uF" H 8000 1850 50  0000 L CNN
	1    7950 1950
	1    0    0    -1  
$EndComp
$Comp
L CP1 C4
U 1 1 53B42B31
P 5850 1950
F 0 "C4" H 5900 2050 50  0000 L CNN
F 1 "10uF" H 5900 1850 50  0000 L CNN
	1    5850 1950
	1    0    0    -1  
$EndComp
Text GLabel 7600 3050 0    60   Input ~ 0
VIO
Text GLabel 5900 3950 3    60   Input ~ 0
LED_WRITE#
Text GLabel 5600 3950 3    60   Input ~ 0
LED_READ#
Text GLabel 4500 3400 2    60   Input ~ 0
LED_READ#
Text GLabel 4500 3500 2    60   Input ~ 0
LED_WRITE#
Text GLabel 6700 3800 0    60   Input ~ 0
CS#
Text GLabel 6900 3700 0    60   Input ~ 0
CLK
Text GLabel 6900 3500 0    60   Input ~ 0
MISO
Text GLabel 6700 3600 0    60   Input ~ 0
MOSI
Text GLabel 4500 2900 2    60   Input ~ 0
CS#
Text GLabel 4500 3300 2    60   Input ~ 0
CLK
Text GLabel 4500 3200 2    60   Input ~ 0
MISO
Text GLabel 4500 3000 2    60   Input ~ 0
MOSI
Text GLabel 2450 3200 0    60   Input ~ 0
D-
Text GLabel 2450 3300 0    60   Input ~ 0
D+
Text GLabel 2450 2850 0    60   Input ~ 0
VIO
$Comp
L GND #PWR02
U 1 1 53B42553
P 3300 2350
F 0 "#PWR02" H 3300 2350 30  0001 C CNN
F 1 "GND" H 3300 2280 30  0001 C CNN
	1    3300 2350
	1    0    0    -1  
$EndComp
Text GLabel 8050 1650 2    60   Input ~ 0
1V8
Text GLabel 6600 1650 0    60   Input ~ 0
VCC
Text GLabel 4450 1650 0    60   Input ~ 0
VCC
Text GLabel 5500 2800 0    60   Input ~ 0
3V3
$Comp
L GND #PWR03
U 1 1 53B424EE
P 7700 4300
F 0 "#PWR03" H 7700 4300 30  0001 C CNN
F 1 "GND" H 7700 4230 30  0001 C CNN
	1    7700 4300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 53B424E9
P 7050 2350
F 0 "#PWR04" H 7050 2350 30  0001 C CNN
F 1 "GND" H 7050 2280 30  0001 C CNN
	1    7050 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 53B424E4
P 4900 2350
F 0 "#PWR05" H 4900 2350 30  0001 C CNN
F 1 "GND" H 4900 2280 30  0001 C CNN
	1    4900 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 53B424D8
P 3500 5000
F 0 "#PWR06" H 3500 5000 30  0001 C CNN
F 1 "GND" H 3500 4930 30  0001 C CNN
	1    3500 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 53B424D1
P 2700 2350
F 0 "#PWR07" H 2700 2350 30  0001 C CNN
F 1 "GND" H 2700 2280 30  0001 C CNN
	1    2700 2350
	1    0    0    -1  
$EndComp
Text GLabel 2700 1850 2    60   Input ~ 0
D-
Text GLabel 2700 1950 2    60   Input ~ 0
D+
Text GLabel 2450 2950 0    60   Input ~ 0
VCC
Text GLabel 3800 1650 2    60   Input ~ 0
VCC
Text GLabel 5950 1650 2    60   Input ~ 0
3V3
Text GLabel 7350 2850 0    60   Input ~ 0
1V8
Text GLabel 8050 2850 2    60   Input ~ 0
3V3
$Comp
L JUMPER3 JP1
U 1 1 53B41F12
P 7700 2850
F 0 "JP1" H 7750 2750 40  0000 L CNN
F 1 "VIO SELECT JUMPER" H 7700 2950 40  0000 C CNN
	1    7700 2850
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 53B41C9E
P 7050 3800
F 0 "R6" V 7130 3800 50  0000 C CNN
F 1 "1k" V 7050 3800 50  0000 C CNN
	1    7050 3800
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 53B41C8C
P 7450 3700
F 0 "R5" V 7530 3700 50  0000 C CNN
F 1 "1k" V 7450 3700 50  0000 C CNN
	1    7450 3700
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 53B41C86
P 7050 3600
F 0 "R4" V 7130 3600 50  0000 C CNN
F 1 "1k" V 7050 3600 50  0000 C CNN
	1    7050 3600
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 53B41C7F
P 7450 3500
F 0 "R1" V 7530 3500 50  0000 C CNN
F 1 "1k" V 7450 3500 50  0000 C CNN
	1    7450 3500
	0    -1   -1   0   
$EndComp
$Comp
L CP1 C2
U 1 1 53B41C6A
P 3650 1950
F 0 "C2" H 3700 2050 50  0000 L CNN
F 1 "10uF" H 3700 1850 50  0000 L CNN
	1    3650 1950
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 53B41C4F
P 3300 1950
F 0 "C1" H 3350 2050 50  0000 L CNN
F 1 "0.1uF" H 3350 1850 50  0000 L CNN
	1    3300 1950
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 53B41C3E
P 5500 1950
F 0 "C3" H 5550 2050 50  0000 L CNN
F 1 "0.1uF" H 5550 1850 50  0000 L CNN
	1    5500 1950
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 53B41C28
P 7650 1950
F 0 "C5" H 7700 2050 50  0000 L CNN
F 1 "0.1uF" H 7700 1850 50  0000 L CNN
	1    7650 1950
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 53B41C02
P 5600 3600
F 0 "R2" V 5680 3600 50  0000 C CNN
F 1 "470" V 5600 3600 50  0000 C CNN
	1    5600 3600
	-1   0    0    1   
$EndComp
$Comp
L R R3
U 1 1 53B41BFE
P 5900 3600
F 0 "R3" V 5980 3600 50  0000 C CNN
F 1 "470" V 5900 3600 50  0000 C CNN
	1    5900 3600
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 53B41BF3
P 5900 3050
F 0 "D2" H 5900 3150 50  0000 C CNN
F 1 "LED_WRITE" H 5900 2950 50  0000 C CNN
	1    5900 3050
	0    1    1    0   
$EndComp
$Comp
L LED D1
U 1 1 53B41BF0
P 5600 3050
F 0 "D1" H 5600 3150 50  0000 C CNN
F 1 "LED_READ" H 5600 2950 50  0000 C CNN
	1    5600 3050
	0    1    1    0   
$EndComp
$Comp
L LM1117 U1
U 1 1 53B41B4E
P 4900 1800
F 0 "U1" H 4900 2100 60  0000 C CNN
F 1 "LM1117-3.3" H 4950 1550 60  0000 L CNN
F 2 "SOT223" H 4800 2000 60  0000 C CNN
	1    4900 1800
	1    0    0    -1  
$EndComp
$Comp
L LM1117 U2
U 1 1 53B41B49
P 7050 1800
F 0 "U2" H 7050 2100 60  0000 C CNN
F 1 "LM1117-1.8" H 7100 1550 60  0000 L CNN
F 2 "SOT223" H 6950 2000 60  0000 C CNN
	1    7050 1800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
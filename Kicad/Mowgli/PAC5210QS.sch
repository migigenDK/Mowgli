EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 9
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Kicad-PAC-Library:PAC5210QS U?
U 1 1 628A12FC
P 4350 3250
F 0 "U?" H 4225 4693 129 0000 C CNN
F 1 "PAC5210QS" H 4225 4483 129 0000 C CNN
F 2 "" H 4350 3250 50  0001 C CNN
F 3 "" H 4350 3250 50  0001 C CNN
	1    4350 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J13
U 1 1 628A4DE6
P 750 1150
F 0 "J13" H 858 1431 50  0000 C CNN
F 1 "Serial" H 858 1340 50  0000 C CNN
F 2 "" H 750 1150 50  0001 C CNN
F 3 "~" H 750 1150 50  0001 C CNN
	1    750  1150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 628A6656
P 1400 1300
F 0 "#PWR?" H 1400 1150 50  0001 C CNN
F 1 "+5V" H 1415 1473 50  0000 C CNN
F 2 "" H 1400 1300 50  0001 C CNN
F 3 "" H 1400 1300 50  0001 C CNN
	1    1400 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 628A8446
P 1650 1050
F 0 "#PWR?" H 1650 800 50  0001 C CNN
F 1 "GND" H 1655 877 50  0000 C CNN
F 2 "" H 1650 1050 50  0001 C CNN
F 3 "" H 1650 1050 50  0001 C CNN
	1    1650 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1050 950  1050
Wire Wire Line
	1400 1300 1400 1350
Wire Wire Line
	950  1350 1400 1350
Text Label 5900 3100 0    50   ~ 0
UART_TX
Text Label 950  1250 0    50   ~ 0
UART_RX
Text Label 5900 3000 0    50   ~ 0
UART_RX
Text Label 950  1150 0    50   ~ 0
UART_TX
Text HLabel 6700 3100 2    50   Input ~ 0
PAC5210_UART_TX
Wire Wire Line
	5700 3100 6700 3100
Wire Wire Line
	5700 3000 6700 3000
$Comp
L Connector:Conn_01x04_Male J17
U 1 1 628AAD10
P 1950 1150
F 0 "J17" H 2058 1431 50  0000 C CNN
F 1 "SWD" H 2058 1340 50  0000 C CNN
F 2 "" H 1950 1150 50  0001 C CNN
F 3 "~" H 1950 1150 50  0001 C CNN
	1    1950 1150
	1    0    0    -1  
$EndComp
Text Label 2150 1150 0    50   ~ 0
SWD_DA
Text Label 2150 1250 0    50   ~ 0
SWD_CLK
$Comp
L power:GND #PWR?
U 1 1 628AB761
P 2300 1400
F 0 "#PWR?" H 2300 1150 50  0001 C CNN
F 1 "GND" H 2305 1227 50  0000 C CNN
F 2 "" H 2300 1400 50  0001 C CNN
F 3 "" H 2300 1400 50  0001 C CNN
	1    2300 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1400 2300 1350
Wire Wire Line
	2300 1350 2150 1350
$Comp
L power:+5V #PWR?
U 1 1 628ABE44
P 2300 1000
F 0 "#PWR?" H 2300 850 50  0001 C CNN
F 1 "+5V" H 2315 1173 50  0000 C CNN
F 2 "" H 2300 1000 50  0001 C CNN
F 3 "" H 2300 1000 50  0001 C CNN
	1    2300 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1000 2300 1050
Wire Wire Line
	2300 1050 2150 1050
Wire Wire Line
	5900 3300 5700 3300
Text Label 5900 3300 0    50   ~ 0
SWD_DA
Text Label 5900 3200 0    50   ~ 0
SWD_CLK
Wire Wire Line
	5900 3200 5700 3200
Text HLabel 6700 3000 2    50   Input ~ 0
PAC5210_UART_RX
Text HLabel 2300 2400 0    50   Input ~ 0
PAC5210_PC4
Wire Wire Line
	2750 2400 2300 2400
Wire Wire Line
	9000 2350 8900 2350
Wire Wire Line
	8900 2350 8900 2400
Wire Wire Line
	8900 2450 9000 2450
$Comp
L 74xx:74LS366 U?
U 1 1 628E51AF
P 9500 2050
F 0 "U?" H 9500 2931 50  0000 C CNN
F 1 "74LS366" H 9500 2840 50  0000 C CNN
F 2 "" H 9500 2050 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS366" H 9500 2050 50  0001 C CNN
	1    9500 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4700 8900 4700
Wire Wire Line
	8900 4700 8900 4750
Wire Wire Line
	8900 4800 9000 4800
$Comp
L 74xx:74LS366 U?
U 1 1 628ECD7C
P 9500 4400
F 0 "U?" H 9500 5281 50  0000 C CNN
F 1 "74LS366" H 9500 5190 50  0000 C CNN
F 2 "" H 9500 4400 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS366" H 9500 4400 50  0001 C CNN
	1    9500 4400
	1    0    0    -1  
$EndComp
Text HLabel 8800 4750 0    50   Input ~ 0
PAC5210_74366_nOE
Wire Wire Line
	8900 4750 8800 4750
Connection ~ 8900 4750
Wire Wire Line
	8900 4750 8900 4800
Text HLabel 8800 2400 0    50   Input ~ 0
PAC5210_74366_nOE
Wire Wire Line
	8900 2400 8800 2400
Connection ~ 8900 2400
Wire Wire Line
	8900 2400 8900 2450
Text HLabel 2300 2500 0    50   Input ~ 0
PAC5210_PC3
Wire Wire Line
	2750 2500 2300 2500
Text HLabel 6750 4200 2    50   Output ~ 0
PAC5210_PA6
Wire Wire Line
	6750 4200 5700 4200
Text HLabel 6700 2800 2    50   Output ~ 0
PAC5210_PE4
Wire Wire Line
	6700 2800 5700 2800
$EndSCHEMATC

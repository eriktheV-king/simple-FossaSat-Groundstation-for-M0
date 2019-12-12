# simple-FossaSat-Groundstation-for-M0

 * filename simple-FossaSat-Groundstation-for-M0
 * version: 3
 * Vking's FOSSA MODIFIED Ground Station Example
 * by Erikthevking http://lab.vking.earth
 * 
 * this is based on the FOSSA Ground Station Example 
 * but I desactivated the transmitting part of the code
 * so that we do not interfere with the FOSSASAT-1 satellite setup process
 * 
 * to enable send mode, uncomment line :  // printControls(); 
 *                       and uncomment : // process serial command
 *                        in loop function
 * 
 * tested on SAMD-21G18 Adafruit Feather M0 with SX1278 featherwing and SSD1306 128x32 Oled display featherwing
 *                                                                         (Uses pins 20, 21 for SDA, SCL)
 * 
 * added Adafruit Featherwing with SSD1306 Oled 128x32
 * so that we don't need to have a serial monitor all the time.
 * On reception, the display will show a counter counting upwards on every successful reception 
 * on error, errorcode will shop up on display, and error counter will count upwards on every error
 * On succesful reception, received parameters will be shown in serial monitor only



 * FOSSA Ground Station Example
 *
 * Tested on Arduino Uno and SX1278, can be used with any LoRa radio
 * from the SX127x or SX126x series. Make sure radio type (line 21)
 * and pin mapping (lines 26 - 29) match your hardware!
 *
 * References:
 *
 * RadioLib error codes:
 * https://jgromes.github.io/RadioLib/group__status__codes.html
 *
 * FOSSASAT-1 Communication Guide:
 * https://github.com/FOSSASystems/FOSSASAT-1/blob/master/FOSSA%20Documents/FOSSASAT-1%20Comms%20Guide.pdf

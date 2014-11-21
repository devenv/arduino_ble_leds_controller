//"SPI.h/Nordic_nRF8001.h/RBL_nRF8001.h" are needed in every new project
#include <SPI.h>
#include <Nordic_nRF8001.h>
#include <RBL_nRF8001.h>

#include <Servo.h>
#include <SPI.h>
#include <boards.h>
#include "Boards.h"
#include "Adafruit_NeoPixel.h"


#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

#define PIN_CAPABILITY_NONE      0x00
#define PIN_CAPABILITY_DIGITAL   0x01

// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h

byte pin_mode[1];
byte pin_state[1];

// 30 LEDs on OUTPUT pin 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, 8, NEO_GRB + NEO_KHZ800);

Servo servos[MAX_SERVOS];

void setup()
{
	Serial.begin(57600);
	Serial.println("BLE Arduino Slave");

	pin_mode[0] = OUTPUT;
	pin_state[0] = LOW;

	strip.begin();
	strip.show();

	ble_set_name("My Bag");
	ble_begin();
}

static byte buf_len = 0;

void ble_write_string(byte *bytes, uint8_t len)
{
	if (buf_len + len > 20)
	{
		for (int j = 0; j < 15000; j++)
			ble_do_events();

		buf_len = 0;
	}

	for (int j = 0; j < len; j++)
	{
		ble_write(bytes[j]);
		buf_len++;
	}

	if (buf_len == 20)
	{
		for (int j = 0; j < 15000; j++)
			ble_do_events();

		buf_len = 0;
	}
}

void reportPinCapability(byte pin)
{
	byte buf[] = {'P', pin, 0x00};
	byte pin_cap = 0;

	if (IS_PIN_DIGITAL(pin))
		pin_cap |= PIN_CAPABILITY_DIGITAL;

	buf[2] = pin_cap;
	ble_write_string(buf, 3);
}

void reportPinDigitalData(byte pin)
{
  byte state = digitalRead(pin);
  byte mode = pin_mode[pin];
  byte buf[] = {'G', pin, mode, state};         
  ble_write_string(buf, 4);
}

void sendCustomData(uint8_t *buf, uint8_t len)
{
	uint8_t data[20] = "Z";
	memcpy(&data[1], buf, len);
	ble_write_string(data, len + 1);
}

byte queryDone = false;

void loop()
{
	while(ble_available())
	{
		byte cmd;
		cmd = ble_read();
		Serial.write(cmd);

		// Parse data here
		switch (cmd)
		{
			case 'V': // query protocol version
			{
				byte buf[] = {'V', 0x00, 0x00, 0x01};
				ble_write_string(buf, 4);
			}
			break;

			case 'C': // query board total pin count
			{
				byte buf[2];
				buf[0] = 'C';
				buf[1] = 1;
				ble_write_string(buf, 2);
			}
			break;

			case 'M': // query pin mode
			{
				byte pin = ble_read();
				byte buf[] = {'M', pin, pin_mode[pin]}; // report pin mode
				ble_write_string(buf, 3);
			}
			break;

			case 'S': // set pin mode
			{
				byte pin = ble_read();
				byte mode = ble_read();

								if (mode != pin_mode[pin])
				{
					pinMode(pin, mode);
					pin_mode[pin] = mode;

					if (mode == OUTPUT)
					{
//						digitalWrite(pin, LOW);
						pin_state[pin] = LOW;
					}
				}

			
				if ( (mode == INPUT) || (mode == OUTPUT) )
					reportPinDigitalData(pin);
			}
			break;

			case 'G': // query pin data
			{
				byte pin = ble_read();
				reportPinDigitalData(pin);
			}
			break;

			case 'T': // set pin digital state
			{
				byte pin = ble_read();
				byte state = ble_read();

				if(state == LOW) {
					 colorWipe(strip.Color(255, 0, 0), 50); 
				} else {
					 colorWipe(strip.Color(0, 255, 0), 50); 
				}
//				digitalWrite(pin, state);
				reportPinDigitalData(pin);
			}
			break;

			case 'A': // query all pin status
				for (int pin = 0; pin < 1; pin++)
				{
					reportPinCapability(pin);
					if ( (pin_mode[pin] == INPUT) || (pin_mode[pin] == OUTPUT) )
						reportPinDigitalData(pin);
				}

				queryDone = true;
				{
					uint8_t str[] = "ABC";
					sendCustomData(str, 3);
				}

				break;

			case 'P': // query pin capability
			{
				byte pin = ble_read();
				reportPinCapability(pin);
			}
			break;

			case 'Z':
			{
				byte len = ble_read();
				byte buf[len];
				for (int i = 0; i < len; i++)
					buf[i] = ble_read();
				Serial.println("->");
				Serial.print("Received: ");
				Serial.print(len);
				Serial.println(" byte(s)");
				Serial.print(" Hex: ");
				for (int i = 0; i < len; i++)
					Serial.print(buf[i], HEX);
				Serial.println();
			}
		}

		// send out any outstanding data
		ble_do_events();
		buf_len = 0;

		return; // only do this task in this loop
	}

	ble_do_events();
	buf_len = 0;
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait)
{
	for(uint16_t i = 0; i < strip.numPixels(); i++)
	{
		strip.setPixelColor(i, c);
		strip.show();
		delay(wait);
	}
}

void rainbow(uint8_t wait)
{
	uint16_t i, j;

	for(j = 0; j < 256; j++)
	{
		for(i = 0; i < strip.numPixels(); i++)
		{
			strip.setPixelColor(i, Wheel((i + j) & 255));
		}
		strip.show();
		delay(wait);
	}
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
	uint16_t i, j;

	for(j = 0; j < 256 * 5; j++) // 5 cycles of all colors on wheel
	{
		for(i = 0; i < strip.numPixels(); i++)
		{
			strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
		}
		strip.show();
		delay(wait);
	}
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait)
{
	for (int j = 0; j < 10; j++) //do 10 cycles of chasing
	{
		for (int q = 0; q < 3; q++)
		{
			for (int i = 0; i < strip.numPixels(); i = i + 3)
			{
				strip.setPixelColor(i + q, c);  //turn every third pixel on
			}
			strip.show();

			delay(wait);

			for (int i = 0; i < strip.numPixels(); i = i + 3)
			{
				strip.setPixelColor(i + q, 0);      //turn every third pixel off
			}
		}
	}
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait)
{
	for (int j = 0; j < 256; j++)     // cycle all 256 colors in the wheel
	{
		for (int q = 0; q < 3; q++)
		{
			for (int i = 0; i < strip.numPixels(); i = i + 3)
			{
				strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
			}
			strip.show();

			delay(wait);

			for (int i = 0; i < strip.numPixels(); i = i + 3)
			{
				strip.setPixelColor(i + q, 0);      //turn every third pixel off
			}
		}
	}
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
	WheelPos = 255 - WheelPos;
	if(WheelPos < 85)
	{
		return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	else if(WheelPos < 170)
	{
		WheelPos -= 85;
		return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	else
	{
		WheelPos -= 170;
		return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
	}
}

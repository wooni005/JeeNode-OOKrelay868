/// @dir ookRelay2
/// Generalized decoder and relay for 868 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include "decoders.h"

#define DEBUG 0		 // set to 1 to also report results on the serial port
//#define DEBUG_LED 7 // define as pin 1..19 to blink LED on each pin change
//#define PULSE_DEBUG

// RF12 communication settings
#define NODEID 29
#define NETGRP 5

// I/O pin allocations, leave any of these undefined to omit the code for it
#define PIN_868 14		 // AIO1 = 868 MHz receiver

#ifdef PULSE_DEBUG
volatile word buf[300];
byte head, tail;
#endif

VisonicDecoder viso;
EMxDecoder emx;
KSxDecoder ksx;
FSxDecoder fsx;

DecoderInfo di_868[] = {
//	 { 1, "VISO", &viso },
//	 { 2, "EMX", &emx },
	 { 3, "KSX", &ksx },
	 { 4, "FSX", &fsx },
	 { -1, 0, 0 }
};

// State to track pulse durations measured in the interrupt code
volatile word pulse_868;
word last_868; // never accessed outside ISR's

ISR(ANALOG_COMP_vect) {
	 word now = micros();
	 pulse_868 = now - last_868;
	 last_868 = now;
#ifdef PULSE_DEBUG
	 buf[head++] = pulse_868;
#endif
}

// Outgoing data buffer for RF12
byte packetBuffer [20], packetFill;

// Timer to only relay packets up to 10x per second, even if more come in.
MilliTimer sendTimer;

static void setupPinChangeInterrupt () {
	 pinMode(PIN_868, INPUT);
	 digitalWrite(PIN_868, 1);	 // pull-up

	 // enable analog comparator with fixed voltage reference
	 //ACBG=Analog Comparator Bandgap,
	 //ACI=Analog Comparator Interrupt Flag,
	 //ACIE=Analog Comparator Interrupt Enable
	 ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);

	 //ADEN: ADC Disable
	 ADCSRA &= ~ _BV(ADEN);

	 //ACME: 0=Select the input pin to the negative input of the analog comperator
	 ADCSRB |= _BV(ACME);

	 //Select the right ADC input
	 ADMUX = PIN_868 - 14;
}

// Append a new data item to the outgoing packet buffer (if there is room
static void addToBuffer (byte code, const char* name, const byte* buf, byte len) {
#if DEBUG
	 Serial.print(name);
	 for (byte i = 0; i < len; ++i) {
			Serial.print(' ');
			Serial.print((int) buf[i], HEX);
	 }
//	 for (byte i = 0; i < len; ++i) {
//			Serial.print(' ');
//			Serial.print((int) buf[i], BIN);
//	 }
	 //Serial.print(' ');
	 //Serial.print(millis() / 1000);
	 //Serial.println();
#endif

	 if ((packetFill + len) < sizeof packetBuffer) {
			packetBuffer[packetFill++] = code + (len << 4);
			memcpy(packetBuffer + packetFill, buf, len);
			packetFill += len;
	 } else {
#if DEBUG
			Serial.print(" dropped: ");
			Serial.print(name);
			Serial.print(", ");
			Serial.print((int) len);
			Serial.println(" bytes");
#endif
	 }
}

static void addDecodedData (DecoderInfo& di) {
	 byte size;
	 const byte* data = di.decoder->getData(size);
	 addToBuffer(di.typecode, di.name, data, size);
	 di.decoder->resetDecoder();
}

// Check for a new pulse and run the corresponding decoders for it
static void runPulseDecoders (DecoderInfo* pdi, volatile word& pulse) {
	 // get next pulse with and reset it - need to protect against interrupts
	 cli();
	 word p = pulse;
	 pulse = 0;
	 sei();

	 // if we had a pulse, go through each of the decoders
	 if (p != 0) {
#if DEBUG_LED
			digitalWrite(DEBUG_LED, 1);
#endif
			while (pdi->typecode >= 0) {
				 if (pdi->decoder->nextPulse(p)) {
						//nextPulse returns 1: decoding is ready
						addDecodedData(*pdi);
						pdi->decoder->resetDecoder();
				 }
				 ++pdi;
			}
#if DEBUG_LED
			digitalWrite(DEBUG_LED, 0);
#endif
	 }
}

// see http://jeelabs.org/2011/01/27/ook-reception-with-rfm12b-2/
static void rf12_init_OOK () {
	 //rf12_initialize(0, RF12_868MHZ);
	 rf12_control(0x8027); // 8027		868 Mhz;disabel tx register; disable RX fifo buffer; xtal cap 12pf, same as xmitter

	 rf12_control(0x82c0); // 82c0		enable receiver ; enable basebandblock
	 //rf12_control(0xA68a); // A68A		868.2500 MHz
	 rf12_control(0xA686); // A68A		868.3500 MHz

	 rf12_control(0xc691); // c691		c691 datarate 2395 kbps 0xc647 = 4.8kbps
	 //rf12_control(0x9489); // 9489		VDI; FAST;200khz;GAIn -6db ;DRSSI 97dbm
	 //rf12_control(0x9491); // 9489		VDI; FAST;200khz;GAIn -14db; DRSSI 97dbm
	 rf12_control(0x9431); // 9489		VDI; FAST;400khz;GAIn -14db ;DRSSI 97dbm
	 //rf12_control(0x9439); // 9489		VDI; FAST;400khz;GAIn -20db; DRSSI 97dbm
	 rf12_control(0xC220); // c220		datafiltercommand ; ** not documented command
	 rf12_control(0xCA00); // ca00		FiFo and resetmode command ; FIFO fill disabeld
	 rf12_control(0xC473); // c473		AFC run only once ; enable AFC ;enable frequency offset register ; +3 -4
	 rf12_control(0xCC67); // cc67		pll settings command
	 rf12_control(0xB800); // TX register write command not used
	 rf12_control(0xC800); // disable low dutycycle
	 rf12_control(0xC040); // 1.66MHz,2.2V not used see 82c0
}

void setup () {
#if DEBUG_LED
	 pinMode(DEBUG_LED, 1);
	 // brief LED flash on startup to make sure it works
	 digitalWrite(DEBUG_LED, 1);
	 delay(100);
	 digitalWrite(DEBUG_LED, 0);
#endif

#if DEBUG
	 Serial.begin(57600);
	 Serial.println("\n[JeeNode_OOKrelay868]");
#endif

	 rf12_initialize(NODEID, RF12_868MHZ, NETGRP);
	 rf12_init_OOK();

	 setupPinChangeInterrupt();
}

void loop () {
	 runPulseDecoders(di_868, pulse_868);

	 if (sendTimer.poll(100) && packetFill > 0) {
			rf12_initialize(NODEID, RF12_868MHZ, NETGRP);

			Serial.print(" -> OK");
			for (byte i = 0; i < packetFill; ++i) {
				 Serial.print(" ");
				 Serial.print((int)packetBuffer[i], DEC);
			}
			//Serial.print(" (OK");
			//for (byte i = 0; i < packetFill; ++i) {
			//	 Serial.print(" ");
			//	 Serial.print((int)packetBuffer[i], HEX);
			//}
			Serial.println();

			rf12_sendNow(0, packetBuffer, packetFill);
			rf12_sendWait(1);
			rf12_init_OOK();

#ifdef PULSE_DEBUG
			if (tail != head) {
				 Serial.print("Pulse value(s): ");
				 while (tail != head) {
						Serial.print(buf[tail++], DEC);
						if (tail != head) Serial.print(" ");
				 }
				 Serial.println();
			}
			tail = 0;
			head = 0;
#endif
			packetFill = 0;
	 }
}

/// @file
/// Generalized decoder framework for 868 MHz and 433 MHz OOK signals.
// 2010-04-11 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <Arduino.h>
#include <util/crc16.h>

/// This is the general base class for implementing OOK decoders.
class DecodeOOK {
protected:
	 byte bits, flip, state, pos, data[25];
	 // the following fields are used to deal with duplicate packets
	 word lastCrc, lastTime;
	 byte repeats, minGap, minCount;

	 // gets called once per incoming pulse with the width in us
	 // return values: 0 = keep going, 1 = done, -1 = no match
	 virtual char decode (word width) =0;
	 
	 // add one bit to the packet data buffer
	 void gotBit (char value) {
			//char val = value+'0';
			//Serial.print(val);
			
			byte *ptr = data + pos; //Set a pointer to the right pos in the data buffer
			//*ptr = (*ptr >> 1) | (value << 7);		
			*ptr |= (value << bits); //AW!! Changed a bit, same result
			Serial.print(bits);
			
			if (++bits >= 7) {
				 bits = 0;
				 if (++pos < sizeof data) {
						// Prepare for the next byte to receive
						ptr++;
						*ptr = 0; //Reset next value
				 } else {
						resetDecoder();
						//return 0;
						return;
				 }
			}
			
			state = OK;
	 }
	 
	 // store a bit using Manchester encoding
	 void manchester (char value) {
			flip ^= value; // manchester code, long pulse flips the bit
			gotBit(flip);
	 }
	 
	 // move bits to the front so that all the bits are aligned to the end
	 void alignTail (byte max =0) {
			// align bits
			if (bits != 0) {
				 data[pos] >>= 8 - bits;
				 for (byte i = 0; i < pos; ++i)
				 data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
				 bits = 0;
			}
			// optionally shift bytes down if there are too many of 'em
			if (max > 0 && pos > max) {
				 byte n = pos - max;
				 pos = max;
				 for (byte i = 0; i < pos; ++i)
				 data[i] = data[i+n];
			}
	 }
	 
	 void reverseBits () {
			for (byte i = 0; i < pos; ++i) {
				 byte b = data[i];
				 for (byte j = 0; j < 8; ++j) {
						data[i] = (data[i] << 1) | (b & 1);
						b >>= 1;
				 }
			}
	 }
	 
	 void reverseNibbles () {
			for (byte i = 0; i < pos; ++i)
			data[i] = (data[i] << 4) | (data[i] >> 4);
	 }
	 
	 bool checkRepeats () {
			// calculate the checksum over the current packet
			word crc = ~0;
			for (byte i = 0; i < pos; ++i)
			crc = _crc16_update(crc, data[i]);
			// how long was it since the last decoded packet
			word now = millis() / 100; // tenths of seconds
			word since = now - lastTime;
			// if different crc or too long ago, this cannot be a repeated packet
			if (crc != lastCrc || since > minGap) {
				 repeats = 0;
			} else {
				 //Message repeated
				 //Serial.print("R");
			}
			// save last values and decide whether to report this as a new packet
			lastCrc = crc;
			lastTime = now;
			return (repeats++ != minCount);
	 }

public:
	 enum { UNKNOWN, SYNC, T0, T1, T2, T3, OK, TRAILING, DONE };

	 DecodeOOK (byte gap =5, byte count =0) 
	 : lastCrc (0), lastTime (0), repeats (0), minGap (gap), minCount (count)
	 { resetDecoder(); }
	 
	 bool nextPulse (word width) {
			if (state != DONE)
			switch (decode(width)) {
			case -1: // decoding failed
				 resetDecoder();
				 break;
			case 1: // decoding finished
				 while (bits) {
						//Serial.println("Padding");
						gotBit(0); //Fill the rest of the received buffer with zero's
				 }

				 // Added repeat state, to reset 
				 
				 if (!checkRepeats()) {
						// Not a repeated message, DONE
						state =	DONE;
				 } else {
						// Message was already received
						// Reset decoder for the next message
						resetDecoder();
				 }
				 break;
			}			
			return state == DONE;
	 }
	 
	 const byte* getData (byte& count) const {
			count = pos;
			return data; 
	 }
	 
	 void resetDecoder () {
			//Serial.print("-RD-");
			bits = pos = flip = 0;
			state = UNKNOWN;
	 }
};

// 868 MHz decoders

/// OOK decoder for Visonic devices.
class VisonicDecoder : public DecodeOOK {
public:
	 VisonicDecoder () {}
	 
	 virtual char decode (word width) {
			if (200 <= width && width < 1000) {
				 byte w = width >= 600;
				 switch (state) {
				 case UNKNOWN:
				 case OK:
						state = w == 0 ? T0 : T1;
						break;
				 case T0:
						gotBit(!w);
						if (w)
						return 0;
						break;
				 case T1:
						gotBit(!w);
						if (!w)
						return 0;
						break;
				 }
				 // sync error, flip all the preceding bits to resync
				 for (byte i = 0; i <= pos; ++i)
				 data[i] ^= 0xFF; 
			} else if (width >= 2500 && 8 * pos + bits >= 36 && state == OK) {
				 for (byte i = 0; i < 4; ++i)
				 gotBit(0);
				 alignTail(5); // keep last 40 bits
				 // only report valid packets
				 byte b = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
				 if ((b & 0xF) == (b >> 4))
				 return 1;
			} else
			return -1;
			return 0;
	 }
};

/// OOK decoder for FS20 type EM devices.
class EMxDecoder : public DecodeOOK {
public:
	 EMxDecoder () : DecodeOOK (30) {} // ignore packets repeated within 3 sec
	 
	 // see also http://fhz4linux.info/tiki-index.php?page=EM+Protocol
	 virtual char decode (word width) {
			if (200 <= width && width < 1000) {
				 byte w = width >= 600;
				 switch (state) {
				 case UNKNOWN:
						if (w == 0)
						++flip;
						else if (flip > 20)
						state = OK;
						else
						return -1;
						break;
				 case OK:
						if (w == 0)
						state = T0;
						else
						return -1;
						break;
				 case T0:
						gotBit(w);
						break;
				 }
			} else if (width >= 1500 && pos >= 9)
			return 1;
			else
			return -1;
			return 0;
	 }
};

/// OOK decoder for FS20 type KS devices.
class KSxDecoder : public DecodeOOK {
protected:
	 byte pulse, msgType, bitCount;
public:
	 KSxDecoder () {}
	 
	 bool gotBit (char value) {
			//char val = value+'0';
			//Serial.print(pos);
			//Serial.print(':');
			//Serial.print(bits);
			//Serial.print('-');
			//Serial.print(val);
			//Serial.print('.');
			
			byte *ptr = data + pos; //Set a pointer to the right pos in the data buffer

			if ((bitCount % 5) != 4) {
				 //*ptr |= (value << (7-bits)); //AW!! Reversed bits starting with b0 instead of b7
				 *ptr = (*ptr >> 1) | (value << 7);
				 if (++bits >= 8) {
						bits = 0;

						//Serial.println();
						if (pos == 0) {
							 //First received byte with msgType
							 msgType = *ptr & 0x0F;
							 //Serial.print("-Type:");
							 //Serial.print(msgType);
							 //Serial.print('-');
						}
						if (++pos < sizeof data) {
							 ptr++;
							 *ptr = 0; //Reset next value
						} else {
							 resetDecoder();
							 return 0;
						}
				 }
			} else {
				 // Every 5th bit needs to be '1'
				 if (!value) {
						// It is a '0' and this is wrong
						//Serial.println('Wrong parity');
						resetDecoder();
						return 0;
				 }
			}
			
			bitCount++;
			if ((bitCount == 49) && (msgType == 1)) {
				 //Serial.println("-EndOfMsg");
				 return 1; // Ready: End of message
			}
			if ((bitCount == 79) && (msgType == 7)) {
				 //Serial.println("-EndOfMsg");
				 return 1; // Ready: End of message
			}
			state = OK;
			return 0;
	 }

	 // see also http://www.dc3yc.homepage.t-online.de/protocol.htm
	 virtual char decode (word width) {
			/*
			if (200 <= width && width < 1000) {
				 if (width >= 600) Serial.print("-");
				 else							Serial.print("_");
			} else {
				 Serial.print(".P:");
				 Serial.println(width);
			}
*/
			if (200 <= width && width < 1000) {
				 byte w = width >= 600;
				 switch (state) {
				 case UNKNOWN:
						if (flip == 0) {
							 if (w) { // Long pulse received, next needs to be a short one.
									pulse = 0;
									flip++;
							 }
						} else {
							 // Receiving preamble
							 if (!pulse) {
									//Previous was a long pulse, now expect a short pulse
									if (!w) { 
										 flip++;
										 pulse = 1; 
										 //Serial.print("p");
									}	else {
										 flip = 0; //Long pulse received: Preamble failed
										 //Serial.println("X");
									}
							 } else {
									//Previous was a short pulse, now expect a long pulse
									if (w) {
										 flip++;	
										 pulse = 0; 
										 //Serial.print("P"); 
									} else {
										 //Short pulse received
										 if ((flip >= 14) && (flip <= 20)) { // Preamble is 7 till 10 pulses (*2=14..20)
												//This could be a sync pulse (logical '1')
												//Then next pulse should be a long pulse
												state = SYNC;
												//Serial.print("flip:");
												//Serial.print(flip);
										 } else {
												flip = 0; //Short pulse received: Preamble failed
												//Serial.println("x");
										 }
									}
							 }
						}
						break;
				 case SYNC:
						if (w) {
							 //Correct: received a long pulse
							 state = OK;
							 bits = pos = 0;
							 bitCount = 0;
							 msgType = 0;
							 //Serial.print("S");
						} else {
							 //Strange: the next part of the pulse was a short pulse, it should be a long one
							 //Serial.println("SYNC failed.");
							 return -1; // Return decoding Failed
						}
						break;
				 case OK:
						state = w == 0 ? T0 : T1; //AW!! Reversed T0 and T1
						break;
				 case T0:
						//Serial.print("1");
						if (gotBit(1)) return 1; // Return Ready
						if (!w) {
							 //Serial.println("TO=decode failed.");
							 return -1; // Return decoding Failed
						}
						break;
				 case T1:
						//Serial.print("0");
						if (gotBit(0)) return 1; // Return Ready
						if (w) {
							 //Serial.println("T1=decode failed.");
							 return -1; // Return decoding Failed
						}
						break;
				 }
			} else if (width >= 1500 && pos >= 6) {
				 //Serial.println("decode ready.");
				 return 1; // Return decoding Ready
			} else {
				 //Serial.println("pulse: decode failed.");
				 return -1; // Return decoding Failed
			}
			
			return 0;
	 }
};

/// OOK decoder for FS20 type FS devices.
class FSxDecoder : public DecodeOOK {
protected:
	 byte bitCount;
	 bool extendedMsg, parity;
public:
	 FSxDecoder () {}
	 
	 bool gotBit (char value) {
			byte *ptr = data + pos; //Set a pointer to the right pos in the data buffer
			*ptr |= (value << (7-bits)); //AW!! Reversed bits starting with b0 instead of b7
			if (value) parity = !parity;
			
			if ((bitCount % 9) != 8) {
				 if (++bits >= 8) {
						bits = parity = 0;

						if (pos == 3) {
							 //4th byte with command: b5=extendMsg
							 //extendedMsg = ((*ptr & 0x20) != 0);
							 extendedMsg = false;
							 //Serial.print("-ExtMsg:");
							 //Serial.print(extendedMsg);
							 //Serial.print("-");
						}

						if (++pos < sizeof data) {
							 ptr++;
							 *ptr = 0; //Reset next value
						} else {
							 resetDecoder();
							 return 0;
						}
				 }
			} else {
				 // This is the even parity bit, check this
				 if (parity == value) {
						// Parity OK
						//Serial.print("E");
				 } else {
						// Decoding failed, parity error
						resetDecoder();
						return 0;
				 }
			}
			bitCount++;
			
/*			if (bitCount >= 44) {
				 Serial.print("-");
				 Serial.print(bitCount);
				 Serial.print("-");
			} */
			if ((bitCount == 45) && !extendedMsg) {
				 //End of normal message (5 bytes * 9 bits)
				 //Serial.print("-");
				 return 1; // Ready: End of message
			}
			if ((bitCount == 54) && extendedMsg) {
				 //End of extended message (6 bytes * 9 bits)
				 //Serial.print("-End-");
				 return 1; // Ready: End of message
			}
			state = OK;
			return 0;
	 }

	 // see also http://fhz4linux.info/tiki-index.php?page=FS20%20Protocol
	 virtual char decode (word width) {
			/*if (300 <= width && width < 775) {
				 //if (width >= 500) Serial.print("-");
				 //else							Serial.print("_");
			} else {
				 Serial.print(".P:");
				 Serial.println(width);
			} */
			if (300 <= width && width < 775) {
				 byte w = width >= 500;
				 switch (state) {
				 case UNKNOWN:
						//Serial.print("UNKNOWN:");
						if (w == 0) {
							 //Preamble: 12 times a logical '0', which is normally 24 times at this point
							 //Serial.print("p");
							 ++flip;
						} else if (flip > 20) {
							 //Preamble end: sync pulse: 1 time a logical '1'
							 //Serial.print("P");
							 state = SYNC;
						} else {
							 //Serial.println("UNKNOWN=decode failed.");
							 return -1; //decoding failed
						}
						break;
				 case SYNC:
						if (w) {
							 //Correct: received a logical '1'
							 state = OK;
							 //Serial.print("S");
							 bitCount = parity = extendedMsg = 0;
							 data[0] = 0;
						} else {
							 //Strange: the next part of the pulse was a logical '0', expected a '1'
							 //Serial.println("SYNC=decode failed.");
							 return -1; //decoding failed
						}
						break;
				 case OK: //First part of logical '0' or '1'
						state = w == 0 ? T0 : T1;
						break;
				 case T0: //Collect the next part of the logical '0'
						if (!w) {
							 //Correct: received a logical '0'
							 //Serial.print("0");
							 if (gotBit(0)) return 1; // Return Ready
						} else {
							 //Strange: the next part of the pulse was a logical '1', expected a '0'
							 //Serial.println("TO=decode failed.");
							 return -1; //decoding failed
						}
						break;
				 case T1: //Collect the next part of the logical '1'
						if (w) {
							 //Correct: received a logical '1'
							 //Serial.print("1");
							 if (gotBit(1)) return 1; // Return Ready
						} else {
							 //Strange: the next part of the pulse was a logical '0', expected a '1'
							 //Serial.println("T1=decode failed.");
							 return -1; //decoding failed
						}
						break;
				 }
			} else if (width >= 1500 && pos >= 5) {
				 //Terminating pulse received and enough data
				 //Serial.println("decode ready.");
				 return 1; //decoding ready
			} else {
				 //Serial.println("pulse:decode failed");
				 return -1; //decoding failed
			}
			return 0;
	 }
};

// Dumb Arduino IDE pre-processing bug - can't put this in the main source file!
/// Structure used to defined each entry in the decoder table.
typedef struct {
	 char typecode;
	 const char* name;
	 DecodeOOK* decoder;
} DecoderInfo;

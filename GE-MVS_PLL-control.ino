/*
* MaxTroller
 * Copyright (C)2011, Ground Loop
 * 2011-07-02 Version 1.0
 * 2011-07-05 Version 2.0
 * 2011-07-06 Version 2.1
 * 2011-08-11 Version 2.2 (fixed 12.5kHz tuning)
 * 2011-08-12 Version 2.3 (setting for trace cut vs not)
 *
 * Receive tuning commands from UniTrunker as a simulated Uniden
 * BC346XT, and send PLL settings directly to the MaxTrac PLL for
 * tuning.
 *
 * !!! USE WITH CAUTION !!!
 * If you chose to build a new radio out of parts and software, you
 * must take ALL necessary measures to ensure you do not ever make
 * unauthorized transmissions or receive prohibitted bands.  It is
 * YOUR RESPONSIBILITY as a radio builder to secure any necessary
 * licenses, certifications and permission needed to operate a
 * homebrew radio.
 *
 * Required safety measures include programming your radio software to
 * inhibit ALL transmission, setting tx deviation to zero, removing
 * exciter sections, and powering your radio with a low-current
 * (500mA?) supply or fuse.
 *
 * Your radio, your ham license, YOUR responsibility.
 *
 ****** ****** ****** ****** ****** ****** ****** ****** ****** ******
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ****** ****** ****** ****** ****** ****** ****** ****** ****** ******
 *
 * To interface to a MaxTrack, you should cut the traces near 
 * two pins:
 * J6-6 (Serial Data)
 * J6-7 (Serial Clock)
 * This isolates the control CPU from the PLL, while still allowing
 * the CPU to program the DAC for Reference Clock calibration.  If you
 * are noncommital or curious, there is a fair chance the Arduino can
 * override the CPU and work just fine without cutting these traces.
 * Some PLL programming may get garbled during a collision, especially
 * on Trunk-only radios.  Try it and see.
 *
 * If you have a Conventional codeplug in the radio, you can park it on
 * a conventional channel and rely on the CPU to unmute the audio when
 * Carrier Squelch receives signal.  Good for you.
 *
 * However, if you have a Trunk-only radio, it will never 'unmute'
 * itself, regardless of squelch.  So, you must cut the CPU signals
 * from RX MUTE and PA MUTE and drive those from the Arduino.  This is
 * Pin 6 (RX MUTE) and Pin 7 (PA MUTE) on the CPU803.  These traces
 * are easy to access and cut.  Then you must tap the same lines at
 * the nearby vias, bond them together, and bring those to Digital 6
 * of the Arduino.  If this sounds too hard, maybe replacing your
 * EPROM or whole radio is easier to get at least one Conventional
 * channel.  

 * There is a jumper option that lets you get muted audio on the back
 * accessory connector as well, with no PA involved, but you still
 * have to reroute RX MUTE to let the Arduino control that gate.
 *
 * The J6-8 (LE) pin doesn't need to be disconnected/cut, which is
 * good because it's hard to reach.  We can just haggle with the CPU
 * over it, and win every time.
 *
 * To sense carrier squelch for a crude RSSI, tap into the "CSQ"
 * testpoint between J6 and the CPU can.  It's conveniently circled
 * and labeled "CSQ" on the silkscreen.
 *
 * CONNECT:
 * Arduino Digital Pin 2 to J6-6 (Serial Data)
 * Arudino Digital Pin 3 to J6-7 (Serial Clock)
 * Arduino Digital Pin 4 to J6-8 (Synth LE)
 * Arduino Digital Pin 5 to U601-1 (HS Data) [UNUSED Here]
 * Arudino Digital Pin 6 to replace U803 Pin6&7 (RX UNMUTE & PA UNMUTE)
 * Arudino Digital Pin 7 to J9-2 (Display Enable) OPTIONAL
 * Arudino Digital Pin 8 to CSQ (Carrier Squelch testpoint)
 * Connect Arduino Ground to J6-11 or other convenient ground.
 */

#define DATA  2   /* Serial Data */
#define CLK 3   /* Serial Clock */
#define LE  4   /* Synthesizer Latch Enable */
#define DISCDAT 5   /* Discriminator data (NOT analog) */
#define UNMUTE  6   /* HIGH to unmute RX audio and Amp */
#define DISP  7   /* Display Enable (latch) */
#define CSQ 8   /* Squelch sense */


/* Must be defined to one of:
   0: All traces intact (Conventional radio)
   1: J6-6 and J6-7 traces are cut (Trunking-only radios)
*/

#define TRACES_CUT 1  //Modified 4 April 2012
#define LED 13      /* Built into (most) Arduino boards */
#define SAMPS 256       /* Number of samples in the buffer (max 256) */
byte samp[SAMPS];       /* contains the bit samples */
volatile byte samp_idx = 0;   /* Next sample slot */

// I miss these:

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


void setup()
{
  Serial.begin (115200);
  Serial.println ("GE MVS Controller");

  pinMode (LED, OUTPUT);
  digitalWrite (LED, LOW);  /* Turn off on-board LED */

  /*
   * Drive signals onto all PLL pins.
   *
   * Note that the contention/draw from CPU also driving LE is weak,
   * through a 1k resistor.  We can strong-arm it.  Likewise for
   * Display Enable.
   */

  pinMode (LE,     OUTPUT);
  pinMode (UNMUTE, OUTPUT);

  //drive_bus (0);

  /* Default idle pin states */
  digitalWrite (LE, HIGH); 
  digitalWrite (DISP, LOW);
  digitalWrite (UNMUTE, LOW);
  digitalWrite (CLK, LOW);

/*some shit here to pretune the radio PLL with 32 bits to set R, N, A inital values
 *MC145159 requires the PLL be tuned with 32 bits at first programming, last bit, which
 *is the control bit, needs to be 1.
 *
 *The pll tune section below should satisfy the requirements to tune only the N and A counters
 *after the R counter has already been set. The LSB for the control bit is already zero.
  */



  pinMode (DISCDAT, INPUT); /* Tri-state, without pull-up */
  digitalWrite (DISCDAT,  LOW);
  pinMode (CSQ, INPUT);    
  digitalWrite (CSQ, LOW);    // this will need to change if radio CSQ output 5V on closed squelch

  /* 
   * We want to oversample a 3600bps stream at 5x:
   * 3600 * 5 = 18,000 Hz sample rate.  
   * Arduino runs at 16MHz Hz.  
   * So, we want one timer event every 889 clock cycles.
   */
  TIMSK1 = 0;     /* Disable all timer1 interrupts */
  TCCR1A = 0;     /* Normal: Don't drive OC1A/OC1B */
  TCCR1B = _BV(WGM12) | _BV(CS10); /* CTC Mode 4, clk/1 no prescale */
  OCR1A = 888;      /* 889 ticks between interrupts */

  display (0xFFFF);

  drive_bus(1);
  emit_byte (0b00101000);   
  emit_byte (0b00000001);
  emit_byte (0b00110011);
  emit_byte (0b10001001);
  pulse_le();
  drive_bus(0);
}

/* Turn on or off bitstream sample collection */
void collect_samples (byte go)
{
  if (go) {
    samp_idx = 0;    /* Trash any pre-existing samples */
    /* Enable the timer1 Compare A interrupt.  GO! */
    sbi (TIMSK1, OCIE1A);
  } else {
    cbi (TIMSK1, OCIE1A); /* Stop */
  }
}

/* Interrupt Service Routine for Timer1 Compare A */
ISR(TIMER1_COMPA_vect)
{
  samp[samp_idx++] = PIND;  /* Inhale all of PortD states */

#if SAMPS != 256
  samp_idx &= SAMPS - 1;  /* Wrap */
#endif
}

/* Dummy variable used for delay.  Updating this is slower than a NOP. */

volatile unsigned long int waste = 0;


/* We are about to drive data onto the serial bus for the PLL
   divider.  If the pins are still connected to the MaxTrac CPU, we
   have to drive/override them now.  They are normally tri-stated by
   Arduino so that the CPU can access the Ref Osc tuning DAC (warp).

   If the traces are cut, then they must always be driven by
   Arduino, and the CPU has isolated access only to the DAC.  Clear as
   mud? */

void drive_bus (byte enable)
{
#if TRACES_CUT
  enable = 1;     /* ignore parameter, force drive */
#endif

  if (enable) {
    pinMode (DISP, OUTPUT);
    pinMode (DATA, OUTPUT);
    pinMode (CLK, OUTPUT);
    digitalWrite (DATA, LOW);
  } 

  else {
    pinMode (DISP, INPUT);
    pinMode (DATA, INPUT);
    pinMode (CLK, INPUT);
    digitalWrite (DATA, HIGH);
    }
}

/*
 * Send a (positive) synthesizer Latch Enable pulse to the PLL
 */

void pulse_le (void)
{
  digitalWrite (LE, LOW);
  waste++;      /* + pulse width: 6.3uS */
  waste++;
  digitalWrite (LE, HIGH);
}

/*
 * Send one byte (8 bits) to the serial clock/data lines.
 * MSB first
 * We're shooting for (approximately) 16 microsecond period here.  
 * Sequence is important, but timing is not critical.
 */

void emit_byte (byte c)
{
  byte bit;
  for (bit = 0; bit < 8; bit++)
    {
      /* Change DATA state only while CLK is low */
      digitalWrite (DATA, (c & 0x80) ? LOW : HIGH);
      waste++;
      waste++; 
      digitalWrite (CLK, LOW); /* rising edge latches data */

      c <<= 1;      
      waste++;
      waste++;
      waste++;
      digitalWrite (CLK, HIGH);
      /* Loop delay is about 8.8uS */
    }

  /* Idle state.  Probably doesn't matter. */
  digitalWrite (DATA, LOW);
}

/* 
 * Compute the PLL tuning bits and send them to the dividers.
 * Rx Freq = (128 * N + A) * 5 kHz - 45 MHz 
 * 45 MHz is the first mixer.  For GE MVS VHF radios, we tune the
 * VCO to 45MHz above the desired RX freq.
 */

void set_pll (unsigned long int hz)

{
  unsigned int r;   // R is the 14 bit reference counter, must be loaded first, then N and A immediately follow
  unsigned int n;   /* N is the 10-bit divide-by-128 counter  */
  byte a;           /* A is the 7-bit remainder counter */

  hz += 45000000;   /* first mixer: 45 MHz */
  hz /= 5000;      /* Divide by 5KHz reference. (ie, step size) */
  n = hz / 128;     // 128 is the prescaler
  a = hz - (n * 128);

  r = 2560;      // reference counter int value derived from ref osc / tuning step -> 12.8 / .005


  drive_bus (1);

  //emit_byte ((r) & 0xFF);			        // send first eight bits of r counter
  //emit_byte (((r & 0xFF) & n) & 0xFF);	// send last 6 bits of r counter, then send first two bits of n counter
  //emit_byte ((n & 0xFF) & 0xFF);			// send last eight bits of n counter
  //emit_byte (((a) & 0xFF) & 0x01 << 7); 	// send seven bits for a counter, and one HIGH bit for control bit
  

 /*these four lines are for hard coded R, N, A counter testing
  * 32 bits, are sent to program the PLL with the RX info for 151.820MHz
  * Tested and confirmed to have worked 20180104
  emit_byte (0b00101000);   
  emit_byte (0b00000001);
  emit_byte (0b00110011);
  emit_byte (0b10001001);
  */
  
  //emit_byte (0x14);
  //emit_byte (0x01);
  /* Send "(n << 8)|(a << 1)", 24 bits, MSB first, LSB always zero */
  //emit_byte (n >> 2);
  emit_byte ((n >> 8 ) & 0xFF);  /* N high byte */
  emit_byte (n & 0xFF);   /* N low byte */
  //emit_byte (a & 0b1);   // send only seven bits?
  emit_byte (a << 1);   /* A and LSB 0 */
  //emit_byte (0x01 << 7);        // send only one HIGH bit?
  
  pulse_le();     /* Latch it */


  /* 
     Send 14-bit prescalar "R".  
     This divides the Reference Oscillator (12.8 Mhz) to get 5 kHz
     It's a constant (2560) for the entire valid range.
  */

  //emit_byte (0x14);   // 0x is R=2560<<1 with LSB flag set, adding the LSB flag pushes the bits up by 1 bit, doubling the original bit value by 2
  //emit_byte (0x01);
  //pulse_le();
  
  drive_bus (0);

}



/* Tune to a frequency passed in as a UniTrunker "QSH" command.
   This code is fairly rigid, but matches what UniTrunker always sends.

   Turn on logging in the UniTrunker Voice Receiver window to see the
   actual commands.  They look like this:

   QSH,8570000,AUTO,NFM,0,2,0,0,0,0

   We care only about the frequency, given in units of 100 Hz.
*/

void tune (char *s)
{
  unsigned long int hz;

  s += 4;     /* Skip "QSH," */

  /* Read in the Hz value */

  hz = 0; 
  while (*s && (*s != ',')) {

    hz = hz * 10 + (*s++ - '0');
  }

  hz *= 100;      /* We're actually given 1/10 kHz (7 digits) */

  /* The Electronic Communication Privacy Act (ECPA) of 1986 prohibits
     manufacturing a receiver capable of receiving prohibited
     frequencies.  Some of those may be within PLL tuning range of the
     Maxtrac, and must be blocked here to prevent tuning:
  824.000-849.000 MHZ
  869.000-894.000 MHZ
  */

  if ((hz >= 824000000UL && hz <= 849000000UL) ||
      (hz >= 869000000UL && hz <= 894000000UL))
    return;     /* Do not tune! */



  /* Parking.
     Set up your UniTrunker (Receiver->Control->Park) to tune to
     "0.00000" for Park.  We detect that magic value here to know when
     Audio has ended and kick the RXMUTE line low.  (We don't actually
     detune.)

     NOTE: Version 1.0.0.22 of UniTrunker has a bug that causes
     "premature parking".  It will send the Park command while the
     audio call is still in progress.  It will also switch calls in
     progress, even to a lower priority call.
  */

  if (hz == 0) {

    digitalWrite (UNMUTE, LOW); /* Park.  Mute.  Shhhhhh. */

  } else {

    set_pll (hz);   /* Tune */
    delay (10);     /* Reduce tuning noise burst */
    digitalWrite (UNMUTE, HIGH);/* Unmute RX & PA audio gates */
  }
}



/* 

   Set the display segments.
   There are 16 segments: 
   Two 7-segment numeric displays
   Two LEDs.
   (Note the Latching method is different than PLL.)
 */

void display (unsigned int d)
{
  drive_bus (1);
  digitalWrite (DISP, HIGH);
  emit_byte ((d >> 8) & 0xFF);
  emit_byte ((d >> 0) & 0xFF);
  digitalWrite (DISP, LOW);

  drive_bus (0);
}



/* Just a simple animation pattern to show life on the display */

byte bling[] = {

  1, 9, 11, 10, 2, 6, 5, 13, 12, 10, 2, 0 /* single-segment pattern */
};

#define BLINGS (sizeof (bling) / sizeof(bling[0]))

byte bling_idx = 0;





#define BUFSIZE 40    /* Max command length from UniTrunker */

char in_str[BUFSIZE];   /* Incoming command buffer from RS232 */
int in_idx = 0;     /* Command buffer index */



/* 
 * Process a fully-received string from UniTrunker. 
 * The responses are important, as UniTrunker will retry/reset if 
 * we don't respond appropriately.
 */

void process()
{
  in_str[in_idx] = 0;     /* Force null-termination */

  if (!strcmp (in_str, "MDL"))  /* Model */
    Serial.print ("MDL,BC346XT\r"); /* lies */
  else if (!strcmp (in_str, "PWR")) /* RSSI Level */

    Serial.print ((digitalRead(CSQ) == HIGH) ? "PWR,99,0\r" : "PWR,0,0\r" );

  else if (!strcmp (in_str, "REL")) /* (Not Uniden) Release all outputs & pull-ups */

    {
      DDRD = 0;
      PORTD = 0;
      DDRB = 0;
      PORTB = 0;

      /* Must reset Arduino to get them back! */
    }

  else if (!strncmp (in_str, "BIT", 3))

    {

      if (in_str[4] == '0')

  collect_samples (0);

      else

  collect_samples (1);

      Serial.print ("BIT,OK\r");

    }

  else if (!strncmp (in_str, "QSH", 3)) /* Quick Search Hold */

    {

      digitalWrite (LED, HIGH);

      tune (in_str);    /* Pass the whole command line */

      Serial.print ("QSH,OK\r");

      digitalWrite (LED, LOW);

    }

  else if (in_idx == 0)   /* Quietly ignore empty commands */

    ;

  else

    Serial.print ("ERR\r"); /* Unrecognized command or garbage */



  in_idx = 0;     /* Reset input buffer */



  /* Step the display animation once for each received command */

  display (1 << bling[bling_idx++]);

  if (bling_idx >= BLINGS)

    bling_idx = 0;

}



#define OUT_BYTES 3+64+1

byte str_out[OUT_BYTES];

byte str_out_idx = 0;



void nibble_send (byte n)

{

  n &= 0xF;

  Serial.write (n < 10 ? ('0' + n) : ('A' + n - 10));

}



void nibble_add (byte n)

{

  static int ncount = 0;



  n &= 0xF;

  str_out[str_out_idx++] = (n < 10 ? ('0' + n) : ('A' + n - 10));



  if (str_out_idx == OUT_BYTES-1) {

    str_out[str_out_idx++]='\r';

    Serial.write (str_out, str_out_idx);



    str_out_idx = 0;

    str_out[str_out_idx++] = 'R';

    str_out[str_out_idx++] = '1';

    str_out[str_out_idx++] = ',';

  }

}





#if 0

void serial_add (byte c)

{

  byte i;

  str_out[str_out_idx++] = c;

  if (str_out_idx == OUT_BYTES) {

    Serial.print ("R1,");

    for (i = 0; i < OUT_BYTES; i++) {

      send_nibble (str_out[i] >> 4);

      send_nibble (str_out[i] >> 0);

    }

    Serial.print ('\r');

    str_out_idx = 0;

  }

}

#endif



#if 0

// Raw for testing 

/* 

 * Send a single bit out the serial port.

 * (Every eighth call transmits one byte, naturally)

 *

 * The MSB represents the first bit sent, LSB the last bit sent.

 *

 * 'bit' argument must be 0 or 1

 */

void send_bit (byte bit) {

  static byte c = 0;

  static byte count = 0;



  c <<= 1;

  c |= bit;

  if (++count & 8) {

    count = 0;

    Serial.write (c);

  }

}

#endif



void send_bit (byte bit) {

  static byte c = 0;

  static byte count = 0;



  c <<= 1;

  c |= bit;

  if (++count & 4) {

    count = 0;

    nibble_add (c);

  }

}



/* 

 * Process and send half the samples, starting at slot 'base' 

 */

void dump (byte base)

{

  byte i;



  digitalWrite (LED, HIGH);

  for (i = 0; i < SAMPS/2; i++){

    byte is_hi;



    is_hi = !!(samp[base + i] & _BV(PORTD5));

    //handle_bit (is_hi);

    send_bit (is_hi);   /* Send this bit */

  }

  digitalWrite (LED, LOW);

}



/*

 * Collect serial characters and, when the command terminator is

 * received, process the whole command.

 */

void check_serial_in(void)

{

  byte c;



  if (Serial.available() > 0) {

    c = Serial.read();    // get one incoming byte

    if (c == '\r' || in_idx == BUFSIZE - 1)

      process ();

    else

      in_str[in_idx++] = c;

  }

}



/* 

 * The main loop just waits for the ISR to fill the sample array, then

 * works on the "not busy" half.

 */

void loop(void)

{

  /* Wait for lower samples to be stable */

  while (!(samp_idx & (SAMPS/2)))

    check_serial_in();

  /* Dump the lower samples */

  dump (0);



  /* Wait for upper samples to be stable */

  while (samp_idx & (SAMPS/2))

    check_serial_in();

  /* Dump upper samples */

  dump (SAMPS/2);

}


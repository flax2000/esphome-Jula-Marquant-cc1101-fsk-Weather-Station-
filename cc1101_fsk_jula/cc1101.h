//made by swe dude
//used this as starting point https://github.com/dbuezas/esphome-cc1101
//decoding from: https://github.com/merbanan/rtl_433/blob/master/src/devices/emax.c
//may or may not decode the following: Emax W6, rebrand Altronics x7063/4, Optex 990040/50/51, Orium 13093/13123, Infactory FWS-1200, Newentor Q9, Otio 810025, Protmex PT3390A, Jula Marquant 014331/32, Weather Station or temperature/humidity sensor

//only tried with Jula Marquant temperature/humidity sensor


int start_delay_counter = 1;

int channel_;
int channel_old=0;
float Deviation_;
float Deviation_old=0;
float DRate_;
float DRate_old=0;
float Bandwidth_;
float Bandwidth_old=0;

int uv_index = 0;
int light_lux = 0;
float gust_ms = 0;


unsigned long millis_minne = 0;
int rx_counter = 0;

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
  int result = 0;
  result += 0xAA;//adding missing payload
  for (unsigned i = 0; i < num_bytes; ++i) {
    result += message[i];
  }
  return result;
}




#ifndef CC1101TRANSCIVER_H
#define CC1101TRANSCIVER_H

#include <ELECHOUSE_CC1101_SRC_DRV.h>

class CC1101 : public PollingComponent {


    int _SCK;
    int _MISO;
    int _MOSI;
    int _CSN;


  public:
    Sensor *fsk_temp_ = new Sensor();
    Sensor *fsk_hum_ = new Sensor();
    Sensor *fsk_rssi_ = new Sensor();
    Sensor *fsk_battery_low_ = new Sensor();
    Sensor *fsk_avg_rx_ = new Sensor();

    CC1101(int SCK, int MISO, int MOSI, int CSN )
      : PollingComponent(20) {
      _SCK = SCK;
      _MISO = MISO;
      _MOSI = MOSI;
      _CSN = CSN;
    }

    float get_setup_priority() const override {
      return esphome::setup_priority::AFTER_WIFI;
    }




    byte Read_ReceiveData_with_limit(byte * rxBuffer, byte limit)
    {
      byte size;
      byte status[2];
#define   BYTES_IN_RXFIFO   0x7F            //byte number in RXfifo
      if (ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)
      {
        size = ELECHOUSE_cc1101.SpiReadReg(CC1101_RXFIFO);
        if (size > limit) size = limit;
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxBuffer, size);
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, status, 2);
        ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX);
        ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
        return size;
      }
      else
      {
        ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX);
        ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
        return 0;
      }
    }






    void setup()
    {
      ;
    }



    void update() override {
      if (start_delay_counter) start_delay_counter++;
      if (start_delay_counter == 250)
      {
        start_delay_counter = -1;
        ELECHOUSE_cc1101.addSpiPin(_SCK, _MISO, _MOSI, _CSN, 0);
        ELECHOUSE_cc1101.setModul(0);
        if (ELECHOUSE_cc1101.getCC1101()) {     // Check the CC1101 Spi connection.
          ESP_LOGD("setup", "Connection OK");//Serial.println("Connection OK");
        } else {
          ESP_LOGD("setup", "Connection Error");//Serial.println("Connection Error");
        }


        ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
        ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode.
        ELECHOUSE_cc1101.setModulation(0);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
        //try increasing/decreasing slightly
        ELECHOUSE_cc1101.setMHZ(430.695221);//(433.935);        // 433.935 Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
        ELECHOUSE_cc1101.setDeviation(40);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
        ELECHOUSE_cc1101.setChannel(125);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
        ELECHOUSE_cc1101.setChsp(25.390625);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
        ELECHOUSE_cc1101.setRxBW(250);//250  (150/125dr ok)     // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
        //try setDRate(10.96); setDRate(10.97);setDRate(10.99); setDRate(11); setDRate(11.01);
        ELECHOUSE_cc1101.setDRate(10.94);       // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
        ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
        ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
        ELECHOUSE_cc1101.setSyncWord(0xCA, 0x54); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
        ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
        ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
        ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
        ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
        ELECHOUSE_cc1101.setLengthConfig(2);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
        ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
        ELECHOUSE_cc1101.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
        ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
        ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
        ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
        ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.

        ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
        ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
        ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

#define CC1101_AGCCTRL2     0x1B        // AGC control
        // CC1101_REG_AGCCTRL2
#define CC1101_MAX_DVGA_GAIN_0                        0b00000000  //  7     6     reduce maximum available DVGA gain: no reduction (default)
#define CC1101_MAX_DVGA_GAIN_1                        0b01000000  //  7     6                                         disable top gain setting
#define CC1101_MAX_DVGA_GAIN_2                        0b10000000  //  7     6                                         disable top two gain setting
#define CC1101_MAX_DVGA_GAIN_3                        0b11000000  //  7     6                                         disable top three gain setting
#define CC1101_LNA_GAIN_REDUCE_0_DB                   0b00000000  //  5     3     reduce maximum LNA gain by: 0 dB (default)
#define CC1101_LNA_GAIN_REDUCE_2_6_DB                 0b00001000  //  5     3                                 2.6 dB
#define CC1101_LNA_GAIN_REDUCE_6_1_DB                 0b00010000  //  5     3                                 6.1 dB
#define CC1101_LNA_GAIN_REDUCE_7_4_DB                 0b00011000  //  5     3                                 7.4 dB
#define CC1101_LNA_GAIN_REDUCE_9_2_DB                 0b00100000  //  5     3                                 9.2 dB
#define CC1101_LNA_GAIN_REDUCE_11_5_DB                0b00101000  //  5     3                                 11.5 dB
#define CC1101_LNA_GAIN_REDUCE_14_6_DB                0b00110000  //  5     3                                 14.6 dB
#define CC1101_LNA_GAIN_REDUCE_17_1_DB                0b00111000  //  5     3                                 17.1 dB
#define CC1101_MAGN_TARGET_24_DB                      0b00000000  //  2     0     average amplitude target for filter: 24 dB
#define CC1101_MAGN_TARGET_27_DB                      0b00000001  //  2     0                                          27 dB
#define CC1101_MAGN_TARGET_30_DB                      0b00000010  //  2     0                                          30 dB
#define CC1101_MAGN_TARGET_33_DB                      0b00000011  //  2     0                                          33 dB (default)
#define CC1101_MAGN_TARGET_36_DB                      0b00000100  //  2     0                                          36 dB
#define CC1101_MAGN_TARGET_38_DB                      0b00000101  //  2     0                                          38 dB
#define CC1101_MAGN_TARGET_40_DB                      0b00000110  //  2     0                                          40 dB
#define CC1101_MAGN_TARGET_42_DB                      0b00000111  //  2     0     

        //experimental setting mayby improve range remove if cant recieve

        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, CC1101_MAGN_TARGET_33_DB | CC1101_LNA_GAIN_REDUCE_0_DB | CC1101_MAX_DVGA_GAIN_0); //range
        //ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, CC1101_MAGN_TARGET_40_DB | CC1101_LNA_GAIN_REDUCE_0_DB | CC1101_MAX_DVGA_GAIN_0); //low noice

        millis_minne = millis();
        ESP_LOGD("setup", "Rx Mode");//Serial.println("Rx Mode");
      }

      if (!start_delay_counter)
      {


        if (channel_ != channel_old)
        {
          ELECHOUSE_cc1101.setChannel(channel_);
          channel_old = channel_;
          ESP_LOGD("rx", "channel set");
        }

        if (Deviation_ != Deviation_old)
        {
          ELECHOUSE_cc1101.setDeviation(Deviation_);
          Deviation_old = Deviation_;
          ESP_LOGD("rx", "Deviation set");
        }

        if (DRate_ != DRate_old)
        {
          ELECHOUSE_cc1101.setDRate(DRate_);
          DRate_old = DRate_;
          ESP_LOGD("rx", "DRate set");
        }
        if (Bandwidth_ != Bandwidth_old)
        {
          ELECHOUSE_cc1101.setRxBW(Bandwidth_);
          Bandwidth_old = Bandwidth_;
          ESP_LOGD("rx", "Bandwidth set");
        }



        if ((millis() - millis_minne) > 300000)
        {
          fsk_avg_rx_->publish_state(rx_counter * 60000.0 / (millis() - millis_minne));
          rx_counter = 0;
          millis_minne = millis();
        }


        //Checks whether something has been received.
        //When something is received we give some time to receive the message in full.(time in millis)
        if (ELECHOUSE_cc1101.CheckRxFifo(50)) {
          byte b[50] = {0};

            //Rssi Level in dBm
            //Serial.print("Rssi: ");
            int rssi = (ELECHOUSE_cc1101.getRssi());
            //Serial.print(rssi);
            //Link Quality Indicator
            //Serial.print("LQI: ");
            //Serial.println(ELECHOUSE_cc1101.getLqi());

            //Get received Data and calculate length
            //int len = ELECHOUSE_cc1101.ReceiveData(b);

            // added this with a limit had payloads 200+ size....
            int len = Read_ReceiveData_with_limit(b, 32);
            //b[len] = '\0';

            //Serial.print("Lenght: ");
            //Serial.print(len);
            //Serial.print(" data ");
            //Print received in char format.
            //Serial.println((char *) buffer);

            //Print received in bytes format.
            // for (int i = 0; i < len; i++) {
            //  Serial.print(b[i], HEX);
            //  Serial.print(" ");
            //}
            //Serial.println();
            /*
                    uint8_t b[32] = {0};

                  for (int i = 0; i < 32; i++) {
                    Serial.print(bb[i+6], HEX);
                    Serial.print(",");
                    b[i]=bb[i+6];

                  }
              Serial.println();
            */
            if (len > 30)
            {


              ESP_LOGD("th", "data %x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x" , b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15], b[16], b[17], b[18], b[19], b[20], b[21], b[22], b[23], b[24], b[25], b[26], b[27], b[28], b[29], b[30], b[31], b[32]);
              //decoding from: https://github.com/merbanan/rtl_433/blob/master/src/devices/emax.c

              //for some reason we are missing the first 0xAA of the payload so needed to move the decoding 1 step
              //(b[31] == b[30]) and add result += 0xAA; to add_bytes to get a valid crc check
              //> Lenght: 32 data A3 8 13 A6 9A A3 3B AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA AA CC AA


              if ((add_bytes(b, 30) & 0xff) != b[30]) {
                ESP_LOGD("rx", "decoder failed");
              }
              else
              {
                rx_counter++;
                int channel     = (b[0] & 0x0f);
                int kind        = ((b[0] & 0xf0) >> 4);
                int id_          = (b[1] << 4) | (b[2] >> 4);
                int battery_low = (b[2] & 0x08);
                int pairing     = (b[2] & 0x04);


                if (kind != 0) {  // if not Rain/Wind ... sensor

                  int temp_raw    = ((b[3] & 0x0f) << 8) | (b[4] & 0xf0) | (b[5] & 0x0f); // weird format
                  float temp_f    = (temp_raw - 900) * 0.1f;
                  float temp    = ((temp_f - 32) * 5) / 9.0;
                  int hum    = b[6];
                  ESP_LOGD("rx", "temp hum sensor");


                  if (channel == 3 && id_ == 129 && kind == 10) //extra checks help when bad data passes crc
                  {

                    fsk_temp_->publish_state(temp);
                    fsk_hum_->publish_state(hum);
                    fsk_rssi_->publish_state(rssi);
                    fsk_battery_low_->publish_state(battery_low); 

                  }


                  ESP_LOGD("th", "tmp %.1f hum %i rssi %i channel %i id %i kind %i battery low %i pairing %i" , temp, hum, rssi, channel, id_, kind, battery_low, pairing);


                }


                else {  // if Rain/Wind sensor
                  ESP_LOGD("rx", "Rain/Wind sensor");

                  int temp_raw      = ((b[3] & 0x0f) << 8) | (b[4]); // weird format
                  float temp_f      = (temp_raw - 900) * 0.1f;
                  float temp    = ((temp_f - 32) * 5) / 9.0;
                  int hum      = b[5];
                  int wind_raw      = (((b[6] - 1) & 0xff) << 8) | ((b[7] - 1) & 0xff);   // need to remove 1 from byte , 0x01 - 1 = 0 , 0x02 - 1 = 1 ... 0xff -1 = 254 , 0x00 - 1 = 255.
                  float speed_ms   = (wind_raw * 0.2f) * 0.277777778;
                  int direction_deg = (((b[8] - 1) & 0x0f) << 8) | ((b[9] - 1) & 0xff);
                  int rain_raw      = (((b[10] - 1) & 0xff) << 8) | ((b[11] - 1) & 0xff);
                  float rain_mm     = rain_raw * 0.2f;



                  if (b[28] == 0x17) {                               // with UV/Lux, without Wind Gust
                        uv_index      = (b[12] - 1) & 0x1f;
                    int lux_14        = (b[13] - 1) & 0xFF;
                    int lux_15        = (b[14] - 1) & 0xFF;
                    int lux_multi     = ((lux_14 & 0x80) >> 7);
                        light_lux     = ((lux_14 & 0x7f) << 8) | (lux_15);
                    if (lux_multi == 1) {
                      light_lux = light_lux * 10;
                    }
                  }
                  if (b[28] == 0x16) {                               //without UV/Lux with Wind Gust
                     gust_ms = (b[15] / 1.5f) * 0.277777778;

                  }

                  ESP_LOGD("th", "tmp %.1f hum %i wind speed %.1f direction %i rain %.1f  rssi %i uv_index %i lux %i gust %.1f channel %i id %i kind %i battery low %i pairing %i " , temp, hum, speed_ms, direction_deg, rain_mm, uv_index, light_lux, gust_ms, channel, id_, kind, battery_low, pairing);





                  if (channel == 3 && id_ == 129) //extra checks help when bad data passes crc
                  {
                    fsk_temp_->publish_state(temp);
                    fsk_hum_->publish_state(hum);
                    fsk_rssi_->publish_state(rssi);
                    fsk_battery_low_->publish_state(battery_low);                 			  
				  }
               

			   }
              }
            }
          }
        }
      }
    
};

#endif





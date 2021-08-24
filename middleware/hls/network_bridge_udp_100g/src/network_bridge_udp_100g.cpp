/************************************************
Copyright (c) 2016, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software 
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.// Copyright (c) 2015 Xilinx, Inc.
************************************************/

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <hls_stream.h>
#include "ap_int.h"
#include <stdint.h>
#include "galapagos_packet.h"

using namespace hls;


ap_uint<32> byteSwap32(ap_uint<32> & inputVector) {

    return (inputVector.range(7,0), inputVector(15, 8), inputVector(23, 16), inputVector(31, 24));
        
}

//ap_uint<512> byteSwap512(ap_uint<512> & inputVector) {
//
//    //return inputVector;
//    return  (inputVector.range(7,0),    inputVector(15, 8),    inputVector(23, 16),   inputVector(31, 24),   inputVector(39,32),   inputVector(47,40),   inputVector(55,48),  inputVector(63,56),
//            inputVector(71,64),   inputVector(79, 72),   inputVector(87, 80),   inputVector(95, 88),   inputVector(103,96),  inputVector(111,104), inputVector(119,112), inputVector(127,120),
//            inputVector(135,128), inputVector(143, 136), inputVector(151, 144), inputVector(159, 152), inputVector(167,160), inputVector(175,168), inputVector(183,176), inputVector(191,184),
//            inputVector(199,192), inputVector(207, 200), inputVector(215, 208), inputVector(223, 216), inputVector(231,224), inputVector(239,232), inputVector(247,240), inputVector(255,248),
//            inputVector(263,256), inputVector(271, 264), inputVector(279, 272), inputVector(287, 280), inputVector(295,288), inputVector(303,296), inputVector(311,304), inputVector(319,312),
//            inputVector(327,320), inputVector(335, 328), inputVector(343, 336), inputVector(351, 344), inputVector(359,352), inputVector(367,360), inputVector(375,368), inputVector(383,376),
//            inputVector(391,384), inputVector(399, 392), inputVector(407, 400), inputVector(415, 408), inputVector(423,416), inputVector(431,424), inputVector(439,432), inputVector(447,440),
//            inputVector(455,448), inputVector(463, 456), inputVector(471, 464), inputVector(479, 472), inputVector(487,480), inputVector(495,488), inputVector(503,496), inputVector(511,504)
//            );
//        
//}

struct axiWord512{
	ap_uint<512>		data;
	ap_uint<16>		keep;
	ap_uint<1>		last;
};

void rxPath(stream<axiWord512>&       lbRxDataIn,
    		stream<axiWord512>& txGalapagosBridge,
            ap_uint<PACKET_DEST_LENGTH> *id,
            ap_uint<PACKET_DEST_LENGTH> *dest,
            ap_uint<PACKET_USER_LENGTH> *size
            ) {
#pragma HLS PIPELINE II=1

    if (!lbRxDataIn.empty() ) {
        axiWord512 currWord = lbRxDataIn.read();

        // debug
        *id = currWord.data.range(PACKET_DEST_LENGTH+PACKET_USER_LENGTH-1, PACKET_USER_LENGTH);
        *dest =  currWord.data.range(PACKET_DEST_LENGTH+PACKET_DEST_LENGTH+PACKET_USER_LENGTH-1, PACKET_DEST_LENGTH+PACKET_USER_LENGTH);
        *size = currWord.data.range(PACKET_USER_LENGTH-1,0);

        txGalapagosBridge.write(currWord);
    }

}


void txPath(
            stream<axiWord512>& rxGalapagosBridge,
    		stream<axiWord512> 	   &lbTxDataOut,
            ap_uint  <32> * remote_ip_tx,
            ap_uint <4> * tx_state,
            ap_uint <2> arp_status,
            const ap_uint<32> ip_table[256]
            ) {
#pragma HLS PIPELINE II=1
	static enum sState {TX_IDLE = 0, WAIT_4_CYCLE, WAIT_ARP_STATUS, TX_WRITE_METADATA, TX_SINGLE_FLIT, TX_WRITE_HEADER, TX_STREAM, TX_LAST} sinkState;
    
    static ap_uint<8> dest;
    static ap_uint<32> dest_ip_addr;
    static axiWord512 header;
    static ap_uint<2> wait_arp_0 = 2;
    static ap_uint<2> wait_arp_1 = 2;
    static ap_uint<2> wait_arp_2 = 2;
    static ap_uint<2> wait_arp_3 = 2;
    static ap_uint<32> old_dest_ip;
    static ap_uint<4> counter;

    *remote_ip_tx = dest_ip_addr;
    *tx_state = sinkState;
    
    switch(sinkState){
        case TX_IDLE:
            if(!rxGalapagosBridge.empty()){

                header = rxGalapagosBridge.read();
                dest = header.data.range(31,24);
                dest_ip_addr = ip_table[dest*4];

                if (dest_ip_addr != old_dest_ip) {
                    old_dest_ip = dest_ip_addr;
                    sinkState = WAIT_4_CYCLE;
                    
                } else {
                    if (!header.last) {
                        sinkState = TX_WRITE_HEADER;
                    } else {
                        sinkState = TX_SINGLE_FLIT;
                    }
                }
            }
            break;

        case WAIT_4_CYCLE:
            if (counter < 4) {
                counter ++;
            } else {
                counter = 0;
                sinkState = WAIT_ARP_STATUS;
            }

            break;
        
        case  WAIT_ARP_STATUS:
            if (arp_status == 0) // arp status == 0, done
            {
                if (!header.last) {
                    sinkState = TX_WRITE_HEADER;
                } else {
                    sinkState = TX_SINGLE_FLIT;
                }
            }
            break;
            
        case  TX_SINGLE_FLIT:
            {
                lbTxDataOut.write(header);
                sinkState = TX_IDLE;
            }
            break;

        case TX_WRITE_HEADER:
            {
                //header.data = byteSwap512(header.data);
                lbTxDataOut.write(header);
                sinkState = TX_STREAM;
            }
            break;

        case TX_STREAM:
            if(!rxGalapagosBridge.empty()){
                axiWord512 currWord;
                currWord = rxGalapagosBridge.read();
                //currWord.data = byteSwap512(currWord.data);
                lbTxDataOut.write(currWord);
                if(currWord.last)
                    sinkState = TX_IDLE;
                else
                    sinkState = TX_STREAM;
            }
            break;
        default:
            sinkState = TX_IDLE;
            break;

    }
    

}

void network_bridge_udp_100g(
                 stream<axiWord512>& rxGalapagosBridge,
                 stream<axiWord512>&       lbRxDataIn,
				 stream<axiWord512> 		&lbTxDataOut,
                 stream<axiWord512>& txGalapagosBridge,
                 ap_uint<32> * remote_ip_tx,
                 ap_uint <4> * tx_state,
                 ap_uint <2> arp_status,
                 const ap_uint<32> ip_table[256],

                 // debug
                 ap_uint<PACKET_DEST_LENGTH> *id,
                ap_uint<PACKET_DEST_LENGTH> *dest,
                ap_uint<PACKET_USER_LENGTH> *size
                 ) {
	#pragma HLS INTERFACE ap_ctrl_none port=return
	#pragma HLS INTERFACE ap_none port=remote_ip_tx
	#pragma HLS DATAFLOW


    #pragma HLS INTERFACE AXIS port=lbRxDataIn
    #pragma HLS INTERFACE AXIS port=lbTxDataOut
    #pragma HLS INTERFACE AXIS port=rxGalapagosBridge
    #pragma HLS INTERFACE AXIS port=txGalapagosBridge
    #pragma HLS INTERFACE bram port=ip_table 

    rxPath(lbRxDataIn, txGalapagosBridge, id, dest, size);
    txPath(rxGalapagosBridge, lbTxDataOut, remote_ip_tx, tx_state, arp_status, ip_table);
}

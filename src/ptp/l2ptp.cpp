#include <QNEthernet.h>

#include "l2ptp.h"

using namespace qindesign::network;

l2PTP::l2PTP(bool master_, bool slave_, bool p2p_):
PTPBase(master_,slave_,p2p_)
{

}

void l2PTP::initSockets()
{
    uint8_t mac[6];
    mac[0] = 0x01;
    mac[1] = 0x80;
    mac[2] = 0xc2;
    mac[3] = 0x00;
    mac[4] = 0x00;
    mac[5] = 0x0e;
    qindesign::network::Ethernet.setMACAddressAllowed(mac, true);
    mac[0] = 0x01;
    mac[1] = 0x1b;
    mac[2] = 0x19;
    mac[3] = 0x00;
    mac[4] = 0x00;
    mac[5] = 0x00;
    qindesign::network::Ethernet.setMACAddressAllowed(mac, true);

}

void l2PTP::updateSockets()
{
    const int bufferSize = qindesign::network::EthernetFrame.parseFrame();
    if (bufferSize <= 0)
    {
        return;
    }

    uint8_t buf[bufferSize];
    //Serial.printf("BufferSize:%d\n",bufferSize);
    const int frameSize = qindesign::network::EthernetFrame.read(buf, bufferSize);

    if (frameSize < qindesign::network::EthernetFrame.minFrameLen() - 4)
    {
        //Serial.printf("SHORT Frame[%d]: \n", frameSize);
        return;
    }

    //Serial.printf("Frame[%d]: dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x", frameSize, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
    const uint16_t type = (uint16_t{buf[12]} << 8) | buf[13];

    if(type != 0x88f7){
        return;
    }

    timespec recv_ts;
    EthernetFrame.timestamp(recv_ts);

    const int payloadStart = 14;
    const int payloadSize = frameSize - payloadStart;
    //Serial.printf("PayloadSize:%d\n",payloadSize);

    const uint8_t* payloadBuf = &buf[payloadStart];
    
    /*for (int i = 0; i < payloadSize; i++)
    {
        Serial.printf(" %02x", payloadBuf[i]);
    }
    Serial.printf("\n");*/

    parsePTPMessage(payloadBuf, payloadSize, recv_ts);
}

void l2PTP::sendPTPMessage(const uint8_t *buf, int size, bool generalMessage)
{
    uint8_t srcmac[6];
    qindesign::network::Ethernet.macAddress(srcmac);
    uint8_t dstmac[6];
    dstmac[0] = 0x01;
    dstmac[1] = 0x80;
    dstmac[2] = 0xc2;
    dstmac[3] = 0x00;
    dstmac[4] = 0x00;
    dstmac[5] = 0x0e;

    qindesign::network::EthernetFrame.beginFrame(dstmac,srcmac,(uint16_t)0x88f7);
    int w=qindesign::network::EthernetFrame.write(buf,size);
    const int fill = 46-w;
    if(fill>0){
        uint8_t buf0[fill] = {0};
        w+=qindesign::network::EthernetFrame.write(buf0,fill);
    }
    bool sent=qindesign::network::EthernetFrame.endFrame();
    /*if(sent){
        Serial.printf("Sent %d\n",w);
    }else{
        Serial.printf("Sent failed %d\n",w);
    }*/
}

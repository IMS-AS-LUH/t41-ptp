#include <QNEthernet.h>

#include "l3ptp.h"

const IPAddress adr{224, 0, 1, 129};
const IPAddress pAdr{224, 0, 0, 107};
const int eventPort = 319;
const int generalPort = 320;

l3PTP::l3PTP(bool master_, bool slave_, bool p2p_):
PTPBase(master_,slave_,p2p_)
{

}

void l3PTP::initSockets()
{
    eventSocket = new qindesign::network::EthernetUDP;
    generalSocket = new qindesign::network::EthernetUDP;

    eventSocket->beginMulticast(adr, eventPort, true);
    generalSocket->beginMulticast(adr, generalPort, true);

    if(p2p){
        pEventSocket = new qindesign::network::EthernetUDP;
        pGeneralSocket = new qindesign::network::EthernetUDP;
        pEventSocket->beginMulticast(pAdr, eventPort, true);
        pGeneralSocket->beginMulticast(pAdr, generalPort, true);
    }
}

void l3PTP::updateSockets()
{
    const int esize = eventSocket->parsePacket();
    if (esize > 0)
    {
        timespec erecv_ts;
        eventSocket->timestamp(erecv_ts);
        uint8_t ebuf[esize];
        eventSocket->read(ebuf, esize);
        parsePTPMessage(ebuf,esize,erecv_ts);
    }

    const int gsize = generalSocket->parsePacket();
    if (gsize > 0)
    {
        timespec grecv_ts;
        generalSocket->timestamp(grecv_ts);
        uint8_t gbuf[gsize];
        generalSocket->read(gbuf, gsize);
        parsePTPMessage(gbuf,gsize,grecv_ts);
    }
    if(p2p){
        const int esize = pEventSocket->parsePacket();
        if (esize > 0)
        {
            timespec erecv_ts;
            pEventSocket->timestamp(erecv_ts);
            uint8_t ebuf[esize];
            pEventSocket->read(ebuf, esize);
            parsePTPMessage(ebuf,esize,erecv_ts);
        }

        const int gsize = pGeneralSocket->parsePacket();
        if (gsize > 0)
        {
            timespec grecv_ts;
            pGeneralSocket->timestamp(grecv_ts);
            uint8_t gbuf[gsize];
            pGeneralSocket->read(gbuf, gsize);
            parsePTPMessage(gbuf,gsize,grecv_ts);
        }
    }
}

void l3PTP::sendPTPMessage(const uint8_t *buf, int size, bool generalMessage){
    if(generalMessage){
        generalSocket->send(adr,generalPort,buf,size);
    }else{
        eventSocket->send(adr,eventPort,buf,size);
    }
}

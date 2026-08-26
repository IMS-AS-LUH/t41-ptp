#include <QNEthernet.h>

#include "l3ptp.h"

// The most we read from a datagram. The longest PTP message this library
// parses is 54 bytes; anything beyond that is TLVs nothing here looks at.
//
// A fixed buffer, not a variable-length one. It used to be
// `uint8_t buf[esize]`, with esize straight from parsePacket(): the size
// of what went on the stack was decided by whoever sent the packet.
// Bounded by the frame in the normal case, but IP_REASSEMBLY is on by
// default in lwIP and is not turned off in QNEthernet's lwipopts.h, so a
// fragmented datagram can arrive larger.
static constexpr int PTP_RECV_BUF_LEN = 64;

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
        uint8_t ebuf[PTP_RECV_BUF_LEN];
        const int en = esize > PTP_RECV_BUF_LEN ? PTP_RECV_BUF_LEN : esize;
        eventSocket->read(ebuf, en);
        parsePTPMessage(ebuf,en,erecv_ts);
    }

    const int gsize = generalSocket->parsePacket();
    if (gsize > 0)
    {
        timespec grecv_ts;
        generalSocket->timestamp(grecv_ts);
        uint8_t gbuf[PTP_RECV_BUF_LEN];
        const int gn = gsize > PTP_RECV_BUF_LEN ? PTP_RECV_BUF_LEN : gsize;
        generalSocket->read(gbuf, gn);
        parsePTPMessage(gbuf,gn,grecv_ts);
    }
    if(p2p){
        const int esize = pEventSocket->parsePacket();
        if (esize > 0)
        {
            timespec erecv_ts;
            pEventSocket->timestamp(erecv_ts);
            uint8_t ebuf[PTP_RECV_BUF_LEN];
            const int en = esize > PTP_RECV_BUF_LEN ? PTP_RECV_BUF_LEN : esize;
            pEventSocket->read(ebuf, en);
            parsePTPMessage(ebuf,en,erecv_ts);
        }

        const int gsize = pGeneralSocket->parsePacket();
        if (gsize > 0)
        {
            timespec grecv_ts;
            pGeneralSocket->timestamp(grecv_ts);
            uint8_t gbuf[PTP_RECV_BUF_LEN];
            const int gn = gsize > PTP_RECV_BUF_LEN ? PTP_RECV_BUF_LEN : gsize;
            pGeneralSocket->read(gbuf, gn);
            parsePTPMessage(gbuf,gn,grecv_ts);
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

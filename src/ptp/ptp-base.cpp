#include <Arduino.h>
#include <QNEthernet.h>
#include <TimeLib.h>
#include "ptp-base.h"

const int logging = 0;
const int hwOffset = -200; // Hardware Offset

void printTime(const NanoTime t)
{
    NanoTime x = t;
    const int ns = x % 1000;
    x /= 1000;
    const int us = x % 1000;
    x /= 1000;
    const int ms = x % 1000;
    x /= 1000;

    tmElements_t tme;
    breakTime((time_t)x, tme);

    Serial.printf("%02d.%02d.%04d %02d:%02d:%02d::%03d:%03d:%03d\n", tme.Day, tme.Month, 1970 + tme.Year, tme.Hour, tme.Minute, tme.Second, ms, us, ns);
}

NanoTime timespecToNanoTime(const timespec &tm)
{
    const NanoTime s = tm.tv_sec;
    const NanoTime ns = tm.tv_nsec;
    return (s * NS_PER_S) + ns;
}

NanoTime bufferToNanoTime(const uint8_t *buf)
{
    NanoTime s = ((NanoTime)buf[34]) << 40;
    s += ((NanoTime)buf[35]) << 32;
    s += ((NanoTime)buf[36]) << 24;
    s += ((NanoTime)buf[37]) << 16;
    s += ((NanoTime)buf[38]) << 8;
    s += ((NanoTime)buf[39]);
    NanoTime ns = ((NanoTime)buf[40]) << 24;
    ns += ((NanoTime)buf[41]) << 16;
    ns += ((NanoTime)buf[42]) << 8;
    ns += ((NanoTime)buf[43]);
    return (s * NS_PER_S) + ns;
}

NanoTime bufferToCorrection(const uint8_t *buf)
{
    NanoTime ns = ((NanoTime)buf[8]) << 56;
    ns += ((NanoTime)buf[9]) << 48;
    ns += ((NanoTime)buf[10]) << 40;
    ns += ((NanoTime)buf[11]) << 32;
    ns += ((NanoTime)buf[12]) << 24;
    ns += ((NanoTime)buf[13]) << 16;
    ns += ((NanoTime)buf[14]) << 8;
    ns += ((NanoTime)buf[15]);
    return (ns >> 16);
}

void timespecToBuffer(const timespec &tm, uint8_t *buf)
{
    const NanoTime s = tm.tv_sec;
    const NanoTime ns = tm.tv_nsec;

    buf[34] = (s >> 40)  & 0xff;
    buf[35] = (s >> 32)  & 0xff;
    buf[36] = (s >> 24)  & 0xff;
    buf[37] = (s >> 16)  & 0xff;
    buf[38] = (s >> 8)   & 0xff;
    buf[39] =  s         & 0xff;
    buf[40] = (ns >> 24) & 0xff;
    buf[41] = (ns >> 16) & 0xff;
    buf[42] = (ns >> 8)  & 0xff;
    buf[43] =  ns        & 0xff;
}

void nanoTimeToTimespec(const NanoTime t, timespec &tm)
{
    const NanoTime ns = t % NS_PER_S;
    const NanoTime s = t / NS_PER_S;
    tm.tv_sec = s;
    tm.tv_nsec = ns;
}

PTPBase::PTPBase(bool master_, bool slave_, bool p2p_):
master(master_),
slave(slave_),
p2p(p2p_)
{

}

void PTPBase::begin()
{
    reset();
    if (initialised)
    {
        return;
    }
    initSockets();

    initialised = true;

    if (logging)
    {
        Serial.println("PTP Started");
    }
}

void PTPBase::update()
{
    if (initialised)
    {
        updateSockets();
        if (syncSequenceID > 0 && followUpSequenceID > 0 && syncSequenceID == followUpSequenceID)
        {
            syncSequenceID = 0;
            followUpSequenceID = 0;
            delayRequestMessage();
        }
        bool allTimestampsUpdated=t1updated && t2updated && t3updated && t4updated;
        if(p2p){
            allTimestampsUpdated&=t5updated && t6updated;
        }
        if (allTimestampsUpdated)
        {
            t1updated = false;
            t2updated = false;
            t3updated = false;
            t4updated = false;
            t5updated = false;
            t6updated = false;
            if (t1lastvalid && t2lastvalid && syncSequenceID == followUpSequenceID)
            {
                t1lastvalid = false;
                t2lastvalid = false;
                updateTimer();
            }
        }
        
        if (ppsupdated && t1lastvalid && t2lastvalid)
		{
			ppsupdated = false;
		    t1lastvalid = false;
		    t2lastvalid = false;
		    updatePPS();
		}
    }
}

void PTPBase::reset()
{
    if (logging)
    {
        Serial.println("Reset PTP state");
    }
    clockID[0] = 0xFF;
    clockID[1] = 0xFF;
    qindesign::network::Ethernet.macAddress(clockID + 2);

    timespec tm;
    tm.tv_sec = 0;
    tm.tv_nsec = 0;
    qindesign::network::EthernetIEEE1588.writeTimer(tm);

    t1 = -1;
    t1last = -1;
    t2 = -1;
    t2last = -1;
    t2new = -1;
    t3 = -1;
    t4 = -1;
    t5 = -1;
    t6 = -1;

    t1updated = false;
    t2updated = false;
    t3updated = false;
    t4updated = false;
    t5updated = false;
    t6updated = false;
    t1lastvalid = false;
    t2lastvalid = false;

    nspsAccu = 0;
    driftNSPS = 0;
}

void PTPBase::setKi(double val)
{
    KI = val;
    reset();
}

void PTPBase::setKp(double val)
{
    KP = val;
    reset();
}

void PTPBase::updateController()
{
    const double t1diff = (t1 - t1last);
    const double t2diff = (t2 - t2last);
    const double currentDrift = t2diff / t1diff;
    const double currentDriftNsps = (1.0 - currentDrift) * NS_PER_S;
    
    // Max XTAL drift should be around 30ppm (30000ns/s). If drift is much higher (100000ns/s), master clock is most likely invalid due to severe adjustments
    const bool driftError = currentDriftNsps > 100000 || currentDriftNsps < -100000; 
    
    const bool freqMode = currentDriftNsps > 1000 || currentDriftNsps < -1000;
    
    const bool coarseMode = currentOffset > 1000 || currentOffset < -1000;
    const NanoTime offsetCorrection = -currentOffset;

    double nspsAdjust = 0;
    double nspsAdjustC = 0;
    double nspsAdjustP = 0;
    double nspsAdjustI = 0;

    if(!driftError){
        if(freqMode)
        {
            driftNSPS = fmod(driftNSPS + currentDriftNsps,NS_PER_S);
            nspsAdjustC = driftNSPS;
            nspsAdjust = nspsAdjustC;
            qindesign::network::EthernetIEEE1588.adjustFreq(nspsAdjust);
            nspsAccu = 0;
        }
        else if (coarseMode)
        {
            qindesign::network::EthernetIEEE1588.offsetTimer(offsetCorrection);

            nspsAccu = 0;
            t2 += offsetCorrection;
        }
        else
        {
            nspsAccu += offsetCorrection;
            nspsAdjustC = driftNSPS;
            nspsAdjustP = (static_cast<double>(offsetCorrection) * KP);
            nspsAdjustI = (static_cast<double>(nspsAccu) * KI);
            nspsAdjust = nspsAdjustC + nspsAdjustP + nspsAdjustI;
            qindesign::network::EthernetIEEE1588.adjustFreq(nspsAdjust);
            t2 += offsetCorrection;
        }
    
        if(!freqMode && !coarseMode && currentOffset < 100 && currentOffset > -100){
        	lockcount++;
        }else{
        	lockcount=0;
        }
    }
    else
    {
        lockcount=0; //no lock if in driftError state
    }

    if (logging)
    {

        Serial.printf("T2diff:%f T1diff:%f\n", t2diff, t1diff);
        Serial.printf("T2-T1:%d T4-T3:%d\n", (int)(t2 - t1), (int)(t4 - t3));
        Serial.printf("Delay:%dns ", (int)currentDelay);
        Serial.printf("Offset:%dns ", (int)currentOffset);
        Serial.printf("Drift:%dns \n", (int)currentDriftNsps);

        if(driftError)
        {
            Serial.printf("Drift Error\n No controller update.\n");
        }
        else if(freqMode)
        {
            Serial.printf("Freq mode adjust f: %f ns/s", nspsAdjust);
        }
        else if (coarseMode)
        {
            Serial.printf("Coarse mode adjust:%d ns", (int)offsetCorrection);
        }
        else
        {
            Serial.printf("Fine filter mode ns/s:%f C:%f P(%f):%f I(%f):%f", nspsAdjust, nspsAdjustC, KP, nspsAdjustP, KI, nspsAdjustI);
        }

        Serial.println();
        if(!driftError)
        {
            Serial.printf("ENET_ATINC %08X\n", ENET_ATINC);
            Serial.printf("ENET_ATPER %d\n", ENET_ATPER);
            Serial.printf("ENET_ATCOR %d (%f)\n", ENET_ATCOR, 25000000 / nspsAdjust);
        }
        Serial.println();
        Serial.println();
        Serial.println();
    }
    else
    {
        //Serial.printf("%d %d %d %d %d %f %f %f %f\n",updateCounter,coarseMode ? 0 : 1,(int)currentDelay, (int)currentOffset, (int)currentDriftNsps, nspsAdjustC, nspsAdjustP, nspsAdjustI, tempmonGetTemp());
    }
    updateCounter++;
}

int PTPBase::getLockCount()
{
	return lockcount;
}

void PTPBase::updateTimer()
{
    if (logging >= 2)
    {
        Serial.println("NEW DATA");
    }
    if(p2p){
        currentDelay = ((t6 - t3) - (t5 - t4)) / 2;
        currentOffset = (t2 - t1) - currentDelay + 500;
    }else{
        currentDelay = ((t4 - t1) - (t3 - t2)) / 2;
        currentOffset = (t2 - t1) - currentDelay + hwOffset;
    }
    
    updateController();
}

void PTPBase::updatePPS()
{
    currentDelay=0;
    currentOffset = (t2 - t1) - currentDelay;
    updateController();
}

NanoTime PTPBase::getOffset()
{
    return currentOffset;
}
NanoTime PTPBase::getDelay()
{
    return currentDelay;
}

void PTPBase::parsePTPMessage(const uint8_t *buf, int size, const timespec &recv_ts)
{
    const uint8_t messageType = buf[0] & 0x0f;
    const uint8_t versionPTP = buf[1] & 0x0f;
    const uint8_t domainNumer = buf[4];
    if(versionPTP==2){
        if (logging >= 2)
        {
            Serial.printf("PTPMessage messageType:%d versionPTP:%d domainNumer:%d\n", messageType, versionPTP, domainNumer);
        }
        if(slave && messageType==0){
            parseSyncMessage(buf,recv_ts);
        }else if(master && messageType==1){
            parseDelayRequestMessage(buf,recv_ts);
        }
        else if(slave && messageType == 8){
            parseFollowUpMessage(buf);
        }else if(slave && messageType==3 || messageType==9){
            parseDelayResponseMessage(buf,recv_ts);
        }else if(slave && messageType==10){
            parseDelayResponseFollowUpMessage(buf);
        }
    }
}


void PTPBase::setT2(NanoTime ts){
    t2new = ts;
    
    if (logging)
    {
        Serial.print("T2 Sync  receive timestamp=");
        printTime(t2new);
    }
}

void PTPBase::parseSyncMessage(const uint8_t *buf, const timespec &recv_ts)
{
    const uint8_t twoStepFlag = buf[6] & 0x02;
    const uint16_t sequenceID = (buf[30] << 8) | buf[31];

    if (twoStepFlag > 0 && sequenceID > 0) // Sync twoStep
    {	
    	setT2(timespecToNanoTime(recv_ts));
    	syncSequenceID = sequenceID;
    }
}

void PTPBase::setT1(NanoTime ts){
	t1last = t1;
    t1lastvalid = t1last > 0;
    t1 = ts;
    t1updated = true;
    
    t2last = t2; // Update T2 only if valid T1 data was received. Otherwise T2last and T1last might not be frrom the same sequenceID
    t2lastvalid = t2last > 0;
    t2 = t2new;
    t2updated = true;

    if (logging)
    {
        Serial.print("T1 Sync  send    timestamp=");
        printTime(t1);
    }
}

void PTPBase::parseFollowUpMessage(const uint8_t *buf)
{
    const uint16_t sequenceID = (buf[30] << 8) | buf[31];
    if(sequenceID > 0 && sequenceID == syncSequenceID)
    {
        followUpSequenceID = sequenceID;
        setT1(bufferToNanoTime(buf)+bufferToCorrection(buf));
        if (logging > 1)
        {
            Serial.printf("T1 corrected by %" PRId64 "\n", bufferToCorrection(buf));
        }
    }
}

void PTPBase::setT4(NanoTime ts){
	t4 = ts;
    t4updated = true;
    if (logging)
    {
        Serial.print("T4 Delay receive timestamp=");
        printTime(t4);
    }
}

void PTPBase::parseDelayResponseMessage(const uint8_t *buf, const timespec &recv_ts)
{
    const bool requestingPortIdentityMatch = (buf[44] == clockID[0]) && (buf[45] == clockID[1]) && (buf[46] == clockID[2]) && (buf[47] == clockID[3]) &&
                                             (buf[48] == clockID[4]) && (buf[49] == clockID[5]) && (buf[50] == clockID[6]) && (buf[51] == clockID[7]);
    if (requestingPortIdentityMatch)
    {
        setT4(bufferToNanoTime(buf)-bufferToCorrection(buf));
        t6 = timespecToNanoTime(recv_ts);
        t6updated = true;
        if (logging > 1)
        {
            Serial.printf("T4 corrected by -%" PRId64 "\n", bufferToCorrection(buf));
        }
        if (logging)
        {
            if(p2p){
                Serial.print("T6 Resp  receive timestamp=");
                printTime(t6);
            }else{
                Serial.println("");
            }
        }
    }
}

void PTPBase::parseDelayResponseFollowUpMessage(const uint8_t *buf)
{
    const bool requestingPortIdentityMatch = (buf[44] == clockID[0]) && (buf[45] == clockID[1]) && (buf[46] == clockID[2]) && (buf[47] == clockID[3]) &&
                                             (buf[48] == clockID[4]) && (buf[49] == clockID[5]) && (buf[50] == clockID[6]) && (buf[51] == clockID[7]);
    if (requestingPortIdentityMatch)
    {
        t5 = bufferToNanoTime(buf);
        t5updated = true;
        if (logging)
        {
            Serial.print("T5 Resp  send    timestamp=");
            printTime(t5);
            Serial.println("");
        }
    }    
}

void PTPBase::parseDelayRequestMessage(const uint8_t *buf, const timespec &recv_ts)
{
    const uint16_t sequenceID = (buf[30] << 8) | buf[31];
    t4s = timespecToNanoTime(recv_ts)+ hwOffset;
    timespec ts;
    nanoTimeToTimespec(t4s,ts);
    if (logging)
    {
        Serial.print("T4s Delay receiv timestamp=");
        printTime(t4s);
    }
    delayResponseMessage(buf,sequenceID,ts);
}

void PTPBase::delayResponseMessage(const uint8_t *request_buf, uint16_t sequenceID, const timespec &request_recv_ts)
{
    if(!initialised){
        return;
    }
    uint16_t size=54;
    uint8_t type=9;
    uint8_t control=3;
   
    uint8_t buf[size] = {0};

    initPTPMessage(buf, size, type, sequenceID, control);
    buf[33]=0;
    timespecToBuffer(request_recv_ts,buf);
    buf[44] = request_buf[20];
    buf[45] = request_buf[21];
    buf[46] = request_buf[22];
    buf[47] = request_buf[23];
    buf[48] = request_buf[24];
    buf[49] = request_buf[25];
    buf[50] = request_buf[26];
    buf[51] = request_buf[27];
    buf[52] = request_buf[28];
    buf[53] = request_buf[29];
    sendPTPMessage(buf,size,true);
}

void PTPBase::announceMessage()
{
    if(!initialised || !master){
        return;
    }
    uint16_t size=64;
    uint8_t type=11;
    uint8_t control=5;
   
    uint8_t buf[size] = {0};

    initPTPMessage(buf, size, type, announceServerSequenceID++, control);
    buf[7]=8;//PTPTimescale
    buf[33]=0;
    buf[45]=37;//UTCOffset
    buf[47]=128;
    buf[48]=248;
    buf[49]=0xfe;
    buf[50]=0xff;
    buf[51]=0xff;
    buf[52]=128;
    buf[53] = clockID[0];
    buf[54] = clockID[1];
    buf[55] = clockID[2];
    buf[56] = clockID[3];
    buf[57] = clockID[4];
    buf[58] = clockID[5];
    buf[59] = clockID[6];
    buf[60] = clockID[7];
    buf[63] = 0xa0;
    sendPTPMessage(buf,size,true);
}

void PTPBase::syncMessage()
{
    if(!initialised || !master){
        return;
    }
    uint16_t size=44;
    uint8_t type=0;
    uint8_t control=0;
   
    uint8_t buf[size] = {0};

    initPTPMessage(buf, size, type, syncServerSequenceID, control);
    buf[6] =2;
    buf[33]=0;
    qindesign::network::EthernetIEEE1588.timestampNextFrame();
    sendPTPMessage(buf,size,false);
    

    struct timespec send_ts;
    if (logging >= 2)
    {
        Serial.print("Wait for T1s Delay send timestamp");
    }
    while (!qindesign::network::EthernetIEEE1588.readAndClearTxTimestamp(send_ts))
    {
        if (logging >= 2)
        {
            Serial.print(".");
        }
    }
    if (logging >= 2)
    {
        Serial.println(" finished");
    }
    t1s = timespecToNanoTime(send_ts)+ hwOffset;
    nanoTimeToTimespec(t1s,send_ts);
    if (logging)
    {
        Serial.print("T1s Delay send   timestamp=");
        printTime(t1s);
    }
    followUpMessage(send_ts);
    syncServerSequenceID++;
}

void PTPBase::followUpMessage(const timespec &send_ts)
{
    if(!initialised || !master){
        return;
    }
    uint16_t size=44;
    uint8_t type=8;
    uint8_t control=2;
   
    uint8_t buf[size] = {0};

    initPTPMessage(buf, size, type, syncServerSequenceID, control);
    buf[33]=0;
    timespecToBuffer(send_ts,buf);
    sendPTPMessage(buf,size,true);
}

void PTPBase::setT3(NanoTime ts){
	t3 = ts;
    t3updated = true;
    if (logging)
    {
        Serial.print("T3 Delay send    timestamp=");
        printTime(t3);
    }
}

void PTPBase::delayRequestMessage()
{
    uint16_t size;
    uint8_t type;
    uint8_t control;
    if(!p2p){
        type=1;
        size=44;
        control=1;
    }else{
        type=2;
        size=54;
        control=5;
    }
    uint8_t buf[size] = {0};

    initPTPMessage(buf, size, type, delayRequestSequenceID, control);
    qindesign::network::EthernetIEEE1588.timestampNextFrame();
    sendPTPMessage(buf,size,false);
    delayRequestSequenceID++;

    struct timespec send_ts;
    if (logging >= 2)
    {
        Serial.print("Wait for T3 Delay send timestamp");
    }
    while (!qindesign::network::EthernetIEEE1588.readAndClearTxTimestamp(send_ts))
    {
        if (logging >= 2)
        {
            Serial.print(".");
        }
    }
    if (logging >= 2)
    {
        Serial.println(" finished");
    }
    setT3(timespecToNanoTime(send_ts));
}

void PTPBase::ppsInterruptTriggered(NanoTime pps_ts, NanoTime local_ts){
	if(!initialised || !master){
		return;
	}
	setT2(local_ts);
	setT1(pps_ts);
	
	ppsupdated=true;
}

void PTPBase::initPTPMessage(uint8_t *buf, const uint16_t messageLength, const uint8_t messageType, const uint16_t sequenceID, const uint8_t controlField)
{
    buf[0] = messageType;
    buf[1] = 2;
    buf[2] = (messageLength >> 8) & 0x00ff;
    buf[3] = messageLength & 0x00ff;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = 0;
    buf[9] = 0;
    buf[10] = 0;
    buf[11] = 0;
    buf[12] = 0;
    buf[13] = 0;
    buf[14] = 0;
    buf[15] = 0;
    buf[16] = 0;
    buf[17] = 0;
    buf[18] = 0;
    buf[19] = 0;
    buf[20] = clockID[0];
    buf[21] = clockID[1];
    buf[22] = clockID[2];
    buf[23] = clockID[3];
    buf[24] = clockID[4];
    buf[25] = clockID[5];
    buf[26] = clockID[6];
    buf[27] = clockID[7];
    buf[28] = 0;
    buf[29] = 1;
    buf[30] = (sequenceID >> 8) & 0x00ff;
    buf[31] = sequenceID & 0x00ff;
    buf[32] = controlField;
    buf[33] = 0x7f;
}

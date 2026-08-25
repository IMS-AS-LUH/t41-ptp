#pragma once

using NanoTime = int64_t;

constexpr NanoTime NS_PER_S = 1000*1000*1000;

class PTPBase
{
public:
    PTPBase(bool master_, bool slave_, bool p2p_);
    void begin();
    void update();
    void reset();
    void setKi(double val);
    void setKp(double val);
    NanoTime getOffset();
    NanoTime getDelay();
    void syncMessage();
    void announceMessage();
    void ppsInterruptTriggered(NanoTime pps_ts, NanoTime local_ts);
    int getLockCount();

    // Announce dataset and advertised message intervals.
    //
    // The defaults below are exactly the values this library hardcoded before
    // these became configurable, so leaving them alone changes nothing.
    //
    // A grandmaster disciplined by an external reference needs to say so:
    // with clockClass fixed at 248 and timeSource at INTERNAL_OSCILLATOR it
    // announces itself as free-running whatever its actual lock state, which
    // both misinforms the BMCA and makes it impossible for receivers to tell
    // a locked clock from one that has lost its reference.
    void setClockClass(uint8_t val);
    void setClockAccuracy(uint8_t val);
    void setOffsetScaledLogVariance(uint16_t val);
    void setPriority1(uint8_t val);
    void setPriority2(uint8_t val);
    void setTimeSource(uint8_t val);
    void setCurrentUtcOffset(int16_t val);

    // logMessageInterval as advertised in the PTP header. These only set what
    // is announced -- the caller still has to send at the matching rate, or
    // the announcement lies.
    void setLogSyncInterval(int8_t val);
    void setLogAnnounceInterval(int8_t val);

protected:
    virtual void initSockets()=0;
    virtual void updateSockets()=0;
    virtual void sendPTPMessage(const uint8_t *buf, int size, bool generalMessage)=0;
    
    void parsePTPMessage(const uint8_t *buf, int size, const timespec &recv_ts);
    bool master;
    bool slave;
    bool p2p;
    
private:
	void setT1(NanoTime ts);
	void setT2(NanoTime ts);
	void setT3(NanoTime ts);
	void setT4(NanoTime ts);
    void parseSyncMessage(const uint8_t *buf, const timespec &recv_ts);
    void parseFollowUpMessage(const uint8_t *buf);
    void parseDelayResponseMessage(const uint8_t *buf, const timespec &recv_ts);
    void parseDelayResponseFollowUpMessage(const uint8_t *buf);
    void parseDelayRequestMessage(const uint8_t *buf, const timespec &recv_ts);
    
    void delayRequestMessage();
    void followUpMessage(const timespec &send_ts);
    void delayResponseMessage(const uint8_t *request_buf, uint16_t sequenceID, const timespec &request_recv_ts);
    void initPTPMessage(uint8_t *buf, const uint16_t messageLength, const uint8_t messageType, const uint16_t sequenceID, const uint8_t controlField);
    void updateController();
    void updateTimer();
    void updatePPS();

    uint8_t clockID[8];
    bool initialised=false;

    uint8_t clockClass = 248;          // default, not traceable
    uint8_t clockAccuracy = 0xfe;      // unknown
    uint16_t offsetScaledLogVariance = 0xffff;  // unknown
    uint8_t priority1 = 128;
    uint8_t priority2 = 128;
    uint8_t timeSource = 0xa0;         // INTERNAL_OSCILLATOR
    int16_t currentUtcOffset = 37;
    int8_t logSyncInterval = 0;        // 1 s
    int8_t logAnnounceInterval = 0;    // 1 s
    uint16_t delayRequestSequenceID = 0;
	int lockcount=0;
    uint16_t syncSequenceID=0;
    uint16_t syncServerSequenceID = 0;
    uint16_t announceServerSequenceID = 0;
    uint16_t followUpSequenceID=0;
    NanoTime t1=-1;
    NanoTime t1last = -1;
    NanoTime t2 = -1;
    NanoTime t2last = -1;
    NanoTime t2new = -1;
    NanoTime t3 = -1;
    NanoTime t4 = -1;
    NanoTime t5 = -1;
    NanoTime t6 = -1;

    NanoTime t1s = -1;
    NanoTime t4s = -1;

    bool t1updated=false;
    bool t2updated=false;
    bool t3updated=false;
    bool t4updated=false;
    bool t5updated=false;
    bool t6updated=false;
    bool t1lastvalid=false;
    bool t2lastvalid=false;
    bool ppsupdated=false;

    NanoTime currentOffset=0;
    NanoTime currentDelay=0;
    int nspsAccu=0;
    double driftNSPS=0;
    double KI=0.5;
    double KP=1.0;
    int updateCounter=0;
    
};

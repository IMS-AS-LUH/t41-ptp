#pragma once

#include "ptp-base.h"

class l2PTP : public PTPBase
{
public:
    l2PTP(bool master_, bool slave_, bool p2p_);
private:
    void initSockets() override;
    void updateSockets() override;
    void sendPTPMessage(const uint8_t *buf, int size, bool generalMessage) override;
};
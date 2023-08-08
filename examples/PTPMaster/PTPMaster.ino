#include <t41-ptp.h>
#include <QNEthernet.h>

byte mac[6];
IPAddress staticIP{192, 168, 0, 211};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{192, 168, 0, 6};

IntervalTimer syncTimer;
IntervalTimer announceTimer;

bool master=true;
bool slave=false;

l3PTP ptp(master,slave,false);
//l2PTP ptp(master,slave,p2p);

void setup()
{
Serial.begin(2000000);
  pinMode(13, OUTPUT);

  // Setup networking
  qindesign::network::Ethernet.setHostname("t41ptpslave");
  qindesign::network::Ethernet.macAddress(mac);
  qindesign::network::Ethernet.begin(staticIP, subnetMask, gateway);
  qindesign::network::EthernetIEEE1588.begin();
  
  qindesign::network::Ethernet.onLinkState([](bool state) {
    Serial.printf("[Ethernet] Link %dMbps %s\n", qindesign::network::Ethernet.linkSpeed(), state ? "ON" : "OFF");
    if (state) {
      ptp.begin();
      syncTimer.begin(syncInterrupt, 1000000);
      announceTimer.begin(announceInterrupt, 1000000);
    }
  });

  Serial.printf("Mac address:   %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print( "IP:            "); Serial.println(qindesign::network::Ethernet.localIP());
  Serial.println();

  // PPS Out
  // peripherial: ENET_1588_EVENT1_OUT
  // IOMUX: ALT6
  // teensy pin: 24
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 = 6;
  qindesign::network::EthernetIEEE1588.setChannelCompareValue(1, NS_PER_S-60);
  qindesign::network::EthernetIEEE1588.setChannelMode(1, qindesign::network::EthernetIEEE1588.TimerChannelModes::kPulseHighOnCompare);
  qindesign::network::EthernetIEEE1588.setChannelOutputPulseWidth(1, 25);
  

  // PPS-IN
  // peripherial: ENET_1588_EVENT2_IN
  // IOMUX: ALT4
  // teensy pin: 15
  attachInterruptVector(IRQ_ENET_TIMER, interrupt_1588_timer); //Configure Interrupt Handler
  NVIC_ENABLE_IRQ(IRQ_ENET_TIMER); //Enable Interrupt Handling
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 4; 
  qindesign::network::EthernetIEEE1588.setChannelMode(2, qindesign::network::EthernetIEEE1588.TimerChannelModes::kCaptureOnRising); //enable Channel2 rising edge trigger
  qindesign::network::EthernetIEEE1588.setChannelInterruptEnable(2, true); //Configure Interrupt generation
}

bool sync=false;
bool announce=false;
bool pps=false;
int noPPSCount=0;

NanoTime interrupt_s=0;
NanoTime interrupt_ns=0;
NanoTime pps_s=0;
NanoTime pps_ns=0;

void loop()
{
  if(announce){
    announce=false;
    ptp.announceMessage();
  }
  if(sync){
    sync=false;
    ptp.syncMessage();
  }
  if(pps){
    pps=false;
    ptp.ppsInterruptTriggered((pps_s*NS_PER_S)+pps_ns, (interrupt_s*NS_PER_S)+interrupt_ns);
    if(ptp.getLockCount() > 5){
      sync=true;
    }
  }
  ptp.update();
  digitalWrite(13, ptp.getLockCount() > 5 && noPPSCount < 5? HIGH : LOW);
}

void syncInterrupt() {
  
  if(noPPSCount > 5){
    sync=true;
  }else{
    noPPSCount++;
  }
}

void announceInterrupt() {
  announce=true;
}

static void interrupt_1588_timer() {
  uint32_t t;
  if (!qindesign::network::EthernetIEEE1588.getAndClearChannelStatus(2)) {
    asm("dsb"); // allow write to complete so the interrupt doesn't fire twice
    return;
  }
  qindesign::network::EthernetIEEE1588.getChannelCompareValue(2,t);

  t = ((NanoTime)t+NS_PER_S-60)%NS_PER_S;
  
  timespec ts;
  qindesign::network::EthernetIEEE1588.readTimer(ts);

  if(ts.tv_nsec < 100*1000*1000 && t > 900*1000*1000){
    pps_s=ts.tv_sec;
    interrupt_s=ts.tv_sec-1;    
  }else{
    interrupt_s=ts.tv_sec;      
    if(ts.tv_nsec < 500*1000*1000){
      pps_s=ts.tv_sec;
    }else{
      pps_s=ts.tv_sec+1;
    }
  }

  interrupt_ns=t;
  pps_ns=0;

  pps=true;
  noPPSCount=0;
  
  asm("dsb"); // allow write to complete so the interrupt doesn't fire twice
}

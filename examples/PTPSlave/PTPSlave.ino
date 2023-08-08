#include <t41-ptp.h>
#include <QNEthernet.h>

byte mac[6];
IPAddress staticIP{192, 168, 0, 210};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{192, 168, 0, 6};

bool p2p=false;
bool master=false;
bool slave=true;

l3PTP ptp(master,slave,p2p);
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
}

void loop()
{
  ptp.update();
  digitalWrite(13, ptp.getLockCount() > 5 ? HIGH : LOW);
}

#include <cassert>
#include <cstring>
#include <thread>
#include <atomic>
#include <cstdlib>
#include "ldlidar_driver.h"
#include "lipkg.h"
#include "serial_interface_linux.h"

namespace ldlidar { uint8_t CalCRC8(const uint8_t*, uint16_t); }

static ldlidar::LiDARFrameTypeDef packet(unsigned start, unsigned end) {
  ldlidar::LiDARFrameTypeDef p{};
  p.header = ldlidar::PKG_HEADER;
  p.ver_len = ldlidar::PKG_VER_LEN;
  p.speed = 3600;
  p.start_angle = start;
  p.end_angle = end;
  for (auto& point : p.point) { point.distance = 1000; point.intensity = 100; }
  p.crc8 = ldlidar::CalCRC8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
  return p;
}
static void feed(ldlidar::LiPkg& parser, const ldlidar::LiDARFrameTypeDef& p) {
  parser.CommReadCallBack(reinterpret_cast<const char*>(&p), sizeof(p));
}
static void revolution(ldlidar::LiPkg& parser) {
  for (unsigned i = 0; i < 30; ++i) {
    unsigned start = (i * 1650) % 36000;
    feed(parser, packet(start, (start + 1650) % 36000));
  }
}
int main() {
  ldlidar::LiPkg a, b;
  a.SetProductType(ldlidar::LDType::LD_14);
  b.SetProductType(ldlidar::LDType::LD_14);
  uint64_t now = 1700000000000000000ULL;
  a.RegisterTimestampGetFunctional([&] { return now += 1000000; });
  b.RegisterTimestampGetFunctional([&] { return now += 1000000; });
  auto p = packet(1001, 1102);
  const char* bytes = reinterpret_cast<const char*>(&p);
  a.CommReadCallBack(bytes, 10);
  feed(b, p); feed(b, p);
  assert(b.GetLidarPowerOnCommStatus());
  a.CommReadCallBack(bytes + 10, sizeof(p) - 10);
  feed(a, p);
  assert(a.GetLidarPowerOnCommStatus());
  // A repeated header must not swallow the beginning of the next packet.
  a.ClearDataProcessStatus();
  a.CommReadCallBack(bytes, 1);
  feed(a, p); feed(a, p);
  assert(a.GetLidarPowerOnCommStatus());
  // Restart abandons a partially received packet and any completed scan.
  a.CommReadCallBack(bytes, 10);
  a.ClearDataProcessStatus();
  feed(a, p); feed(a, p);
  assert(a.GetLidarPowerOnCommStatus());
  revolution(a);
  ldlidar::Points2D scan;
  assert(a.GetLaserScanData(scan));
  assert(!scan.empty());
  for (const auto& point : scan) assert(point.stamp <= now);
  assert(!a.GetLaserScanData(scan));
  revolution(a);
  now = 1000000; // Simulate a backward wall-clock adjustment.
  feed(a, p);
  assert(!a.GetLaserScanData(scan));
  revolution(a);
  assert(a.GetLaserScanData(scan));
  for (const auto& point : scan) assert(point.stamp <= now);
  a.ClearDataProcessStatus();
  assert(a.GetSpeed() == 0);
  assert(!a.GetLaserScanData(scan));
  assert(!a.GetLidarPowerOnCommStatus());
  // Corrupt packets and out-of-range angles cannot establish communication.
  p.crc8 ^= 1;
  feed(a, p); feed(a, p);
  assert(!a.GetLidarPowerOnCommStatus());
  feed(a, packet(36000, 100)); feed(a, packet(36000, 100));
  assert(!a.GetLidarPowerOnCommStatus());
  // Exercise producer/consumer synchronization while packets arrive.
  std::atomic<bool> done{false};
  std::thread consumer([&] {
    ldlidar::Points2D out;
    while (!done.load()) { a.GetLaserScanData(out); a.GetSpeed(); a.GetLidarPowerOnCommStatus(); }
  });
  for (int i = 0; i < 50; ++i) revolution(a);
  done = true;
  consumer.join();
  ldlidar::SerialInterfaceLinux serial;
  assert(!serial.IsOpened());
  assert(serial.Close());
  // A pseudo-terminal exercises actual receiver start/stop and driver teardown.
  int master = posix_openpt(O_RDWR | O_NOCTTY);
  assert(master >= 0);
  assert(grantpt(master) == 0);
  assert(unlockpt(master) == 0);
  std::string port = ptsname(master);
  assert(serial.Open(port, 115200));
  assert(serial.Close());
  assert(serial.Open(port, 115200));
  assert(serial.Close());
  {
    ldlidar::LDLidarDriver driver;
    driver.RegisterGetTimestampFunctional([] { return uint64_t{1000000}; });
    assert(driver.Start(ldlidar::LDType::LD_14, port, 115200));
  }
  close(master);
}

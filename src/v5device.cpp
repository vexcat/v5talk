#include "v5device.hpp"
#include <stdio.h>
#include "cppystruct.h"
#include <unordered_map>
#include <errno.h>
#include <err.h>
#include <sys/types.h>
#include <poll.h>
#include <thread>
#include <sys/time.h>

inline uint32_t mask32(int size) {
  return ((uint32_t)-1 >> (32 - size));
}

//From prosv5
CRC::CRC(uint32_t isize, uint32_t poly): size(isize) {
  for(uint32_t i = 0; i < 256; i++) {
    uint32_t acc = i << (size - 8);
    for(int j = 0; j < 8; j++) {
      if(acc & (1 << (size - 1))) {
        acc <<= 1;
        acc ^= poly;
      } else acc <<= 1;
    }
    table[i] = acc & mask32(size);
  }
}

uint32_t CRC::operator()(const bytes& data, uint32_t acc) {
  for(auto& d: data) {
    uint8_t i = (acc >> (size-8)) ^ d;
    acc = ((acc << 8) ^ table[i]) & mask32(size);
  }
  return acc;
}

CRC VEX_CRC16 = CRC(16, 0x1021);     // CRC-16-CCIT
CRC VEX_CRC32 = CRC(32, 0x04C11DB7); // CRC-32 (the one used everywhere but has no name)

bytes form_simple_packet(uint8_t msg) {
  return { 0xc9, 0x36, 0xb8, 0x47, msg };
}

void V5Device::tx_packet(uint8_t command, const bytes& data) {
  auto tx = form_simple_packet(command);
  tx.insert(tx.end(), data.begin(), data.end());
  write(port, tx.data(), tx.size());
}

uint64_t unixTime() {
  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  return (uint64_t)(currentTime.tv_sec * 1000LL + currentTime.tv_usec / 1000LL);
}

int V5Device::getc(int ms) {
  unsigned char ret;
  int bytes_read = 0;
  struct pollfd p;
  p.fd = port;
  p.events = POLLIN | POLLPRI;
  auto startTime = unixTime();
  while((bytes_read = read(port, &ret, 1)) < 1) {
    if(bytes_read == -1) {
      if(errno != EAGAIN) {
        fprintf(stderr, "read fail\n");
        exit(1);
      } else {
        auto retval = poll(&p, 1, ms);
        if(retval == -1) {
          fprintf(stderr, "poll fail\n");
          exit(1);
        }
        if(ms != -1 && unixTime() - startTime >= ms - 1) {
          throw std::runtime_error("timeout");
        }
      }
    }
  }
  return ret;
}

bytes V5Device::readn(int n, int ms) {
  bytes ret;
  ret.resize(n);
  int i = 0;
  struct pollfd p;
  p.fd = port;
  p.events = POLLIN | POLLPRI;
  auto startTime = unixTime();
  while(i != n) {
    auto delta = read(port, ret.data() + i, n - i);
    if(delta == -1) {
      if(errno != EAGAIN) {
        fprintf(stderr, "read fail\n");
        exit(1);
      } else {//
        auto retval = poll(&p, 1, ms);
        if(retval == -1) {
          fprintf(stderr, "poll fail\n");
          exit(1);
        }
        if(ms != -1 && unixTime() - startTime >= ms - 1) {
          throw std::runtime_error("timeout");
        }
        continue;
      }
    }
    i += delta;
  }
  return ret;
}

V5CommandResponse V5Device::rx_packet() {
  V5CommandResponse ret;
  //Sync Bytes (0xAA, 0x55)
  bytes syncStack;
  while(syncStack.size() < 2) {
    syncStack.push_back(getc());
    if(syncStack.size() == 2 && syncStack[1] != 0x55) {
      syncStack.clear();
      continue;
    }
    if(syncStack.size() == 1 && syncStack[0] != 0xAA) {
      syncStack.clear();
      continue;
    }
  }
  //Command
  ret.command = getc();
  //Length
  int len;
  len = getc();
  //Extended payloads can be used with V5 CDC commands.
  if(ret.command == 0x56 && len & 0x80) {
    len ^= 0x80; len <<= 8;
    len += getc();
  }
  //Payload
  ret.payload = readn(len);
  return ret;
}

bytes V5Device::txrx_packet(uint8_t command, int len, const bytes& data) {
  tx_packet(command, data);
  auto ret = rx_packet();
  if(command != ret.command) throw std::runtime_error("Sent command " + std::to_string(command) + " but got response command of " + std::to_string(ret.command));
  if(len != -1 && ret.payload.size() != len) throw std::runtime_error("Payload length wrong for response to " + std::to_string(command) + " command.");
  return ret.payload;
}

bytes V5Device::formExtendedPayload(int msg, const bytes& data) {
  bytes ret = { 0xc9, 0x36, 0xb8, 0x47, 0x56, (unsigned char)msg };
  if(data.size() >= 0x80) {
    ret.push_back((data.size() >> 8) | 0x80);
    ret.push_back(data.size() & 0xFF);
  } else {
    ret.push_back(data.size());
  }
  ret.insert(ret.end(), data.begin(), data.end());
  auto crc = VEX_CRC16(ret);
  ret.push_back(crc >> 8);
  ret.push_back(crc & 0xFF);
  ret.erase(ret.begin(), ret.begin() + 5);
  return ret;
}

std::unordered_map<int, std::string> nackMessages = {
  {0xFF, "General NACK"},
  {0xCE, "CRC error on recv'd packet"},
  {0xD0, "Payload too small"},
  {0xD1, "Request transfer size too large"},
  {0xD2, "Program CRC error"},
  {0xD3, "Program file error"},
  {0xD4, "Attempted to download/upload uninitialized"},
  {0xD5, "Initialization invalid for this function"},
  {0xD6, "Data not a multiple of 4 bytes"},
  {0xD7, "Packet address does not match expected"},
  {0xD8, "Data downloaded does not match initial length"},
  {0xD9, "Directory entry does not exist"},
  {0xDA, "Max user files, no more room for another user program"},
  {0xDB, "User file exists"}
};

//ack can have garbage at the end of a read transfer, who knows why
bytes rxExtPacket(const bytes& data, int len, uint8_t command, bool checkAck) {
  //We're not checking the CRC, because the USB connection already has error correction.
  if(data[0] != command) throw std::runtime_error("Wrong response command " + std::to_string(data[0]) + " for " + std::to_string(command));
  if(checkAck) {
    if(nackMessages.find(data[1]) != nackMessages.end()) {
      throw std::runtime_error("Device NACK'd because " + nackMessages[data[1]]);
    } else if(data[1] != 0x76) {
      throw std::runtime_error("Device sent unknown NACK");
    }
  }
  if(len != -1 && len != data.size() - (checkAck ? 4 : 3)) throw std::runtime_error("Bad response length for ext command " + std::to_string(command));
  return bytes(data.begin() + (checkAck ? 2 : 1), data.end() - 2);
}

bytes V5Device::txrx_ext_packet(uint8_t command, int len, const bytes& data, bool checkAck) {
  auto tx = formExtendedPayload(command, data);
  auto rx = txrx_packet(0x56, -1, tx);
  return rxExtPacket(rx, len, command, checkAck);
}

V5SystemVersion V5Device::querySystemVersion() {
  V5SystemVersion ret;
  auto data = txrx_packet(0xA4, 8);
  ret.version = std::to_string(data[0]) + "."
              + std::to_string(data[1]) + "."
              + std::to_string(data[2]) + "-"
              + std::to_string(data[3]) + "."
              + std::to_string(data[4]);
  ret.product = data[5];
  ret.productFlags = data[6];
  fprintf(stderr, "V5 %s%s running version %s\n",
  ret.product == (int)Product::CONTROLLER ? "Controller" : "Brain",
  ret.product == (int)Product::CONTROLLER ? (ret.productFlags & 2 ? " (connected)" : " (disconnected)") : "",
  ret.version.c_str());
  return ret;
}

void V5Device::enterFTChannel(FTChannel which, double timeout) {
  auto vers = querySystemVersion();
  bool gotSysVer;
  txrx_ext_packet(0x10, 0, {1, (unsigned char)which});
  //Wait for controller to be connected
  if(vers.product == (int)Product::CONTROLLER) {
    printf("Waiting on channel transfer...\n");
    //For the first 250ms, the controller won't have *actually* initiated the transfer,
    //reporting that it is still connected.
    usleep(250000);
    while(true) {
      gotSysVer = false;
      auto versionQueryThread = std::thread([&]{
        vers = querySystemVersion();
        gotSysVer = true;
      });
      //For a short period during the channel transfer, bytes we send
      //will not be received.
      while(!gotSysVer) {
        //Send command to get system version and discard any response.
        std::thread([&]{ try { 
          tx_packet(0xA4);
        } catch(...) {}}).detach();
        usleep(250000);
      }
      versionQueryThread.join();
      if(vers.productFlags & (int)ControllerFlags::CONNECTED) {
        break;
      }
    }
    printf("Channel transfer OK\n");
  }
}

template <typename T>
bytes toBytes(T arr) {
  bytes ret(arr.begin(), arr.end());
  return ret;
}

struct wackview {
  const bytes* captive;
  size_t size() { return captive->size(); }
  const char* data() { return (const char*)captive->data(); }
  wackview(const bytes& ref): captive(&ref) {}
};

V5FileTransfer V5Device::ftInitialize(FTDirection dir, const str &name, FTLocation where, uint32_t crc, VID vid, uint32_t address, uint32_t length, int options) {
  //Seconds since 2000 Jan 1
  auto ts = time(0) - 946684800;
  auto tx = toBytes(pystruct::pack(PY_STRING("<4B3I4s2I24s"), (int)dir, (int)where, (int)vid, options, length, address, crc, name.substr(name.find_last_of(".") + 1).c_str(), ts, 0x01000000, name.c_str()));
  auto rx = txrx_ext_packet(0x11, pystruct::calcsize(PY_STRING("<H2I")), tx);
  auto [maxPacketSize, fileSize, rxcrc] = pystruct::unpack(PY_STRING("<H2I"), wackview{rx});
  V5FileTransfer response;
  response.maxPacketSize = maxPacketSize;
  response.fileSize = fileSize;
  response.crc = rxcrc;
  return response;
}

void V5Device::ftComplete(FTCompleteAction action) {
  txrx_ext_packet(0x12, 0, {(unsigned char) action});
}

#define getb(n, b) (unsigned char)(((unsigned long long int)n >> (b*8)) & 0xFF)
#define put32(n) getb(n, 0), getb(n, 1), getb(n, 2), getb(n, 3)
#define put16(n) getb(n, 0), getb(n, 1)
#define put8(n) getb(n, 0)

void V5Device::ftWrite(uint32_t addr, const bytes& data) {
  bytes tx = { put32(addr) };
  tx.insert(tx.end(), data.begin(), data.end());
  while(tx.size() % 4) tx.push_back(0);
  txrx_ext_packet(0x13, 0, tx);
}

bytes V5Device::ftRead(uint32_t addr, uint32_t len) {
  bytes tx = { put32(addr), put16(((len+3)/4)*4) };
  auto rx =  txrx_ext_packet(0x14, -1, tx, false);
  rx.erase(rx.begin(), rx.begin() + 4);
  rx.erase(rx.begin() + len, rx.end());
  return rx;
}

void V5Device::ftSetLink(const str& to, VID vid, int options) {
  auto tx = toBytes(pystruct::pack(PY_STRING("<2B24s"), (int)vid, options, to));
  txrx_ext_packet(0x15, 0, tx);
}

uint16_t V5Device::getDirCount(VID vid, int options) {
  bytes tx = { put8(vid), put8(options) };
  auto rx = txrx_ext_packet(0x16, 2, tx);
  return rx[0] + (rx[1] << 8);
}

inline std::string truncateString(std::string str) {
  return std::string(str.c_str());
}

V5FileMeta V5Device::getFileMetadataByIdx(int idx, int options) {
  bytes tx = { put8(idx), put8(options) };
  auto rx = txrx_ext_packet(0x17, pystruct::calcsize(PY_STRING("<B3L4sLL24s")), tx);
  auto [rxidx, size, addr, crc, type, timestamp, version, filename] = pystruct::unpack(PY_STRING("<B3L4sLL24s"), wackview{rx});
  V5FileMeta ret;
  ret.idx = rxidx;
  ret.size = size;
  ret.addr = addr;
  ret.crc = crc;
  ret.type = truncateString(std::string(type));
  ret.timestamp = timestamp;
  ret.version = version;
  ret.name = truncateString(std::string(filename));
  return ret;
}

void V5Device::execute(const str& name, VID vid, bool run) {
  auto tx = toBytes(pystruct::pack(PY_STRING("<2B24s"), (int) vid, run ? 0 : 0x80, name));
  txrx_ext_packet(0x18, 0, tx);
}

V5FileMeta V5Device::getFileMetadataByName(const str& name, VID vid, int options) {
  bytes tx = toBytes(pystruct::pack(PY_STRING("<2B24s"), (int)vid, (int)options, name));
  auto rx = txrx_ext_packet(0x17, pystruct::calcsize(PY_STRING("<B3L4sLL24s")), tx);
  auto [linkedVid, size, addr, crc, type, timestamp, version, filename] =
    pystruct::unpack(PY_STRING("<B3L4sLL24s"), wackview{rx});
  V5FileMeta ret;
  ret.linkedVid = linkedVid;
  ret.size = size;
  ret.addr = addr;
  ret.crc = crc;
  ret.type = truncateString(std::string(type));
  ret.timestamp = timestamp;
  ret.version = version;
  ret.name = truncateString(std::string(filename));
  return ret;
}

V5FileMeta V5Device::getFileMetadataByNameNotBroken(const str& name, VID vid, int options) {
  int entries = getDirCount(vid);
  for(int i = 0; i < entries; i++) {
    V5FileMeta meta = getFileMetadataByIdx(i);
    if(meta.name == name) return meta;
  }
  throw std::runtime_error("No such file.");
}

void V5Device::setProgramFileMetadata(V5FileMeta meta, int options) {
  auto tx = toBytes(pystruct::pack(PY_STRING("<2BI4s2I24s"), (int)meta.linkedVid, options, meta.addr, meta.type.c_str(), meta.timestamp, meta.version, meta.name.c_str()));
  txrx_ext_packet(0x1A, 0, tx);
}

void V5Device::eraseFile(const str& name, VID vid, bool eraseAll) {
  auto tx = toBytes(pystruct::pack(PY_STRING("<2B24s"), (int)vid, eraseAll ? 0x80 : 0, name.c_str()));
  txrx_ext_packet(0x1B, 0, tx);
}

uint8_t V5Device::getProgramFileSlot(const str& name, VID vid, int options) {
  auto tx = toBytes(pystruct::pack(PY_STRING("<2B24s"), (int)vid, options, name.c_str()));
  auto rx = txrx_ext_packet(0x1C, 0, tx);
  return rx[0];
}

V5SystemStatus V5Device::getSystemStatus() {
  auto data = txrx_ext_packet(0x22, pystruct::calcsize(PY_STRING("<x12B3xBI12x")));
  V5SystemStatus ret;
  ret.systemVersion    = std::to_string(data[1]) + "."
                       + std::to_string(data[2]) + "."
                       + std::to_string(data[3]) + "-"
                       + std::to_string(data[4]);
  ret.masterCPUVersion = std::to_string(data[5]) + "."
                       + std::to_string(data[6]) + "."
                       + std::to_string(data[7]) + "-"
                       + std::to_string(data[8]);
  ret.userCPUVersion   = std::to_string(data[9]) + "."
                       + std::to_string(data[10]) + "."
                       + std::to_string(data[11]) + "-"
                       + std::to_string(data[12]);
  ret.touchVersion = data[16];
  ret.systemID = data[17] + (data[18] << 8) + (data[19] << 16) + (data[20] << 24);
  return ret;
}

bytes V5Device::userFIFORead() {
  return txrx_ext_packet(0x27, -1, {1, 0x40});
}

void V5Device::screenCaptureInit() {
  txrx_ext_packet(0x28, 0);
}

bytes V5Device::readFile(
  str name,
  FTCompleteAction onComplete,
  FTLocation where,
  VID vid,
  bool quiet,
  uint32_t address,
  uint32_t length
) {
  bytes ret;
  //Special file read logic:
  //  STORAGE -> address & length need to be queried based on the filename
  //  FRAMEBUFFER -> screenCaptureInit() must be called, use hardcoded length & addr
  //  RAM -> use empty name ''.
  if(where == FTLocation::STORAGE) {
    auto fMeta = getFileMetadataByNameNotBroken(name, vid);
    address = fMeta.addr;
    length = fMeta.size;
  }
  if(where == FTLocation::FRAMEBUFFER) {
    screenCaptureInit();
    address = 0;
    length = 557056;
  }
  if(where == FTLocation::RAM) {
    name = "";
  }
  //Initialize the transfer
  auto tMeta = ftInitialize(FTDirection::DOWNLOAD, name, where, 0, vid, address, length);
  //Start reading
  int lastHyaku = 0;
  for(int i = 0; i < length;) {
    int packetSize = tMeta.maxPacketSize;
    if(i + packetSize > length) {
      packetSize = length - i;
    }
    bytes packetData = ftRead(address + i, packetSize);
    ret.insert(ret.end(), packetData.begin(), packetData.end());
    if(packetSize - packetData.size()) fprintf(stderr, "wanted %u but got %lu\n", packetSize, packetData.size());
    i += packetData.size();
    if(!quiet && (i / (tMeta.maxPacketSize * 100)) != lastHyaku) {
      fprintf(stderr, "\x1b[G\x1b[K%d/%d", lastHyaku = (i / (tMeta.maxPacketSize * 100)), length / (tMeta.maxPacketSize * 100));
      fflush(stdout);
    }
  }
  fprintf(stderr, "\x1b[1000D100%%\n");
  try {
    ftComplete(onComplete);
  } catch(...) {
    fprintf(stderr, "Device unexpectedly closed file transfer, but all data was successfully received.\n");
  }
  return ret;
}

void V5Device::writeFile(
  bytes data,
  str name,
  bool hasLink,
  const str& linkedName,
  VID linkedVID,
  FTCompleteAction onComplete,
  FTLocation where,
  VID vid,
  bool quiet,
  uint32_t address,
  int options
) {
  //All options are necessary for FTLocation::STORAGE.
  if(where == FTLocation::FRAMEBUFFER) {
    //Writing to FRAMEBUFFER doesn't make too much sense, especially
    //since it's just an area in memory that scInit copies to and
    //not directly connected to the screen.
    screenCaptureInit();
    address = 0;
  }
  if(where == FTLocation::RAM) {
    name = "";
    hasLink = false;
  }
  //Initialize the upload
  auto crc = VEX_CRC32(data);
  auto tMeta = ftInitialize(FTDirection::UPLOAD, name, where, crc, vid, address, data.size(), options);
  if(hasLink) {
    ftSetLink(linkedName, linkedVID, options);
  }
  if(tMeta.fileSize < data.size()) throw std::runtime_error("No more space on device.");
  //Begin writing
  for(int i = 0; i < data.size(); i += tMeta.maxPacketSize) {
    int packetSize = tMeta.maxPacketSize;
    if(i + packetSize > data.size()) {
      packetSize = data.size() - i;
    }
    ftWrite(address + i, bytes(data.begin() + i, data.begin() + i + packetSize));
    if(!quiet) putc('*', stderr);
  }
  putc('\n', stderr);
  fprintf(stderr, "Waiting for device to close transfer\n");
  //Complete the transfer
  ftComplete(onComplete);
  fprintf(stderr, "Done\n");
}
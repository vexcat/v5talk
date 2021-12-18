#include <stdio.h>
#include "v5device.hpp"
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <thread>
#include <functional>
#include <mutex>
#include <string.h>

struct Args {
  std::vector<std::string> steps;
  std::unordered_map<std::string, std::string> options;
  bool checkBoolean(const std::string& key) {
    return options[key] != "false" && options[key] != "";
  }
  std::string get(const std::string& key, const std::string& defaultValue = "") {
    if(options[key] != "") return options[key];
    return defaultValue;
  }
  void check(int stepCount, std::vector<std::string> acceptableOptions) {
    if(steps.size() < stepCount) throw std::runtime_error("Too few arguments provided.");
    if(steps.size() > stepCount) throw std::runtime_error("Too many arguments provided.");
    for(auto& pair: options) {
      if(std::find(acceptableOptions.begin(), acceptableOptions.end(), pair.first) == acceptableOptions.end()) {
        throw std::runtime_error("Unknown option " + pair.first);
      }
    }
  }
};

Args additionalParsing(char** argv) {
  Args ret;
  for(char* cstr; (cstr = *argv); argv++) {
    std::string str(cstr);
    if(str.rfind("--", 0) == 0) {
      int assignment = str.find_first_of('=');
      if(assignment != -1) { // k/v
        ret.options[str.substr(2, assignment-2)] = str.substr(assignment + 1);
      } else { // boolean option --enable --no-enable
        if(str.rfind("--no-", 0) == 0) {
          ret.options[str.substr(4)] = "false";
        } else {
          ret.options[str.substr(2)] = "true";
        }
      }
    } else {
      ret.steps.push_back(str);
    }
  }
  return ret;
}
uint32_t toAddress(const std::string& address) {
  return strtol(address.c_str(), NULL, 0);
}

bytes readAll(const std::string& filename) {
  bytes ret;
  FILE* f = fopen(filename.c_str(), "r");
  fseek(f, 0, SEEK_END);
  ret.resize(ftell(f));
  fseek(f, 0, SEEK_SET);
  fread(ret.data(), ret.size(), 1, f);
  fclose(f);
  return ret;
}

void writeAll(const std::string& filename, const bytes& data) {
  FILE* f = fopen(filename.c_str(), "w");
  fwrite(data.data(), data.size(), 1, f);
  fclose(f);
}

std::string toLower(std::string data) {
  std::transform(data.begin(), data.end(), data.begin(),
    [](unsigned char c){ return std::tolower(c); });
  return data;
}

//USER = 1, SYSTEM = 15, RMS = 16, PROS = 24, MW = 32, DEV4 = 40, DEV5 = 48, DEV6 = 56, VEX = 240, UNDEF
V5Device::VID toVID(const std::string& text) {
  auto lower = toLower(text);
  if(lower == "user") return V5Device::VID::USER;
  if(lower == "pros") return V5Device::VID::PROS;
  if(lower == "rms") return V5Device::VID::RMS;
  if(lower == "mw") return V5Device::VID::MW;
  if(lower == "sys") return V5Device::VID::SYSTEM;
  if(lower == "dev4") return V5Device::VID::DEV4;
  if(lower == "dev5") return V5Device::VID::DEV5;
  if(lower == "dev6") return V5Device::VID::DEV6;
  if(lower == "vex") return V5Device::VID::VEX;
  if(lower == "undef") return V5Device::VID::UNDEF;
  return V5Device::VID::USER;
}

V5Device::FTCompleteAction toComplete(const std::string& text) {
  auto lower = toLower(text);
  if(lower == "none") return V5Device::FTCompleteAction::NO_ACTION;
  if(lower == "view") return V5Device::FTCompleteAction::RUN_SCREEN;
  if(lower == "run") return V5Device::FTCompleteAction::EXECUTE;
  return V5Device::FTCompleteAction::NO_ACTION;
}

V5Device::FTLocation toLocation(const std::string& text) {
  auto lower = toLower(text);
  if(lower == "storage") return V5Device::FTLocation::STORAGE;
  if(lower == "framebuffer") return V5Device::FTLocation::FRAMEBUFFER;
  if(lower == "ram") return V5Device::FTLocation::RAM;
  if(lower == "vbuf") return V5Device::FTLocation::VBUF;
  if(lower == "a1") return V5Device::FTLocation::A1;
  if(lower == "b1") return V5Device::FTLocation::B1;
  if(lower == "b2") return V5Device::FTLocation::B2;
  //really just for conversion to number
  return (V5Device::FTLocation)toAddress(text);
}


void cli_upload(V5Device& dev, Args args) {
  args.check(2, { "vid", "link", "linkVID", "onComplete", "to", "address", "quiet", "checkAlreadySent" });
  bytes fileData = readAll(args.steps[0]);
  if(args.checkBoolean("checkAlreadySent")) {
    try {
      auto meta = dev.getFileMetadataByNameNotBroken(args.steps[1], toVID(args.get("vid", "user")));
      auto crc = VEX_CRC32(fileData);
      if(meta.crc == crc) return;
    } catch(...) {
      //If something was caught, that just means we have to upload normally.
    }
  }
  dev.writeFile(
    fileData,
    args.steps[1],
    args.get("link") != "",
    args.get("link"),
    toVID(args.get("linkVID", "pros")),
    toComplete(args.get("onComplete")),
    toLocation(args.get("to", "storage")),
    toVID(args.get("vid", "user")),
    args.checkBoolean("quiet"),
    toAddress(args.get("address", "0"))
  );
}

void cli_download(V5Device& dev, Args args) {
  args.check(2, { "vid", "onComplete", "from", "address", "quiet", "length" });
  writeAll(args.steps[1], dev.readFile(
    args.steps[0],
    toComplete(args.get("onComplete")),
    toLocation(args.get("from", "storage")),
    toVID(args.get("vid", "user")),
    args.checkBoolean("quiet"),
    toAddress(args.get("address", "0")),
    toAddress(args.get("length", "0"))
  ));
}

void cli_rm(V5Device& dev, Args args) {
  args.check(1, { "vid" });
  dev.eraseFile(args.steps[0], toVID(args.get("vid", "user")));
}

void cli_mv(V5Device& dev, Args args) {
  args.check(2, { "vid" });
  dev.getDirCount(toVID(args.get("vid", "user")));
  auto meta = dev.getFileMetadataByNameNotBroken(args.steps[0], toVID(args.get("vid", "user")));
  meta.name = args.steps[1];
  meta.type = args.steps[1].substr(args.steps[1].find_last_of(".") + 1).c_str();
  dev.setProgramFileMetadata(meta);
}

void cli_ls(V5Device& dev, Args args) {
  std::vector<std::string> vids = { "user", "sys", "pros", "rms", "mw", "dev4", "dev5", "dev6", "vex", "undef" };
  for(auto& vid: vids) {
    fprintf(stderr, "Listing for VID folder %s:\n", vid.c_str());
    int entries = dev.getDirCount(toVID(vid));
    for(int i = 0; i < entries; i++) {
      V5FileMeta meta = dev.getFileMetadataByIdx(i);
      fprintf(stderr, "%d: %dbyte @0x%08X (%s) %s\n", meta.idx, meta.size, meta.addr, meta.type.c_str(), meta.name.c_str());
    }
    fprintf(stderr, "\n");
  }
}

void cli_stop(V5Device& dev, Args args) {
  dev.execute("", V5Device::VID::USER, false);
}

void cli_runfile(V5Device& dev, Args args) {
  args.check(1, { "vid" });
  dev.execute(args.steps[0], toVID(args.get("vid", "user")));
}

V5Device::FTChannel toChannel(const std::string& text) {
  auto lower = toLower(text);
  if(lower == "download") return V5Device::FTChannel::DOWNLOAD;
  if(lower == "pit") return V5Device::FTChannel::PIT;
  return V5Device::FTChannel::DOWNLOAD;
}

void cli_transfer(V5Device& dev, Args args) {
  args.check(0, { "to" });
  dev.enterFTChannel(toChannel(args.get("to", "download")));
}
std::string hex(uint8_t byte) {
  char chars[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  std::string ret;
  ret.push_back(chars[byte & 0xF]);
  ret.push_back(chars[(byte & 0xF0) >> 8]);
  return ret;
}

std::string hexes(const bytes& ree) {
  std::string ret;
  for(auto& byte: ree) {
    ret += hex(byte) + " ";
  }
  ret.erase(ret.end() - 1);
  return ret;
}

void cli_fuzz(V5Device& dev, Args args) {
  args.check(1, {});
  auto extCommand = (uint8_t)toAddress(args.steps[0]);
  bytes data;
  for(int i = 0; i <= 40; i++) {
    try {
      auto rx = dev.txrx_ext_packet(extCommand, -1, data);
      fprintf(stderr, "%d length OK, response { %s }\n", i, hexes(rx).c_str());
    } catch(const std::exception& e) {
      fprintf(stderr, "%d length execution reported %s\n", i, e.what());
    }
    data.push_back(0);
  }
}

struct CRCCandidate {
  int where;
  int size;
  bool lilEndian;
};

void cli_check(Args args) {
  args.check(1, { "scan" });
  auto hexData = args.steps[0];
  if(hexData.size() & 1) throw std::runtime_error("odd # of hex characters");
  bytes data;
  data.resize(hexData.size() / 2);
  for(int i = 0; i < hexData.size(); i += 2) {
    std::string b;
    b += hexData[i];
    b += hexData[i + 1];
    data[i/2] = strtol(b.c_str(), nullptr, 16);
  }
  printf("%04X\n", VEX_CRC16(data));
  printf("%08X\n", VEX_CRC32(data));
  //Scan for CRCs in the message with brute force
  if(args.checkBoolean("scan")) {
    std::unordered_map<uint32_t, CRCCandidate> cand;
    //Get all possible CRCs from the message
    for(int i = 0; i < data.size() - 1; i++) {
      cand[data[i+0] + (data[i+1] << 8)] = {i, 2, true};
      cand[data[i+1] + (data[i+0] << 8)] = {i, 2, false};
    }
    for(int i = 0; i < data.size() - 3; i++) {
      cand[data[i+0] + (data[i+1] << 8) + (data[i+2] << 16) + (data[i+3] << 24)] = {i, 4, true};
      cand[data[i+3] + (data[i+2] << 8) + (data[i+1] << 16) + (data[i+0] << 24)] = {i, 4, false};
    }
    if(cand.find(0) != cand.end()) cand.erase(cand.find(0));
    //Search for payloads that match any CRC.
    for(int i = 0; i < data.size(); i++) {
      for(int j = i + 1; j <= data.size(); j++) {
        bytes sub;
        printf("Scanning area [%d,%d)\n", i, j);
        for(int z = i; z < j; z++) {
          sub.push_back(data[z]);
        }
        auto c16 = VEX_CRC16(sub);
        if(cand.find(c16) != cand.end()) {
          printf("Section [%d,%d) matched %02X\n", i, j, c16);
        }
        auto c32 = VEX_CRC32(sub);
        if(cand.find(c32) != cand.end()) {
          printf("Section [%d,%d) matched %04X\n", i, j, c32);
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  if(argc < 3) {
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "v5talk has 7 commands: upload, download, rm, mv, ls, stop, run-file.\n");
    fprintf(stderr, "Here are some examples of them:\n");
    fprintf(stderr, "  v5talk /dev/blah upload my_program.uwu slot_1.bin # Uploads my_program.uwu to the V5 as slot_1.bin\n");
    fprintf(stderr, "  v5talk /dev/blah download slot_1.bin my_program.uwu # Downloads slot_1.bin from the V5 to my_program.uwu\n");
    fprintf(stderr, "  v5talk /dev/blah rm slot_1.bin # Removes slot_1.bin from the V5.\n");
    fprintf(stderr, "  v5talk /dev/blah mv slot_1.bin slot_2.bin # Renames slot_1.bin to slot_2.bin\n");
    fprintf(stderr, "  v5talk /dev/blah ls # Lists files and associated metadata\n");
    fprintf(stderr, "  v5talk /dev/blah stop # Stops the currently executing program\n");
    fprintf(stderr, "  v5talk /dev/blah run-file slot_1.bin # Executes slot_1.bin\n");
    fprintf(stderr, "The funnest command of all:\n");
    fprintf(stderr, "  v5talk /dev/blah fuzz-length 0x2a\n");
    fprintf(stderr, "Of course, you'd also need to specify an address (--address=) for uploading a program and also to upload .ini's.\n");
    fprintf(stderr, "For options each command can take, look at vexcat/v5talk/src/main.cpp.\n");
    return 1;
  }
  try {
    std::string port = argv[1];
    std::string verb = argv[2];
    auto args = additionalParsing(argv + 3);
    if(port != "none") {
      V5Device dev(port.c_str());
      if(verb == "upload") cli_upload(dev, args);
      if(verb == "download") cli_download(dev, args);
      if(verb == "rm") cli_rm(dev, args);
      if(verb == "mv") cli_mv(dev, args);
      if(verb == "ls") cli_ls(dev, args);
      if(verb == "stop") cli_stop(dev, args);
      if(verb == "run-file") cli_runfile(dev, args);
      if(verb == "fuzz-length") cli_fuzz(dev, args);
      if(verb == "transfer") cli_transfer(dev, args);
    } else {
      if(verb == "check") cli_check(args);
    }
  } catch(const std::exception& e) {
    fprintf(stderr, "You broke it >,<\n");
    fprintf(stderr, "Exception text: %s\n", e.what());
  } catch(...) {
    fprintf(stderr, "You broke it >,<\n");
    fprintf(stderr, "Unknown exception type\n");
  }
  return 0;
}
#include <stdint.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using bytes = std::vector<unsigned char>;

//Vex CRC
class CRC {
  uint32_t size;
  uint32_t table[256];
  public:
  CRC(uint32_t size, uint32_t poly);
  uint32_t operator()(const bytes& data, uint32_t accumulator = 0);
};
extern CRC VEX_CRC16;
extern CRC VEX_CRC32;

//File Upload Compression
bytes compress(const bytes& data);

struct V5SystemVersion {
  using str = std::string;
  str version;
  int product;
  int productFlags;
};

struct V5SystemStatus {
  using str = std::string;
  str systemVersion;
  str masterCPUVersion;
  str userCPUVersion;
  uint8_t touchVersion;
  uint8_t systemID;
};

struct V5FileMeta {
  using str = std::string;
  uint8_t idx;
  uint32_t size, addr, crc;
  str type;
  uint32_t timestamp, version;
  str name;
  uint32_t linkedVid;
};

struct V5CommandResponse {
  bytes payload;
  uint8_t command;
};

struct V5FileTransfer {
  uint16_t maxPacketSize;
  uint32_t fileSize;
  uint32_t crc;
};

class V5Device {
  using str = std::string;
  int port = -1;
  std::string myFileName;
  public:
  void tx_packet(uint8_t command, const bytes& data = {});
  V5CommandResponse rx_packet();
  bytes txrx_packet(uint8_t command, int len, const bytes& data = {});
  int getc(int timeout = -1);
  bytes readn(int n, int timeout = -1);
  bytes formExtendedPayload(int msg, const bytes& data = {});
  //len == -1 to not check
  bytes txrx_ext_packet(uint8_t command, int len, const bytes& data = {}, bool checkAck = true);
  V5Device(const char* filename): myFileName(filename) {
    reopen();
    /*
    //Reopen w/o O_NDELAY
    close(fd);
    port = open(filename, O_RDWR | O_NOCTTY);
    if(port == -1) {
      fprintf(stderr, "open fail\n");
      exit(1);
    }
    */
  }
  void reopen() {
    if(port != -1) close(port);
    //First, open the file natively.
    int fd = open(myFileName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1) {
      fprintf(stderr, "Failed to connect to port\n");
      exit(1);
    }
    //Wait for any extra input to go into port's buffer
    //(This can occur if the program is stopped and restarted
    //in the middle of communication, or the remote device
    //is restarted.)
    usleep(10000);
    //Once input has arrived, flush it out.
    tcflush(fd,TCIOFLUSH);
    termios term;
    tcgetattr(fd, &term);
    cfmakeraw(&term);
    term.c_cc[VMIN]  = 1;
    term.c_cc[VTIME] = 2;
    tcsetattr(fd, TCSANOW, &term);
    port = fd;
  }
  enum class VID: int { USER = 1, SYSTEM = 15, RMS = 16, PROS = 24, MW = 32, DEV4 = 40, DEV5 = 48, DEV6 = 56, VEX = 240, UNDEF };
  enum class FTChannel: int { PIT = 0, DOWNLOAD = 1 };
  enum class FTCompleteAction: int { NO_ACTION = 0, EXECUTE = 1, RUN_SCREEN = 3 };
  enum class FTDirection: int { UPLOAD = 1, DOWNLOAD = 2 };
  //prosv5 names: DDR, FLASH, SCREEN
  //firm util names: DDR, QSPI, CBUF, VBUF, A1, B1, B2
  enum class FTLocation: int {
    RAM = 0,
    STORAGE = 1,
    FRAMEBUFFER = 2,
    VBUF = 3,
    DDRC = 4,
    DDRE = 5,
    FLASH = 6,
    A1 = 13,
    B1,
    B2
  };
  enum class Product: int { CONTROLLER = 17, BRAIN = 16 };
  enum class ControllerFlags: int { CONNECTED = 2 };

  //High Level File Transfer

  //When where = STORAGE (i.e. flash memory), then
  //a real file will be read and address & length will
  //be ignored. When where = FRAMEBUFFER, a hardcoded
  //address (0) and length corresponding to the size
  //of the screen will be used (557056 bytes).
  //The only case where address and length matter here
  //is for RAM accesses, where there is no metadata at
  //all to help you know how much to read and where. Of
  //course in that case, name will be ignored.
  bytes readFile(
    str name,
    FTCompleteAction onComplete,
    FTLocation where,
    VID vid,
    bool quiet = false,
    uint32_t address = 0,
    uint32_t length = 0
  );
  //V5 knows the difference between an XVX5 header and a gzip one.
  //Both will execute just fine.
  void writeFile(
    bytes data,
    str name,
    bool hasLink,
    const str& linkedName,
    VID linkVID,
    FTCompleteAction onComplete,
    FTLocation where,
    VID vid,
    bool quiet = false,
    uint32_t address = 0,
    int options = 1     //1 in this bitfield allows overwrites
  );

  //Controller
  void enterFTChannel(FTChannel which, double timeout = 5);
  bytes userFIFORead();

  //File Transfer
  V5FileTransfer ftInitialize(
    FTDirection dir,
    const str& name,
    FTLocation where,
    uint32_t crc,
    VID vid,
    uint32_t address = 0,
    uint32_t length = 0,
    int options = 1     //1 in this bitfield allows overwrites
  );
  void ftComplete(FTCompleteAction action);
  void ftWrite(uint32_t addr, const bytes& data);
  bytes ftRead(uint32_t addr, uint32_t len);
  void ftSetLink(const str& to, VID vid, int options = 0);
  void screenCaptureInit();

  //File Management
  //This function is special. It loads the vid's directory for use by getFileMetadataByIdx
  uint16_t getDirCount(VID vid, int options = 0);
  V5FileMeta getFileMetadataByIdx(int idx, int options = 0);
  void execute(const str& name, VID vid, bool run = true);
  V5FileMeta getFileMetadataByName(const str& name, VID vid, int options = 0);
  V5FileMeta getFileMetadataByNameNotBroken(const str& name, VID vid, int options = 0);
  void setProgramFileMetadata(V5FileMeta meta, int options = 0);
  void eraseFile(const str& name, VID vid, bool eraseAll = false);
  uint8_t getProgramFileSlot(const str& name, VID vid, int options = 0);

  //System Info
  V5SystemStatus getSystemStatus();
  V5SystemVersion querySystemVersion();

  ~V5Device() {
    close(port);
  }
};
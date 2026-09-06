#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Version.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint16_t KA9Q_STATUS_PORT = 5006;
constexpr uint8_t KA9Q_STATUS = 0;
constexpr uint8_t KA9Q_COMMAND = 1;
constexpr uint8_t RTP_VERSION = 2;
// radiod measures channel lifetime in processing frames (nominally 50/s).
constexpr unsigned STREAM_LIFETIME_FRAMES = 500; // About 10 seconds.
constexpr unsigned PROBE_LIFETIME_FRAMES = 100;  // About 2 seconds.
constexpr auto KEEPALIVE_INTERVAL = std::chrono::seconds(3);

static double fullNyquistBandwidth(double sampleRate)
{
    // LOW_EDGE and HIGH_EDGE are encoded as floats.  Move each edge one
    // float representable value inside +/-Fs/2, then recover total width.
    return 2.0 * double(std::nextafter(float(sampleRate / 2.0), 0.0f));
}

// Wire-stable values from ka9q-radio src/status.h. Keep this list in order.
enum StatusType : uint8_t {
    EOL = 0, COMMAND_TAG, CMD_CNT, GPS_TIME, DESCRIPTION,
    STATUS_DEST_SOCKET, SETOPTS, CLEAROPTS, RTP_TIMESNAP, BIN_BYTE_DATA,
    INPUT_SAMPRATE, SPECTRUM_BASE, SPECTRUM_AVG, INPUT_SAMPLES,
    WINDOW_TYPE, NOISE_BW, OUTPUT_DATA_SOURCE_SOCKET,
    OUTPUT_DATA_DEST_SOCKET, OUTPUT_SSRC, OUTPUT_TTL, OUTPUT_SAMPRATE,
    OUTPUT_METADATA_PACKETS, OUTPUT_DATA_PACKETS, OUTPUT_ERRORS, CALIBRATE,
    LNA_GAIN, MIXER_GAIN, IF_GAIN, DC_I_OFFSET, DC_Q_OFFSET, IQ_IMBALANCE,
    IQ_PHASE, DIRECT_CONVERSION, RADIO_FREQUENCY, FIRST_LO_FREQUENCY,
    SECOND_LO_FREQUENCY, SHIFT_FREQUENCY, DOPPLER_FREQUENCY,
    DOPPLER_FREQUENCY_RATE, LOW_EDGE, HIGH_EDGE, KAISER_BETA,
    FILTER_BLOCKSIZE, FILTER_FIR_LENGTH, FILTER2, IF_POWER, BASEBAND_POWER,
    NOISE_DENSITY, DEMOD_TYPE, OUTPUT_CHANNELS, INDEPENDENT_SIDEBAND,
    PLL_ENABLE, PLL_LOCK, PLL_SQUARE, PLL_PHASE, PLL_BW, ENVELOPE,
    SNR_SQUELCH, PLL_SNR, FREQ_OFFSET, PEAK_DEVIATION, PL_TONE, AGC_ENABLE,
    HEADROOM, AGC_HANGTIME, AGC_RECOVERY_RATE, FM_SNR, AGC_THRESHOLD, GAIN,
    OUTPUT_LEVEL, OUTPUT_SAMPLES, OPUS_BIT_RATE, MAXDELAY,
    FILTER2_BLOCKSIZE, FILTER2_FIR_LENGTH, FILTER2_KAISER_BETA,
    SPECTRUM_FFT_N, FILTER_DROPS, LOCK, TP1, TP2, UNUSED4,
    AD_BITS_PER_SAMPLE, SQUELCH_OPEN, SQUELCH_CLOSE, PRESET, DEEMPH_TC,
    DEEMPH_GAIN, UNUSED3, PL_DEVIATION, THRESH_EXTEND, SPECTRUM_SHAPE,
    UNUSED2, RESOLUTION_BW, BIN_COUNT, CROSSOVER, BIN_DATA, RF_ATTEN,
    RF_GAIN, RF_AGC, FE_LOW_EDGE, FE_HIGH_EDGE, FE_ISREAL, UNUSED, AD_OVER,
    RTP_PT, STATUS_INTERVAL, OUTPUT_ENCODING, SAMPLES_SINCE_OVER, PLL_WRAPS,
    RF_LEVEL_CAL, OPUS_DTX, OPUS_APPLICATION, OPUS_BANDWIDTH, OPUS_FEC,
    SPECTRUM_STEP, SPECTRUM_OVERLAP, LIFETIME
};

enum Encoding : unsigned { NO_ENCODING = 0, S16LE, S16BE, OPUS, F32LE };

static_assert(OUTPUT_DATA_DEST_SOCKET == 17 && OUTPUT_SSRC == 18 &&
              OUTPUT_SAMPRATE == 20 && RADIO_FREQUENCY == 33 &&
              LOW_EDGE == 39 && HIGH_EDGE == 40 && DEMOD_TYPE == 48 &&
              OUTPUT_CHANNELS == 49 && PRESET == 85 && RTP_PT == 105 &&
              OUTPUT_ENCODING == 107 && LIFETIME == 117,
              "ka9q-radio status TLV numbers changed");

struct Socket {
    int fd = -1;
    Socket() = default;
    explicit Socket(int value) : fd(value) {}
    ~Socket() { if (fd >= 0) ::close(fd); }
    Socket(const Socket &) = delete;
    Socket &operator=(const Socket &) = delete;
    Socket(Socket &&other) noexcept : fd(other.fd) { other.fd = -1; }
    Socket &operator=(Socket &&other) noexcept {
        if (this != &other) {
            if (fd >= 0) ::close(fd);
            fd = other.fd;
            other.fd = -1;
        }
        return *this;
    }
};

struct Endpoint {
    sockaddr_in addr{};
    std::string iface;
};

static uint32_t interfaceAddress(const std::string &name)
{
    if (name.empty()) return htonl(INADDR_ANY);
    ifaddrs *list = nullptr;
    if (::getifaddrs(&list) != 0) throw std::runtime_error("getifaddrs: " + std::string(strerror(errno)));
    uint32_t result = htonl(INADDR_ANY);
    for (const ifaddrs *p = list; p != nullptr; p = p->ifa_next) {
        if (p->ifa_addr != nullptr && p->ifa_addr->sa_family == AF_INET && name == p->ifa_name) {
            result = reinterpret_cast<const sockaddr_in *>(p->ifa_addr)->sin_addr.s_addr;
            break;
        }
    }
    ::freeifaddrs(list);
    if (result == htonl(INADDR_ANY)) throw std::runtime_error("No IPv4 address for interface " + name);
    return result;
}

static Endpoint resolveEndpoint(std::string text, uint16_t defaultPort,
                                const std::string &overrideIface = {})
{
    Endpoint ep;
    const auto comma = text.rfind(',');
    if (comma != std::string::npos) {
        ep.iface = text.substr(comma + 1);
        text.resize(comma);
    }
    if (!overrideIface.empty()) ep.iface = overrideIface;

    std::string port = std::to_string(defaultPort);
    const auto colon = text.rfind(':');
    if (colon != std::string::npos && text.find(':') == colon) {
        port = text.substr(colon + 1);
        text.resize(colon);
    }
    in_addr numeric{};
    if (::inet_pton(AF_INET, text.c_str(), &numeric) != 1 && text.find('.') == std::string::npos)
        text += ".local";

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo *answers = nullptr;
    const int error = ::getaddrinfo(text.c_str(), port.c_str(), &hints, &answers);
    if (error != 0) throw std::runtime_error("getaddrinfo(" + text + "): " + gai_strerror(error));
    std::memcpy(&ep.addr, answers->ai_addr, sizeof(ep.addr));
    ::freeaddrinfo(answers);
    return ep;
}

static bool isMulticast(const in_addr &addr)
{
    return IN_MULTICAST(ntohl(addr.s_addr));
}

static std::string endpointText(const Endpoint &ep)
{
    char address[INET_ADDRSTRLEN]{};
    const char *rendered = ::inet_ntop(AF_INET, &ep.addr.sin_addr,
                                       address, sizeof(address));
    return std::string(rendered == nullptr ? "<invalid-address>" : rendered) +
        ":" + std::to_string(ntohs(ep.addr.sin_port));
}

static Socket multicastListener(const Endpoint &ep)
{
    Socket sock(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (sock.fd < 0) throw std::runtime_error("socket: " + std::string(strerror(errno)));
    int one = 1;
    ::setsockopt(sock.fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#ifdef SO_REUSEPORT
    ::setsockopt(sock.fd, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
#endif
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = ep.addr.sin_port;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock.fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0)
        throw std::runtime_error("bind: " + std::string(strerror(errno)));
    if (isMulticast(ep.addr.sin_addr)) {
        ip_mreq req{};
        req.imr_multiaddr = ep.addr.sin_addr;
        req.imr_interface.s_addr = interfaceAddress(ep.iface);
        if (::setsockopt(sock.fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req)) != 0)
            throw std::runtime_error("IP_ADD_MEMBERSHIP: " + std::string(strerror(errno)));
    }
    return sock;
}

static Socket commandSocket(const Endpoint &ep)
{
    Socket sock(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (sock.fd < 0) throw std::runtime_error("socket: " + std::string(strerror(errno)));
    const uint32_t local = interfaceAddress(ep.iface);
    if (local != htonl(INADDR_ANY))
        ::setsockopt(sock.fd, IPPROTO_IP, IP_MULTICAST_IF, &local, sizeof(local));
    const unsigned char ttl = 1;
    ::setsockopt(sock.fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    return sock;
}

static void putLength(std::vector<uint8_t> &out, size_t length)
{
    if (length < 128) { out.push_back(static_cast<uint8_t>(length)); return; }
    uint8_t bytes[sizeof(size_t)];
    size_t n = 0;
    while (length != 0) { bytes[n++] = static_cast<uint8_t>(length); length >>= 8; }
    out.push_back(static_cast<uint8_t>(0x80 | n));
    while (n != 0) out.push_back(bytes[--n]);
}

static void putTlv(std::vector<uint8_t> &out, StatusType type,
                   const void *data, size_t length)
{
    out.push_back(static_cast<uint8_t>(type));
    putLength(out, length);
    const auto *p = static_cast<const uint8_t *>(data);
    out.insert(out.end(), p, p + length);
}

static void putUnsigned(std::vector<uint8_t> &out, StatusType type, uint64_t value)
{
    uint8_t bytes[8];
    size_t n = 0;
    while (value != 0) { bytes[7 - n++] = static_cast<uint8_t>(value); value >>= 8; }
    putTlv(out, type, bytes + 8 - n, n);
}

static void putFloat(std::vector<uint8_t> &out, StatusType type, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    bits = htonl(bits);
    putTlv(out, type, &bits, sizeof(bits));
}

static void putDouble(std::vector<uint8_t> &out, StatusType type, double value)
{
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    bits = __builtin_bswap64(bits);
#endif
    putTlv(out, type, &bits, sizeof(bits));
}

static uint64_t getUnsigned(const uint8_t *p, size_t length)
{
    uint64_t value = 0;
    for (size_t i = 0; i < length; ++i) value = (value << 8) | p[i];
    return value;
}

static double getFloat(const uint8_t *p, size_t length)
{
    if (length == 0) return 0;
    if (length != 4) return std::numeric_limits<double>::quiet_NaN();
    uint32_t bits;
    std::memcpy(&bits, p, sizeof(bits));
    bits = ntohl(bits);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static double getDouble(const uint8_t *p, size_t length)
{
    if (length == 0) return 0;
    if (length != 8) return std::numeric_limits<double>::quiet_NaN();
    uint64_t bits;
    std::memcpy(&bits, p, sizeof(bits));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    bits = __builtin_bswap64(bits);
#endif
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

struct Status {
    uint64_t tag = 0;
    uint32_t ssrc = 0;
    double frequency = 0;
    double sampleRate = 0;
    double lowEdge = 0;
    double highEdge = 0;
    double firstLoFrequency = 0;
    double feLowEdge = 0;
    double feHighEdge = 0;
    unsigned channels = 0;
    unsigned encoding = NO_ENCODING;
    unsigned payloadType = 0;
    Endpoint data;
    bool hasData = false;
    bool hasFirstLoFrequency = false;
    bool hasFeLowEdge = false;
    bool hasFeHighEdge = false;
};

static bool nextTlv(const uint8_t *&p, const uint8_t *end,
                    StatusType &type, const uint8_t *&value, size_t &length)
{
    if (p >= end) return false;
    type = static_cast<StatusType>(*p++);
    if (type == EOL) return false;
    if (p >= end) return false;
    uint8_t first = *p++;
    if ((first & 0x80) == 0) length = first;
    else {
        const size_t n = first & 0x7f;
        if (n == 0 || n > sizeof(size_t) || size_t(end - p) < n) return false;
        length = 0;
        for (size_t i = 0; i < n; ++i) length = (length << 8) | *p++;
    }
    if (size_t(end - p) < length) return false;
    value = p;
    p += length;
    return true;
}

static bool parseStatus(const uint8_t *packet, size_t length, Status &s)
{
    if (length < 1 || packet[0] != KA9Q_STATUS) return false;
    const uint8_t *p = packet + 1;
    const uint8_t *end = packet + length;
    StatusType type;
    const uint8_t *value;
    size_t n;
    while (nextTlv(p, end, type, value, n)) {
        switch (type) {
        case COMMAND_TAG: s.tag = getUnsigned(value, n); break;
        case OUTPUT_SSRC: s.ssrc = static_cast<uint32_t>(getUnsigned(value, n)); break;
        case RADIO_FREQUENCY: s.frequency = getDouble(value, n); break;
        case FIRST_LO_FREQUENCY:
            s.firstLoFrequency = getDouble(value, n);
            s.hasFirstLoFrequency = true;
            break;
        case OUTPUT_SAMPRATE: s.sampleRate = getUnsigned(value, n); break;
        case LOW_EDGE: s.lowEdge = getFloat(value, n); break;
        case HIGH_EDGE: s.highEdge = getFloat(value, n); break;
        case FE_LOW_EDGE:
            s.feLowEdge = getFloat(value, n);
            s.hasFeLowEdge = true;
            break;
        case FE_HIGH_EDGE:
            s.feHighEdge = getFloat(value, n);
            s.hasFeHighEdge = true;
            break;
        case OUTPUT_CHANNELS: s.channels = getUnsigned(value, n); break;
        case OUTPUT_ENCODING: s.encoding = getUnsigned(value, n); break;
        case RTP_PT: s.payloadType = getUnsigned(value, n); break;
        case OUTPUT_DATA_DEST_SOCKET:
            if (n == 6) {
                s.data.addr.sin_family = AF_INET;
                std::memcpy(&s.data.addr.sin_addr, value, 4);
                std::memcpy(&s.data.addr.sin_port, value + 4, 2);
                s.hasData = true;
            }
            break;
        default: break;
        }
    }
    return true;
}

static uint32_t randomSsrc()
{
    std::random_device rd;
    uint32_t value;
    do { value = (uint32_t(rd()) << 16) ^ uint32_t(rd()); }
    while (value == 0 || value == 0xffffffffU);
    return value;
}

struct KA9QStream {
    Socket dataSocket;
    unsigned payloadType = 0;
    std::vector<std::complex<float>> pending;
    size_t pendingOffset = 0;
    uint16_t expectedSequence = 0;
    uint32_t expectedTimestamp = 0;
    bool haveRtpState = false;
    bool active = false;
};

class SoapyKA9Q final : public SoapySDR::Device {
public:
    explicit SoapyKA9Q(const SoapySDR::Kwargs &args);
    ~SoapyKA9Q() override;

    std::string getDriverKey() const override { return "ka9q"; }
    std::string getHardwareKey() const override { return "radiod"; }
    SoapySDR::Kwargs getHardwareInfo() const override;
    size_t getNumChannels(int direction) const override { return direction == SOAPY_SDR_RX ? 1 : 0; }
    bool getFullDuplex(int, size_t) const override { return false; }
    std::vector<std::string> listAntennas(int direction, size_t channel) const override;
    void setAntenna(int direction, size_t channel, const std::string &name) override;
    std::string getAntenna(int direction, size_t channel) const override;
    std::vector<std::string> getStreamFormats(int, size_t) const override { return {SOAPY_SDR_CF32}; }
    std::string getNativeStreamFormat(int, size_t, double &fullScale) const override {
        fullScale = 1.0; return SOAPY_SDR_CF32;
    }
    SoapySDR::Stream *setupStream(int direction, const std::string &format,
        const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *stream) override;
    size_t getStreamMTU(SoapySDR::Stream *) const override { return 180; }
    int activateStream(SoapySDR::Stream *, int, long long, size_t) override;
    int deactivateStream(SoapySDR::Stream *, int, long long) override;
    int readStream(SoapySDR::Stream *, void *const *, size_t, int &, long long &, long) override;

    void setFrequency(int direction, size_t channel, double frequency,
                      const SoapySDR::Kwargs &) override;
    void setFrequency(int direction, size_t channel, const std::string &name,
                      double frequency, const SoapySDR::Kwargs &) override;
    double getFrequency(int, size_t) const override { return frequency_; }
    double getFrequency(int direction, size_t channel,
                        const std::string &name) const override;
    std::vector<std::string> listFrequencies(int, size_t) const override { return {"RF"}; }
    SoapySDR::RangeList getFrequencyRange(int direction, size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(int direction, size_t channel,
        const std::string &name) const override;
    void setSampleRate(int direction, size_t channel, double rate) override;
    double getSampleRate(int, size_t) const override { return sampleRate_; }
    std::vector<double> listSampleRates(int direction, size_t channel) const override;
    SoapySDR::RangeList getSampleRateRange(int, size_t) const override {
        return {SoapySDR::Range(8000, 1000000, 400)};
    }
    void setBandwidth(int direction, size_t channel, double bw) override;
    double getBandwidth(int, size_t) const override { return bandwidth_; }
    SoapySDR::RangeList getBandwidthRange(int, size_t) const override {
        return {SoapySDR::Range(100, fullNyquistBandwidth(sampleRate_))};
    }

private:
    Status configure(bool destroy = false,
                     unsigned lifetimeFrames = STREAM_LIFETIME_FRAMES);
    void sendKeepalive();
    void startKeepalive();
    void stopKeepalive();
    void keepaliveLoop();
    int receivePacket(KA9QStream &stream, long timeoutUs);
    void requireRx(int direction, size_t channel) const;

    std::string radioName_;
    std::string iface_;
    uint32_t ssrc_;
    Endpoint control_;
    Socket statusSocket_;
    Socket commandSocket_;
    double frequency_ = 0;
    double rfLowEdge_ = 0;
    double rfHighEdge_ = 0;
    double sampleRate_ = 24000;
    double bandwidth_ = fullNyquistBandwidth(24000.0);
    bool bandwidthExplicit_ = false;
    KA9QStream *stream_ = nullptr;
    bool ownsChannel_ = false;
    mutable std::mutex mutex_;
    std::mutex commandMutex_;
    std::mutex keepaliveMutex_;
    std::condition_variable keepaliveCondition_;
    bool keepaliveStop_ = true;
    std::thread keepaliveThread_;
};

SoapyKA9Q::SoapyKA9Q(const SoapySDR::Kwargs &args)
{
    auto it = args.find("radio");
    if (it != args.end()) radioName_ = it->second;
    else if (const char *env = std::getenv("RADIO")) radioName_ = env;
    if (radioName_.empty()) throw std::runtime_error("SoapyKA9Q requires radio=<radiod-name-or-control-group>");
    if ((it = args.find("iface")) != args.end()) iface_ = it->second;
    if ((it = args.find("ssrc")) != args.end()) ssrc_ = std::stoul(it->second, nullptr, 0);
    else ssrc_ = randomSsrc();
    if ((it = args.find("frequency")) != args.end()) frequency_ = std::stod(it->second);
    if ((it = args.find("rate")) != args.end()) sampleRate_ = std::stod(it->second);
    if ((it = args.find("bandwidth")) != args.end()) {
        bandwidth_ = std::stod(it->second);
        bandwidthExplicit_ = true;
    } else {
        bandwidth_ = fullNyquistBandwidth(sampleRate_);
    }

    control_ = resolveEndpoint(radioName_, KA9Q_STATUS_PORT, iface_);
    iface_ = control_.iface;
    statusSocket_ = multicastListener(control_);
    commandSocket_ = commandSocket(control_);
    SoapySDR::logf(SOAPY_SDR_INFO,
        "KA9Q radio=%s control=%s interface=%s",
        radioName_.c_str(), endpointText(control_).c_str(),
        iface_.empty() ? "<default-route>" : iface_.c_str());

    // A short-lived idle channel obtains the current front-end tuning limits
    // without asking radiod to move the shared tuner or emit sample data.
    // This capability probe is deliberately separate from stream lifetime.
    configure(false, PROBE_LIFETIME_FRAMES);
    ownsChannel_ = true;
    configure(true);
    ownsChannel_ = false;
}

SoapyKA9Q::~SoapyKA9Q()
{
    try {
        if (stream_ != nullptr)
            closeStream(reinterpret_cast<SoapySDR::Stream *>(stream_));
        else if (ownsChannel_)
            configure(true);
    } catch (...) {}
}

SoapySDR::Kwargs SoapyKA9Q::getHardwareInfo() const
{
    return {{"radio", radioName_}, {"ssrc", std::to_string(ssrc_)},
            {"transport", "KA9Q status/control + RTP multicast"}};
}

void SoapyKA9Q::requireRx(int direction, size_t channel) const
{
    if (direction != SOAPY_SDR_RX || channel != 0)
        throw std::runtime_error("SoapyKA9Q supports RX channel 0 only");
}

std::vector<std::string> SoapyKA9Q::listAntennas(int direction, size_t channel) const
{
    // CubicSDR queries TX channel 0 even when getNumChannels(TX) is zero.
    // Be permissive for capability discovery so an invalid TX query does not
    // escape its GUI event handler and terminate the application.
    if (direction == SOAPY_SDR_TX) return {};
    requireRx(direction, channel);
    return {"RX"};
}

void SoapyKA9Q::setAntenna(int direction, size_t channel, const std::string &name)
{
    requireRx(direction, channel);
    // CubicSDR may apply an empty saved antenna name even for a one-antenna device.
    if (!name.empty() && name != "RX")
        throw std::runtime_error("Unknown antenna: " + name);
}

std::string SoapyKA9Q::getAntenna(int direction, size_t channel) const
{
    requireRx(direction, channel);
    return "RX";
}

Status SoapyKA9Q::configure(bool destroy, unsigned lifetimeFrames)
{
    std::lock_guard<std::mutex> commandLock(commandMutex_);
    const uint32_t tag = randomSsrc();
    std::vector<uint8_t> command{KA9Q_COMMAND};
    putUnsigned(command, COMMAND_TAG, tag);
    putUnsigned(command, OUTPUT_SSRC, ssrc_);
    putUnsigned(command, LIFETIME, destroy ? 1 : lifetimeFrames);
    if (!destroy) {
        putDouble(command, RADIO_FREQUENCY, frequency_);
        // radiod validates sample rate against the channel's current
        // encoding, so select uncompressed floating-point IQ first.  A new
        // channel may otherwise inherit an Opus default that rejects rates
        // outside the Opus-supported set.
        putUnsigned(command, OUTPUT_ENCODING, F32LE);
        putUnsigned(command, OUTPUT_CHANNELS, 2);
        putUnsigned(command, OUTPUT_SAMPRATE, std::llround(sampleRate_));
        putFloat(command, LOW_EDGE, float(-bandwidth_ / 2));
        putFloat(command, HIGH_EDGE, float(bandwidth_ / 2));
        putUnsigned(command, DEMOD_TYPE, 0);
        putUnsigned(command, ENVELOPE, 0);
        putUnsigned(command, PLL_ENABLE, 0);
        putUnsigned(command, AGC_ENABLE, 0);
        putUnsigned(command, MAXDELAY, 0);
    }
    command.push_back(EOL);

    if (!destroy) {
        SoapySDR::logf(SOAPY_SDR_INFO,
            "KA9Q command frequency=%.0f rate=%.0f bandwidth=%.0f",
            frequency_, sampleRate_, bandwidth_);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
        const ssize_t sent = ::sendto(commandSocket_.fd, command.data(), command.size(), 0,
            reinterpret_cast<const sockaddr *>(&control_.addr), sizeof(control_.addr));
        if (sent != ssize_t(command.size())) {
            const int error = errno;
            throw std::runtime_error("radiod command send to " + endpointText(control_) +
                " via " + (iface_.empty() ? std::string("default route") : iface_) +
                " failed: " + std::string(strerror(error)));
        }
        if (destroy) return {};

        const auto resendUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        while (std::chrono::steady_clock::now() < resendUntil) {
            pollfd pfd{statusSocket_.fd, POLLIN, 0};
            if (::poll(&pfd, 1, 100) <= 0) continue;
            uint8_t packet[65536];
            const ssize_t n = ::recv(statusSocket_.fd, packet, sizeof(packet), 0);
            if (n <= 0) continue;
            Status status;
            if (parseStatus(packet, size_t(n), status) && status.ssrc == ssrc_ && status.tag == tag) {
                if (!status.hasData) throw std::runtime_error("radiod status lacks output destination");
                if (status.channels != 2 || status.encoding != F32LE)
                    throw std::runtime_error("radiod did not accept two-channel F32LE IQ output");
                frequency_ = status.frequency;
                sampleRate_ = status.sampleRate;
                bandwidth_ = status.highEdge - status.lowEdge;
                if (!status.hasFirstLoFrequency || !status.hasFeLowEdge ||
                    !status.hasFeHighEdge)
                    throw std::runtime_error("radiod status lacks front-end tuning limits");
                const double edge1 = status.firstLoFrequency + status.feLowEdge;
                const double edge2 = status.firstLoFrequency + status.feHighEdge;
                rfLowEdge_ = std::min(edge1, edge2);
                rfHighEdge_ = std::max(edge1, edge2);
                status.data.iface = iface_;
                return status;
            }
        }
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("timed out waiting for radiod status response");
}

void SoapyKA9Q::sendKeepalive()
{
    // Any command naming the channel restarts radiod's lifetime counter.
    // No tag is needed because a keepalive acknowledgement is not useful.
    std::vector<uint8_t> command{KA9Q_COMMAND};
    putUnsigned(command, OUTPUT_SSRC, ssrc_);
    command.push_back(EOL);

    std::lock_guard<std::mutex> commandLock(commandMutex_);
    const ssize_t sent = ::sendto(commandSocket_.fd, command.data(), command.size(), 0,
        reinterpret_cast<const sockaddr *>(&control_.addr), sizeof(control_.addr));
    if (sent != ssize_t(command.size())) {
        const int error = errno;
        throw std::runtime_error("radiod keepalive send to " + endpointText(control_) +
            " via " + (iface_.empty() ? std::string("default route") : iface_) +
            " failed: " + std::string(strerror(error)));
    }
}

void SoapyKA9Q::keepaliveLoop()
{
    std::unique_lock<std::mutex> lock(keepaliveMutex_);
    while (!keepaliveCondition_.wait_for(lock, KEEPALIVE_INTERVAL,
                                         [this] { return keepaliveStop_; })) {
        lock.unlock();
        try {
            sendKeepalive();
        } catch (const std::exception &error) {
            SoapySDR::logf(SOAPY_SDR_WARNING,
                "KA9Q channel keepalive failed: %s", error.what());
        }
        lock.lock();
    }
}

void SoapyKA9Q::startKeepalive()
{
    stopKeepalive();
    {
        std::lock_guard<std::mutex> lock(keepaliveMutex_);
        keepaliveStop_ = false;
    }
    keepaliveThread_ = std::thread(&SoapyKA9Q::keepaliveLoop, this);
}

void SoapyKA9Q::stopKeepalive()
{
    {
        std::lock_guard<std::mutex> lock(keepaliveMutex_);
        keepaliveStop_ = true;
    }
    keepaliveCondition_.notify_all();
    if (keepaliveThread_.joinable()) keepaliveThread_.join();
}

SoapySDR::Stream *SoapyKA9Q::setupStream(int direction, const std::string &format,
    const std::vector<size_t> &channels, const SoapySDR::Kwargs &)
{
    std::lock_guard<std::mutex> lock(mutex_);
    requireRx(direction, channels.empty() ? 0 : channels.front());
    if (channels.size() > 1 || format != SOAPY_SDR_CF32)
        throw std::runtime_error("SoapyKA9Q supports one CF32 RX stream only");
    if (stream_ != nullptr) throw std::runtime_error("SoapyKA9Q stream already open");
    auto *stream = new KA9QStream;
    stream_ = stream;
    return reinterpret_cast<SoapySDR::Stream *>(stream);
}

void SoapyKA9Q::closeStream(SoapySDR::Stream *opaque)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto *stream = reinterpret_cast<KA9QStream *>(opaque);
    if (stream == nullptr || stream != stream_) return;
    stream->active = false;
    stopKeepalive();
    if (ownsChannel_) {
        configure(true);
        ownsChannel_ = false;
    }
    stream_ = nullptr;
    delete stream;
}

int SoapyKA9Q::activateStream(SoapySDR::Stream *opaque, int flags,
                              long long, size_t)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    auto *stream = reinterpret_cast<KA9QStream *>(opaque);
    if (stream == nullptr || stream != stream_) return SOAPY_SDR_STREAM_ERROR;
    if (stream->active) return 0;
    SoapySDR::logf(SOAPY_SDR_INFO,
        "KA9Q activate frequency=%.0f rate=%.0f bandwidth=%.0f",
        frequency_, sampleRate_, bandwidth_);
    try {
        Status status = configure();
        ownsChannel_ = true;
        stream->dataSocket = multicastListener(status.data);
        stream->payloadType = status.payloadType;
    } catch (const std::exception &error) {
        if (ownsChannel_) {
            try { configure(true); } catch (...) {}
            ownsChannel_ = false;
        }
        SoapySDR::logf(SOAPY_SDR_ERROR,
            "KA9Q stream activation failed: %s", error.what());
        return SOAPY_SDR_STREAM_ERROR;
    }
    stream->active = true;
    stream->pending.clear();
    stream->pendingOffset = 0;
    stream->haveRtpState = false;
    startKeepalive();
    return 0;
}

int SoapyKA9Q::deactivateStream(SoapySDR::Stream *opaque, int flags, long long)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    auto *stream = reinterpret_cast<KA9QStream *>(opaque);
    if (stream == nullptr || stream != stream_) return SOAPY_SDR_STREAM_ERROR;
    stream->active = false;
    stopKeepalive();
    stream->dataSocket = Socket();
    if (ownsChannel_) {
        try {
            configure(true);
        } catch (const std::exception &error) {
            SoapySDR::logf(SOAPY_SDR_WARNING,
                "KA9Q channel deletion failed: %s", error.what());
            ownsChannel_ = false;
            return SOAPY_SDR_STREAM_ERROR;
        }
        ownsChannel_ = false;
    }
    return 0;
}

int SoapyKA9Q::receivePacket(KA9QStream &stream, long timeoutUs)
{
    pollfd pfd{stream.dataSocket.fd, POLLIN, 0};
    int timeoutMs = timeoutUs < 0 ? -1 : int((timeoutUs + 999) / 1000);
    const int ready = ::poll(&pfd, 1, timeoutMs);
    if (ready == 0) return SOAPY_SDR_TIMEOUT;
    if (ready < 0) return errno == EINTR ? SOAPY_SDR_TIMEOUT : SOAPY_SDR_STREAM_ERROR;

    uint8_t packet[65536];
    const ssize_t received = ::recv(stream.dataSocket.fd, packet, sizeof(packet), 0);
    if (received < 12) return 0;
    const size_t length = size_t(received);
    const unsigned version = packet[0] >> 6;
    if (version != RTP_VERSION) return 0; // Includes in-band KA9Q status packets.
    const bool padding = (packet[0] & 0x20) != 0;
    const bool extension = (packet[0] & 0x10) != 0;
    const size_t csrcCount = packet[0] & 0x0f;
    const unsigned payloadType = packet[1] & 0x7f;
    const uint16_t sequence = uint16_t(packet[2] << 8 | packet[3]);
    const uint32_t timestamp = uint32_t(packet[4]) << 24 | uint32_t(packet[5]) << 16 |
                               uint32_t(packet[6]) << 8 | packet[7];
    const uint32_t ssrc = uint32_t(packet[8]) << 24 | uint32_t(packet[9]) << 16 |
                          uint32_t(packet[10]) << 8 | packet[11];
    if (ssrc != ssrc_) return 0;
    if (payloadType != stream.payloadType) return 0;

    size_t offset = 12 + 4 * csrcCount;
    if (offset > length) return 0;
    if (extension) {
        if (offset + 4 > length) return 0;
        const size_t words = size_t(packet[offset + 2] << 8 | packet[offset + 3]);
        offset += 4 + 4 * words;
        if (offset > length) return 0;
    }
    size_t payloadLength = length - offset;
    if (padding) {
        const unsigned pad = packet[length - 1];
        if (pad == 0 || pad > payloadLength) return 0;
        payloadLength -= pad;
    }
    if (payloadLength % (2 * sizeof(float)) != 0) return 0;
    const size_t samples = payloadLength / (2 * sizeof(float));

    size_t missing = 0;
    if (stream.haveRtpState) {
        const int32_t delta = int32_t(timestamp - stream.expectedTimestamp);
        if (delta > 0 && uint32_t(delta) < uint32_t(sampleRate_)) missing = size_t(delta);
        if (sequence != stream.expectedSequence)
            SoapySDR::logf(SOAPY_SDR_WARNING, "SoapyKA9Q RTP sequence gap: expected %u, got %u",
                           stream.expectedSequence, sequence);
    }
    stream.pending.assign(missing + samples, std::complex<float>{});
    stream.pendingOffset = 0;
    const uint8_t *src = packet + offset;
    for (size_t i = 0; i < samples; ++i) {
        float iq[2];
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        std::memcpy(iq, src + 2 * sizeof(float) * i, sizeof(iq));
#else
        uint32_t words[2];
        std::memcpy(words, src + 2 * sizeof(float) * i, sizeof(words));
        words[0] = __builtin_bswap32(words[0]);
        words[1] = __builtin_bswap32(words[1]);
        std::memcpy(iq, words, sizeof(iq));
#endif
        stream.pending[missing + i] = {iq[0], iq[1]};
    }
    stream.expectedSequence = uint16_t(sequence + 1);
    stream.expectedTimestamp = timestamp + uint32_t(samples);
    stream.haveRtpState = true;
    return int(stream.pending.size());
}

int SoapyKA9Q::readStream(SoapySDR::Stream *opaque, void *const *buffs,
                          size_t numElems, int &flags, long long &timeNs,
                          long timeoutUs)
{
    auto *stream = reinterpret_cast<KA9QStream *>(opaque);
    if (stream == nullptr || stream != stream_) return SOAPY_SDR_STREAM_ERROR;
    if (!stream->active) return SOAPY_SDR_TIMEOUT;
    flags = 0;
    timeNs = 0;
    auto *out = static_cast<std::complex<float> *>(buffs[0]);
    size_t copied = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(std::max(0L, timeoutUs));
    while (copied < numElems) {
        const size_t available = stream->pending.size() - stream->pendingOffset;
        if (available != 0) {
            const size_t count = std::min(available, numElems - copied);
            std::copy_n(stream->pending.data() + stream->pendingOffset, count, out + copied);
            stream->pendingOffset += count;
            copied += count;
            if (stream->pendingOffset == stream->pending.size()) {
                stream->pending.clear(); stream->pendingOffset = 0;
            }
            continue;
        }
        long remaining = timeoutUs;
        if (timeoutUs >= 0) {
            remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0) break;
        }
        const int result = receivePacket(*stream, remaining);
        if (result < 0) return copied != 0 ? int(copied) : result;
    }
    return copied != 0 ? int(copied) : SOAPY_SDR_TIMEOUT;
}

void SoapyKA9Q::setFrequency(int direction, size_t channel, double frequency,
                             const SoapySDR::Kwargs &)
{
    requireRx(direction, channel);
    if (!std::isfinite(frequency) || frequency < 0)
        throw std::runtime_error("invalid frequency");
    if (frequency != 0 &&
        (frequency < rfLowEdge_ || frequency > rfHighEdge_)) {
        const double applied = std::min(std::max(frequency, rfLowEdge_), rfHighEdge_);
        SoapySDR::logf(SOAPY_SDR_WARNING,
            "KA9Q requested frequency %.0f Hz is outside current passband "
            "%.0f..%.0f Hz; using %.0f Hz",
            frequency, rfLowEdge_, rfHighEdge_, applied);
        frequency_ = applied;
    } else {
        frequency_ = frequency;
    }
    if (stream_ != nullptr && stream_->active) configure();
}

void SoapyKA9Q::setFrequency(int direction, size_t channel,
                             const std::string &name, double frequency,
                             const SoapySDR::Kwargs &args)
{
    if (name != "RF")
        throw std::runtime_error("Unknown frequency component: " + name);
    setFrequency(direction, channel, frequency, args);
}

double SoapyKA9Q::getFrequency(int direction, size_t channel,
                               const std::string &name) const
{
    requireRx(direction, channel);
    if (name != "RF")
        throw std::runtime_error("Unknown frequency component: " + name);
    return frequency_;
}

SoapySDR::RangeList SoapyKA9Q::getFrequencyRange(
    int direction, size_t channel) const
{
    requireRx(direction, channel);
    return {SoapySDR::Range(rfLowEdge_, rfHighEdge_)};
}

SoapySDR::RangeList SoapyKA9Q::getFrequencyRange(
    int direction, size_t channel, const std::string &name) const
{
    requireRx(direction, channel);
    if (name != "RF") return {};
    return {SoapySDR::Range(rfLowEdge_, rfHighEdge_)};
}

void SoapyKA9Q::setSampleRate(int direction, size_t channel, double rate)
{
    requireRx(direction, channel);
    const long long integerRate = std::llround(rate);
    if (!std::isfinite(rate) || rate < 1 ||
        std::fabs(rate - double(integerRate)) > 0.001 ||
        integerRate % 400 != 0)
        throw std::runtime_error("sample rate must be an integer multiple of 400 Hz");
    SoapySDR::logf(SOAPY_SDR_INFO,
        "KA9Q setSampleRate requested=%.0f active=%s",
        rate, stream_ != nullptr && stream_->active ? "yes" : "no");
    sampleRate_ = rate;
    if (!bandwidthExplicit_ || bandwidth_ >= rate)
        bandwidth_ = fullNyquistBandwidth(rate);
    if (stream_ != nullptr && stream_->active) configure();
}

std::vector<double> SoapyKA9Q::listSampleRates(int direction, size_t channel) const
{
    requireRx(direction, channel);
    return {8000, 12000, 16000, 24000, 32000, 48000,
            96000, 192000, 384000, 768000, 1000000};
}

void SoapyKA9Q::setBandwidth(int direction, size_t channel, double bw)
{
    requireRx(direction, channel);
    if (!std::isfinite(bw) || bw <= 0 || bw >= sampleRate_)
        throw std::runtime_error("bandwidth must be greater than zero and less than the sample rate");
    bandwidth_ = bw;
    bandwidthExplicit_ = true;
    if (stream_ != nullptr && stream_->active) configure();
}

static SoapySDR::KwargsList findKA9Q(const SoapySDR::Kwargs &args)
{
    SoapySDR::Kwargs result = args;
    if (result.find("radio") == result.end()) {
        const char *radio = std::getenv("RADIO");
        if (radio == nullptr || *radio == '\0') return {};
        result["radio"] = radio;
    }
    result["driver"] = "ka9q";
    result["label"] = "KA9Q radiod " + result["radio"];
    return {result};
}

static SoapySDR::Device *makeKA9Q(const SoapySDR::Kwargs &args)
{
    return new SoapyKA9Q(args);
}

static SoapySDR::Registry registerKA9Q(
    "ka9q", &findKA9Q, &makeKA9Q, SOAPY_SDR_ABI_VERSION);

} // namespace

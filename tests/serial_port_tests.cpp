#include <gtest/gtest.h>

#include "io/serial_port.hpp"

#include <array>
#include <climits>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <unistd.h>

namespace {

class SerialPortTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_ = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
        ASSERT_GE(master_, 0);
        ASSERT_EQ(grantpt(master_), 0);
        ASSERT_EQ(unlockpt(master_), 0);
        std::array<char, 128> path{};
        ASSERT_EQ(ptsname_r(master_, path.data(), path.size()), 0);
        path_ = path.data();
        observer_ = ::open(path_.c_str(), O_RDONLY | O_NOCTTY | O_CLOEXEC);
        ASSERT_GE(observer_, 0);
    }

    void TearDown() override {
        if (observer_ >= 0)
            ::close(observer_);
        if (master_ >= 0)
            ::close(master_);
    }

    int master_ = -1;
    int observer_ = -1;
    std::string path_;
};

TEST_F(SerialPortTest, ReceivesBytesThroughAPseudoTerminal) {
    auto port = io::SerialPort::Open(path_.c_str());
    const std::array<std::uint8_t, 5> expected{0x54, 0x2C, 0, 0xFF, 0x0A};
    ASSERT_EQ(::write(master_, expected.data(), expected.size()), static_cast<ssize_t>(expected.size()));
    std::array<std::uint8_t, 5> actual{};
    std::size_t used = 0;
    while (used < actual.size()) {
        const auto result = port.read_some(std::span{actual}.subspan(used), std::chrono::milliseconds{200});
        ASSERT_EQ(result.status, io::ReadStatus::Data);
        ASSERT_GT(result.size, 0U);
        used += result.size;
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(SerialPortTest, ReportsTimeoutWhenNoDataArrives) {
    auto port = io::SerialPort::Open(path_.c_str());
    std::array<std::uint8_t, 1> buffer{};
    const auto result = port.read_some(buffer, std::chrono::milliseconds{10});
    EXPECT_EQ(result.status, io::ReadStatus::Timeout);
    EXPECT_EQ(result.size, 0U);
    EXPECT_FALSE(result.error);
}

TEST_F(SerialPortTest, ConfiguresRaw230400AndRestoresOriginalSettings) {
    termios before{}, active{}, after{};
    ASSERT_EQ(tcgetattr(observer_, &before), 0);
    {
        auto port = io::SerialPort::Open(path_.c_str());
        ASSERT_EQ(tcgetattr(observer_, &active), 0);
        EXPECT_EQ(cfgetispeed(&active), B230400);
        EXPECT_EQ(cfgetospeed(&active), B230400);
        EXPECT_EQ(active.c_cflag & CSIZE, static_cast<tcflag_t>(CS8));
        EXPECT_EQ(active.c_cflag & (PARENB | CSTOPB | CRTSCTS), 0U);
        EXPECT_EQ(active.c_lflag & (ICANON | ECHO | ISIG), 0U);
    }
    ASSERT_EQ(tcgetattr(observer_, &after), 0);
    EXPECT_EQ(after.c_iflag, before.c_iflag);
    EXPECT_EQ(after.c_oflag, before.c_oflag);
    EXPECT_EQ(after.c_cflag, before.c_cflag);
    EXPECT_EQ(after.c_lflag, before.c_lflag);
    EXPECT_EQ(cfgetispeed(&after), cfgetispeed(&before));
    EXPECT_EQ(cfgetospeed(&after), cfgetospeed(&before));
    for (std::size_t i = 0; i < NCCS; ++i)
        EXPECT_EQ(after.c_cc[i], before.c_cc[i]);
}

TEST_F(SerialPortTest, ReportsDisconnectWithoutRetryingForever) {
    auto port = io::SerialPort::Open(path_.c_str());
    ASSERT_EQ(::close(master_), 0);
    master_ = -1;
    std::array<std::uint8_t, 1> buffer{};
    EXPECT_EQ(port.read_some(buffer, std::chrono::milliseconds{200}).status, io::ReadStatus::Disconnected);
}

TEST_F(SerialPortTest, RejectsInvalidReadParameters) {
    auto port = io::SerialPort::Open(path_.c_str());
    std::array<std::uint8_t, 1> buffer{};
    EXPECT_THROW(port.read_some({}, std::chrono::milliseconds{0}), std::invalid_argument);
    EXPECT_THROW(port.read_some(buffer, std::chrono::milliseconds{-1}), std::invalid_argument);
    EXPECT_THROW(port.read_some(buffer, std::chrono::milliseconds{static_cast<long long>(INT_MAX) + 1}), std::invalid_argument);
}

TEST(SerialPortOpenTest, RejectsEmptyPaths) {
    EXPECT_THROW(io::SerialPort::Open(nullptr), std::invalid_argument);
    EXPECT_THROW(io::SerialPort::Open(""), std::invalid_argument);
}

TEST(SerialPortOpenTest, ReportsNonexistentDevice) {
    EXPECT_THROW(io::SerialPort::Open("/dev/lidar-test-nonexistent-port"), std::system_error);
}

} // namespace

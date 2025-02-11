#ifndef _MA_TRANSPORT_SERIAL_H_
#define _MA_TRANSPORT_SERIAL_H_

#include <porting/ma_transport.h>

namespace ma {

class Serial final : public Transport {
   public:
    Serial() noexcept;
    ~Serial() noexcept;

    ma_err_t init(const void* config) noexcept override;
    void     deInit() noexcept override;

    size_t available() const noexcept override;
    size_t send(const char* data, size_t length) noexcept override;
    size_t flush() noexcept override;
    size_t receive(char* data, size_t length) noexcept override;
    size_t receiveIf(char* data, size_t length, char delimiter) noexcept override;
};

}  // namespace ma

#endif
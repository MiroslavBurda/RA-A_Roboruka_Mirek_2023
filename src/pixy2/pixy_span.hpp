#pragma once

namespace pixy2 {

// This class mimics std::span from c++20 https://en.cppreference.com/w/cpp/container/span
template<typename T>
class PixySpan {
public:
    PixySpan(const T *data = nullptr, size_t size = 0) : m_data(data), m_size(size) {}
    // prevent copy or move, because the data are tied to the parent structure
    PixySpan(const PixySpan&) = delete;
    ~PixySpan() {}

    void reset(const T *data = nullptr, size_t size = 0) {
        m_data = data;
        m_size = size;
    }

    size_t size() const { return m_size; }

    const T* data() const { return m_data; }

    const T* operator[]( size_t idx ) const {
        if(idx >= m_size) {
            ESP_LOGE("PixySpan", "attempted to get idx %d, but only have %d items.", idx, m_size);
            // abort();
        }
        return &(m_data[idx]); // zde dodáno &
    }

    const T *begin() const { return m_data; }
    const T *cbegin() const { return m_data; }

    const T *end() const { return m_data + m_size; }
    const T *cend() const { return m_data + m_size; }

private:
    const T *m_data;
    size_t m_size;
};
};

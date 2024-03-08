#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

namespace ad {

using namespace std;

enum class AnomalyType { Unknown, Normal, Anomaly };

template <typename T> struct AlphaBeta {
    T alpha;
    T beta;
};

template <typename DataType, typename DistType = float, size_t Channels = 3u> class GEDAD final {
   public:
    explicit GEDAD(size_t buffer_size) noexcept
        : _buffer_cidx(0),
          _buffer_size(buffer_size),
          _view_size(0),
          _shift_dist(1),
          _buffer(),
          _window(),
          _euclidean_dist_thresh() {
        // pre-allocate buffer
        for (auto& c : _buffer) {
            c.resize(buffer_size);
        }
    }

    ~GEDAD() = default;

   public:
    void printBuffer() const {
        size_t i = 0;
        cout << "buffer:\n" << fixed;
        for (const auto& c : _buffer) {
            cout << "  " << i++ << ": ";
            for (const auto& d : c) {
                cout << d << " ";
            }
            cout << endl;
        }
    }

    void printWindow() const {
        size_t i = 0;
        cout << "window:\n" << fixed;
        for (const auto& c : _window) {
            cout << "  " << i++ << ": ";
            for (const auto& d : c) {
                cout << d << " ";
            }
            cout << endl;
        }
    }

    void printEuclideanDistThresh() const {
        cout << "euclidean distance threshold:\n";
        for (size_t i = 0; i < _euclidean_dist_thresh.size(); ++i) {
            cout << "  " << i << ": " << _euclidean_dist_thresh[i] << "\n";
        }
    }

    void printCachedView() const {
        size_t i = 0;
        cout << "cached view:\n";
        for (const auto& c : _cached_view) {
            cout << "  " << i++ << ": ";
            for (const auto& d : c) {
                cout << d << " ";
            }
            cout << endl;
        }
    }

    inline void pushToBuffer(const array<DataType, Channels>& channel_data) noexcept {
        auto buffer_cidx = _buffer_cidx.load();
        for (size_t i = 0; i < Channels; ++i) {
            _buffer[i][buffer_cidx % _buffer_size] = channel_data[i];
        }
        _buffer_cidx.store(buffer_cidx + 1);
    }

    void calEuclideanDistThresh(size_t                                      window_start,
                                size_t                                      window_size,
                                size_t                                      sample_start,
                                size_t                                      sample_size,
                                size_t                                      view_size,
                                size_t                                      batch_size,
                                size_t                                      shift_dist,
                                size_t                                      minimal_n,
                                const array<AlphaBeta<DistType>, Channels>& thresh_calibration) {
        // verify and assign parameters to member variables
        {
            assert(window_size >= 1);
            assert(sample_size >= 1);
            assert(view_size >= 1);
            assert(batch_size >= 1);
            assert(shift_dist >= 1);
            assert(window_start + window_size <= _buffer_size);
            assert(sample_start + sample_size <= _buffer_size);
            assert(view_size * batch_size - (view_size - shift_dist) * (batch_size - 1) <= sample_size);
            _view_size  = view_size;
            _shift_dist = shift_dist;
        }

        // assign the window buffer
        {
            for (size_t i = 0; i < _buffer.size(); ++i) {
                auto& window_i = _window[i];
                window_i.resize(window_size);
                window_i.shrink_to_fit();
                auto it = begin(_buffer[i]) + window_start;
                copy(it, it + window_size, begin(window_i));
            }

            // debug print window buffer
            printWindow();
        }

        // from range sample_start to (sample_start + sample_size - _view_size)
        // randomly select batch_size views, each view has view_size elements
        vector<size_t> view_index =
          generateRandomViewIndex(sample_start, sample_start + sample_size - _view_size, batch_size);
        {
            // debug print view_index
            cout << "view index:\n  " << fixed;
            for (const auto& i : view_index) {
                cout << "(" << i << "," << i + _view_size << ") ";
            }
            cout << endl;
        }

        // calculate the euclidean distance for each channel
        array<vector<DistType>, Channels> euclidean_dist_per_channel;
        {
            size_t window_end        = window_size - _view_size;
            size_t views_per_channel = window_size / _shift_dist;
            for (size_t i = 0; i < Channels; ++i) {
                // for each view, sum the total euclidean distance
                // by sliding the view with shift_dist on the channel buffer
                const auto&      buffer_i = _buffer[i];
                const auto&      window_i = _window[i];
                vector<DistType> euclidean_dist_per_view;
                euclidean_dist_per_view.reserve(view_index.size());
                for (const auto& j : view_index) {
                    vector<DistType> euclidean_dist_per_slide;
                    euclidean_dist_per_slide.reserve(views_per_channel);
                    // slide view on the channel buffer
                    for (size_t k = 0; k < window_end; k += _shift_dist) {
                        // calculate the euclidean distance
                        DistType euclidean_dist{};
                        for (size_t l = 0; l < _view_size; ++l) {
                            euclidean_dist += pow(buffer_i[j + l] - window_i[k + l], 2);
                        }
                        euclidean_dist_per_slide.emplace_back(sqrt(euclidean_dist));
                    }
                    // sort and take minimal_n euclidean distances
                    sort(begin(euclidean_dist_per_slide), end(euclidean_dist_per_slide), less<DistType>{});
                    euclidean_dist_per_slide.resize(minimal_n);
                    euclidean_dist_per_slide.shrink_to_fit();
                    // calculate the average of the minimal_n euclidean distances
                    euclidean_dist_per_view.emplace_back(
                      accumulate(begin(euclidean_dist_per_slide), end(euclidean_dist_per_slide), DistType{}) /
                      static_cast<DistType>(minimal_n));
                }
                euclidean_dist_per_channel[i].swap(euclidean_dist_per_view);
            }

            // debug print euclidean_dist_per_channel
            {
                size_t i = 0;
                cout << "euclidean distance per channel: " << endl;
                for (const auto& dists : euclidean_dist_per_channel) {
                    cout << "  " << ++i << ": ";
                    for (const auto& d : dists) {
                        cout << d << " ";
                    }
                    cout << endl;
                }
            }
        }

        // calculate and assign the final euclidean distance threshold
        {
            for (size_t i = 0; i < Channels; ++i) {
                const auto& euclidean_dist_per_channel_i = euclidean_dist_per_channel[i];
                auto        eucliden_dist =
                  accumulate(begin(euclidean_dist_per_channel_i), end(euclidean_dist_per_channel_i), DistType{}) /
                  static_cast<DistType>(euclidean_dist_per_channel_i.size());
                _euclidean_dist_thresh[i] = thresh_calibration[i].alpha * eucliden_dist + thresh_calibration[i].beta;
            }

            // debug print
            printEuclideanDistThresh();
        }
    }

    inline AnomalyType isLastViewAnomaly(float anormality_tolerance = 0.05f) const {
        // take last view_size elements from the _buffer (ring buffer)
        // copy them to the _cached_view
        size_t buffer_cidx    = _buffer_cidx.load();
        size_t view_start_idx = (buffer_cidx - _view_size) % _buffer_size;
        for (size_t i = 0; i < Channels; ++i) {
            auto& view_per_channel_i = _cached_view[i];
            // check cache buffer size, resize if necessary
            if (view_per_channel_i.size() != _view_size) [[unlikely]] {
                view_per_channel_i.resize(_view_size);
                view_per_channel_i.shrink_to_fit();
            }
            for (size_t j = 0; j < _view_size; ++j) {
                view_per_channel_i[j] = _buffer[i][(view_start_idx + j) % _buffer_size];
            }
        }

        // debug print cached view
        // printCachedView();

        // calculate the euclidean distance for each channel
        // slide the view with shift_dist on the window buffer
        array<float, Channels> anormality;
        fill(begin(anormality), end(anormality), numeric_limits<float>::max());
        for (size_t i = 0; i < Channels; ++i) {
            const auto& window_i     = _window[i];
            const auto& view_i       = _cached_view[i];
            auto&       anormality_i = anormality[i];
            auto        thresh_i     = _euclidean_dist_thresh[i];
            size_t      window_end   = window_i.size() - _view_size;
            for (size_t j = 0; j < window_end; j += _shift_dist) {
                DistType euclidean_dist{};
                for (size_t k = 0; k < _view_size; ++k) {
                    euclidean_dist += pow(view_i[k] - window_i[j + k], 2);
                }
                euclidean_dist = sqrt(euclidean_dist);
                if (euclidean_dist < thresh_i) [[unlikely]] {
                    anormality_i = 0.f;
                    break;
                } else {
                    anormality_i = min(anormality_i, 1.0f - (thresh_i / euclidean_dist));
                }
            }
        }

        // debug print anormality
        {
            size_t i = 0;
            cout << "anormality: \n";
            for (const auto& a : anormality) {
                cout << "  " << i << ": " << a << endl;
            }
            cout << endl;
        }

        return accumulate(begin(anormality), end(anormality), 0.f) > anormality_tolerance ? AnomalyType::Anomaly
                                                                                          : AnomalyType::Normal;
    }

    inline vector<size_t> generateRandomViewIndex(size_t range_start, size_t range_end, size_t n) const {
        vector<size_t> view_index(n);

        random_device                    rd;
        mt19937                          gen(rd());
        uniform_int_distribution<size_t> dis(range_start, range_end);
        size_t                           rand_index;
        for (auto it = begin(view_index); it != end(view_index); ++it) {
            rand_index = dis(gen);
            if (find(begin(view_index), it, rand_index) != it) {
                continue;  // skip if the index is already in the view_index
            }
            *it = rand_index;
        }

        return view_index;
    }

   private:
    atomic<size_t> _buffer_cidx;
    const size_t   _buffer_size;

    size_t _view_size;
    size_t _shift_dist;

    array<vector<DataType>, Channels> _buffer;
    array<vector<DataType>, Channels> _window;

    array<DistType, Channels> _euclidean_dist_thresh;

    // cache buffer, eliminate memory allocation
    mutable array<vector<DataType>, Channels> _cached_view;
};

}  // namespace ad
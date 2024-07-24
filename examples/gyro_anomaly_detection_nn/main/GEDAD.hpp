#pragma once

#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/micro/compatibility.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/micro_op_resolver.h>
#include <tensorflow/lite/micro/micro_profiler.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <ostream>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dsp.hpp"

// #define AD_DEBUG
#define AD_PERF

#ifdef AD_PERF
    #define AD_PERF_TIME_MS(records, name, func)                                              \
        {                                                                                     \
            auto start = chrono::high_resolution_clock::now();                                \
            func;                                                                             \
            auto end      = chrono::high_resolution_clock::now();                             \
            records[name] = chrono::duration_cast<chrono::milliseconds>(end - start).count(); \
        }
#else
    #define AD_PERF_TIME_MS(records, name, func) func;
#endif

namespace ad {

using namespace std;

template <typename DataType = float, size_t Channels = 3u> class GEDAD {
   public:
    explicit GEDAD(size_t buffer_size) noexcept
        : _buffer_cidx(0), _buffer(), _buffer_size(buffer_size), _cached_view() {
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

   protected:
    void assignChachedView(size_t view_size) {
        size_t buffer_cidx    = _buffer_cidx.load();
        size_t view_start_idx = (buffer_cidx - view_size) % _buffer_size;
        for (size_t i = 0; i < Channels; ++i) {
            auto& view_per_channel_i = _cached_view[i];
            // check cache buffer size, resize if necessary
            if (view_per_channel_i.size() != view_size) [[unlikely]] {
                view_per_channel_i.resize(view_size);
                view_per_channel_i.shrink_to_fit();
            }
            for (size_t j = 0; j < view_size; ++j) {
                view_per_channel_i[j] = _buffer[i][(view_start_idx + j) % _buffer_size];
            }
        }
    }

   private:
    atomic<size_t> _buffer_cidx;

    array<vector<DataType>, Channels> _buffer;

   protected:
    const size_t _buffer_size;

    // cache buffer, eliminate memory allocation
    mutable array<vector<DataType>, Channels> _cached_view;
};

template <typename DataType = float, size_t Channels = 3u> class GEDADNN : public GEDAD<DataType, Channels> {
   public:
    explicit GEDADNN(size_t   buffer_size,
                     size_t   tensor_arena_size,
                     void*    model_data,
                     DataType nyquist_rate    = 6667.0 / 2.0,
                     DataType cutoff_freq     = 160.0,
                     size_t   num_taps        = 200,
                     size_t   mtf_bins        = 64,
                     DataType cwt_scale_start = 9.0,
                     DataType cwt_scale_step  = 1.0) noexcept
        : GEDAD<DataType, Channels>(buffer_size) {
        {
            _op_resolver  = nullptr;
            _interpreter  = nullptr;
            _tensor_arena = nullptr;

            initInterpreter(tensor_arena_size, model_data);
        }

        {
            _nyquist_rate = nyquist_rate;
            _cutoff_freq  = cutoff_freq;
            _num_taps     = num_taps;
        }

        {
            _fir_coefficients    = dsp::firwin<DataType>(_num_taps, _cutoff_freq / _nyquist_rate);
            _lfilter_denominator = {1.0};
        }

        { _paa_segments = _inputs[0]->dims->data[1]; }

        { _mtf_bins = mtf_bins; }

        {
            auto [psi, x] = dsp::integrate_wavelet<DataType>(_cwt_wavelet_type);
            _cwt_psi      = move(psi);
            _cwt_x        = move(x);

            _cwt_scale_start = cwt_scale_start;
            _cwt_scale_step  = cwt_scale_step;

            const auto cwt_num_scales = _inputs[0]->dims->data[2];
            _cwt_scales.resize(cwt_num_scales);
            generate(_cwt_scales.begin(), _cwt_scales.end(), [this, i = 0]() mutable {
                return _cwt_scale_start + (i++ * _cwt_scale_step);
            });
        }
    }

    ~GEDADNN() {
        if (_interpreter != nullptr) {
            delete _interpreter;
            _interpreter = nullptr;
        }

        if (_op_resolver != nullptr) {
            delete _op_resolver;
            _op_resolver = nullptr;
        }
    };

    friend ostream& operator<<(ostream& os, const GEDADNN& gedad) {
        os << "GEDADNN:\n";
        os << "  buffer size: " << gedad._buffer_size << endl;
        os << "  cached view size: " << gedad._cached_view[0].size() << endl;

        os << "  num ops: " << gedad._num_ops << endl;
        os << "  inputs shapes: ";
        for (const auto& i : gedad._inputs) {
            os << "[";
            const char* sep = "";
            for (size_t j = 0; j < i->dims->size; ++j) {
                os << sep << i->dims->data[j];
                sep = ", ";
            }
            os << "] ";
        }
        os << endl;

        os << "  outputs shapes: ";
        for (const auto& i : gedad._outputs) {
            os << "[";
            const char* sep = "";
            for (size_t j = 0; j < i->dims->size; ++j) {
                os << sep << i->dims->data[j];
                sep = ", ";
            }
            os << "] ";
        }
        os << endl;

        os << "  nyquist rate: " << gedad._nyquist_rate << endl;
        os << "  cutoff freq: " << gedad._cutoff_freq << endl;
        os << "  num taps: " << gedad._num_taps << endl;

        os << "  fir coefficients size: " << gedad._fir_coefficients.size() << endl;
        os << "  lfilter denominator size: " << gedad._lfilter_denominator.size() << endl;

        os << "  paa segments: " << gedad._paa_segments << endl;

        os << "  mtf bins: " << gedad._mtf_bins << endl;

        os << "  cwt scale start: " << gedad._cwt_scale_start << endl;
        os << "  cwt scale step: " << gedad._cwt_scale_step << endl;

        os << "  cwt psi size: " << gedad._cwt_psi.size() << endl;
        os << "  cwt x size: " << gedad._cwt_x.size() << endl;

        os << "  cwt scales size: " << gedad._cwt_scales.size() << endl;

        return os;
    }

    decltype(auto) predict(size_t                           view_size,
                           bool                             rescale         = true,
                           DataType                         rescale_squeeze = 0.02,
                           DataType                         rescale_expand  = 50.0,
                           const array<DataType, Channels>& cwt_std         = {1.0, 1.0, 1.0},
                           const array<DataType, Channels>& cwt_mean        = {0.0, 0.0, 0.0}) {
        assert(view_size <= this->_buffer_size);
        assert(view_size > 0);

        AD_PERF_TIME_MS(
          _perf, "pre-process", preProcess(view_size, rescale, rescale_squeeze, rescale_expand, cwt_std, cwt_mean));
        AD_PERF_TIME_MS(_perf, "invoke", _interpreter->Invoke());

        array<DataType, 2> losses;
        AD_PERF_TIME_MS(_perf, "post-process", postProcess(losses));

        return make_pair(losses[0], losses[1]);
    }

    void printPerf() const {
        cout << "GEDADNN Perf:" << endl;
#ifdef AD_PERF
        for (const auto& [k, v] : _perf) {
            cout << "  " << k << ": " << v << "ms" << endl;
        }
#else
        cout << "  perf not enabled" << endl;
#endif
    }

   protected:
    void initInterpreter(size_t tensor_arena_size, void* model_data) {
        if (_op_resolver == nullptr) [[unlikely]] {
            _op_resolver = new tflite::MicroMutableOpResolver<_num_ops>();
        }
        constexpr auto line_begin = __LINE__;
        _op_resolver->AddSlice();
        _op_resolver->AddConv2D();
        _op_resolver->AddDepthwiseConv2D();
        _op_resolver->AddAdd();
        _op_resolver->AddTranspose();
        _op_resolver->AddReshape();
        _op_resolver->AddFullyConnected();
        _op_resolver->AddRelu();
        _op_resolver->AddDequantize();
        _op_resolver->AddExp();
        _op_resolver->AddMul();
        _op_resolver->AddMean();
        _op_resolver->AddSub();
        _op_resolver->AddSquaredDifference();
        _op_resolver->AddRsqrt();
        _op_resolver->AddQuantize();
        _op_resolver->AddSum();
        _op_resolver->AddSqrt();
        _op_resolver->AddTransposeConv();
        _op_resolver->AddDiv();
        constexpr auto line_end = __LINE__;
        assert(line_end - line_begin - 1 <= _num_ops);

        auto model = tflite::GetModel(model_data);
        assert(model != nullptr);

        _tensor_arena = unique_ptr<uint8_t[]>(new (align_val_t(64)) uint8_t[tensor_arena_size]{});
        assert(_tensor_arena != nullptr);

        if (_interpreter == nullptr) [[unlikely]] {
            _interpreter =
              new tflite::MicroInterpreter(model, *_op_resolver, _tensor_arena.get(), tensor_arena_size, nullptr);
        }
        assert(_interpreter != nullptr);

        _interpreter->AllocateTensors();

        {
            const auto inputs = _interpreter->inputs().size();
            assert(inputs == 1);  // only 1 input tensor

            for (size_t i = 0; i < inputs; ++i) {
                _inputs.push_back(_interpreter->input_tensor(i));

                assert(i < _cached_inputs.size());
            }

            assert(_inputs.size() == 1);
            assert(_inputs[0]->dims->size == 4);
            assert(_inputs[0]->dims->data[0] == 2);
            assert(_inputs[0]->dims->data[1] == 32);
            assert(_inputs[0]->dims->data[2] == 32);
            assert(_inputs[0]->dims->data[3] == 3);

            const auto size = _inputs[0]->dims->data[1] * _inputs[0]->dims->data[2] * _inputs[0]->dims->data[3];
            for (auto& i : _cached_inputs) {
                i.resize(size);
            }

            _resize_input_shape[0] = _inputs[0]->dims->data[1];
            _resize_input_shape[1] = _inputs[0]->dims->data[2];
        }

        {
            const auto outputs = _interpreter->outputs().size();
            assert(outputs == 2);  // 2 output tensor

            for (size_t i = 0; i < outputs; ++i) {
                _outputs.push_back(_interpreter->output_tensor(i));

                assert(_outputs[i]->dims->size == 4);
                assert(_outputs[i]->dims->data[0] == 1);
                assert(_outputs[i]->dims->data[1] == 32);
                assert(_outputs[i]->dims->data[2] == 32);
                assert(_outputs[i]->dims->data[3] == 3);

                assert(i < _cached_outputs.size());
                const auto size = _outputs[i]->dims->data[0] * _outputs[i]->dims->data[1] * _outputs[i]->dims->data[2] *
                                  _outputs[i]->dims->data[3];
                _cached_outputs[i].resize(size);
            }

            assert(_outputs.size() == 2);
        }
    }

    void preProcess(size_t                           view_size,
                    bool                             rescale,
                    DataType                         rescale_squeeze,
                    DataType                         rescale_expand,
                    const array<DataType, Channels>& cwt_std,
                    const array<DataType, Channels>& cwt_mean) {
        assert(_fir_coefficients.size() == _num_taps);
        assert(_lfilter_denominator.size() == 1);
        assert(this->_cached_view.size() == Channels);

        this->assignChachedView(view_size);

        const auto input_shape      = _inputs[0]->dims->data;
        const auto input_stride_wh  = input_shape[1] * input_shape[2];
        const auto input_stride_whc = input_stride_wh * input_shape[3];
        const auto input_batch_0    = _inputs[0]->data.int8;
        const auto input_batch_1    = _inputs[0]->data.int8 + input_stride_whc;

        auto& cached_inputs_0 = _cached_inputs[0];
        auto& cached_inputs_1 = _cached_inputs[1];
        assert(cached_inputs_0.size() == input_stride_whc);
        assert(cached_inputs_1.size() == input_stride_whc);

        assert(_inputs[0]->quantization.type == kTfLiteAffineQuantization);
        assert(_inputs[0]->quantization.params != nullptr);
        const auto quantization_params = static_cast<TfLiteAffineQuantization*>(_inputs[0]->quantization.params);
        const auto scale               = quantization_params->scale->data[0];
        const auto zero_point          = quantization_params->zero_point->data[0];
#ifdef AD_DEBUG
        cout << "scale: " << scale << endl;
        cout << "zero point: " << zero_point << endl;
#endif

        for (size_t i = 0; i < Channels; ++i) {
            auto& cached_view_i = this->_cached_view[i];
            assert(cached_view_i.size() == view_size);

            if (rescale) {
                for (auto& v : cached_view_i) {
                    v = floor(v * rescale_squeeze) * rescale_expand;
                }
            }

            AD_PERF_TIME_MS(_perf,
                            string("lfilter ") + to_string(i),
                            dsp::lfilter(_lfilter_ctx, _fir_coefficients, _lfilter_denominator, cached_view_i));
#ifdef AD_DEBUG
            cout << "lfilter coefficients size: " << _fir_coefficients.size() << endl;
            cout << "lfilter denominator size: " << _lfilter_denominator.size() << endl;
            cout << "cached view size: " << cached_view_i.size() << endl;
            cout << "lfilter result size: " << _lfilter_ctx.result.size() << endl;
#endif
            AD_PERF_TIME_MS(_perf,
                            string("cwt ") + to_string(i),
                            (dsp::cwt<DataType, float>)(_cwt_ctx, _lfilter_ctx.result, _cwt_scales, _cwt_psi, _cwt_x));
#ifdef AD_DEBUG
            cout << "cwt psi size: " << _cwt_psi.size() << endl;
            cout << "cwt x size: " << _cwt_x.size() << endl;
            cout << "cwt scales size: " << _cwt_scales.size() << endl;
            cout << "cwt result size: " << _cwt_ctx.result.size() << endl;
            cout << "cwt result shape: ";
            for (size_t j = 0; j < _cwt_ctx.shape.size(); ++j) {
                cout << _cwt_ctx.shape[j] << " ";
            }
            cout << endl;
#endif
            AD_PERF_TIME_MS(_perf,
                            string("resize ") + to_string(i),
                            (dsp::resize<DataType, int32_t, float>)(_resize_ctx,
                                                                    _cwt_ctx.result,
                                                                    _cwt_ctx.shape,
                                                                    _resize_input_shape));
#ifdef AD_DEBUG
            cout << "resize input size: " << _cwt_ctx.result.size() << endl;
            cout << "resize input shape: ";
            for (size_t j = 0; j < _cwt_ctx.shape.size(); ++j) {
                cout << _cwt_ctx.shape[j] << " ";
            }
            cout << endl;
            cout << "resize result size: " << _resize_ctx.result.size() << endl;
            cout << "resize result shape: ";
            for (size_t j = 0; j < _resize_ctx.shape.size(); ++j) {
                cout << _resize_ctx.shape[j] << " ";
            }
            cout << endl;
#endif
            AD_PERF_TIME_MS(
              _perf, string("paa ") + to_string(i), (dsp::paa<DataType>)(_paa_ctx, _lfilter_ctx.result, _paa_segments));
#ifdef AD_DEBUG
            cout << "paa segments: " << _paa_segments << endl;
            cout << "paa result size: " << _paa_ctx.result.size() << endl;
#endif
            AD_PERF_TIME_MS(
              _perf, string("mtf ") + to_string(i), (dsp::mtf<DataType, float>)(_mtf_ctx, _paa_ctx.result, _mtf_bins));
#ifdef AD_DEBUG
            cout << "mtf bins: " << _mtf_bins << endl;
            cout << "mtf result size: " << _mtf_ctx.result.size() << endl;
            cout << "mtf result shape: ";
            for (size_t j = 0; j < _mtf_ctx.shape.size(); ++j) {
                cout << _mtf_ctx.shape[j] << " ";
            }
            cout << endl;
#endif

            const auto& cwt_result = _resize_ctx.result;
            const auto& mtf_result = _mtf_ctx.result;
            assert(_resize_ctx.result.size() == input_stride_wh);
            assert(_mtf_ctx.result.size() == input_stride_wh);

            const auto cwt_std_i  = cwt_std[i];
            const auto cwt_mean_i = cwt_mean[i];
            for (size_t j = 0, k = 0; (j < input_stride_wh) & (j < input_stride_whc); ++j, k += Channels) {
                const auto k_add_i      = k + i;
                const auto cwt_result_j = (cwt_result[j] - cwt_mean_i) / cwt_std_i;

                cached_inputs_0[k_add_i] = cwt_result_j;
                cached_inputs_1[k_add_i] = mtf_result[j];
                input_batch_0[k_add_i]   = round((static_cast<DataType>(cwt_result_j) / scale) + zero_point);
                input_batch_1[k_add_i]   = round((static_cast<DataType>(mtf_result[j]) / scale) + zero_point);
            }
        }
    }

    void postProcess(array<DataType, 2>& losses) {
        const auto outputs = _outputs.size();
        assert(outputs == _cached_outputs.size());
        assert(outputs == losses.size());
        for (size_t i = 0; i < outputs; ++i) {
            const auto output_i            = _outputs[i]->data.int8;
            auto&      cached_output_i     = _cached_outputs[i];
            const auto size                = cached_output_i.size();
            const auto quantization_params = static_cast<TfLiteAffineQuantization*>(_outputs[i]->quantization.params);
            const auto zero_point          = quantization_params->zero_point->data[0];
            const auto scale               = quantization_params->scale->data[0];

            for (size_t j = 0; j < size; ++j) {
                cached_output_i[j] = static_cast<DataType>(output_i[j] - zero_point) * scale;
            }

            assert(_cached_inputs[i].size() == cached_output_i.size());
            losses[i] = dsp::psnr<DataType>(_cached_inputs[i], cached_output_i);
        }
    }

   private:
    static constexpr size_t                   _num_ops = 32;
    tflite::MicroMutableOpResolver<_num_ops>* _op_resolver;
    unique_ptr<uint8_t[]>                     _tensor_arena;
    tflite::MicroInterpreter*                 _interpreter;

    vector<TfLiteTensor*> _inputs;
    vector<TfLiteTensor*> _outputs;

    DataType _nyquist_rate;
    DataType _cutoff_freq;
    size_t   _num_taps;

    vector<DataType>             _fir_coefficients;
    dsp::lfilter_ctx_t<DataType> _lfilter_ctx;
    array<DataType, 1>           _lfilter_denominator;

    dsp::paa_ctx_t<DataType> _paa_ctx;
    size_t                   _paa_segments;

    dsp::mtf_ctx_t<DataType> _mtf_ctx;
    size_t                   _mtf_bins;

    vector<DataType>                    _cwt_psi;
    vector<DataType>                    _cwt_x;
    DataType                            _cwt_scale_start;
    DataType                            _cwt_scale_step;
    vector<DataType>                    _cwt_scales;
    dsp::cwt_ctx_t<DataType>            _cwt_ctx;
    static constexpr dsp::cwt_wavelet_t _cwt_wavelet_type = dsp::cwt_wavelet_t::MORLET;

    dsp::resize_ctx_t<DataType> _resize_ctx;
    array<int32_t, 2>           _resize_input_shape;

    array<vector<DataType>, 2> _cached_inputs;
    array<vector<DataType>, 2> _cached_outputs;

#ifdef AD_PERF
    unordered_map<string, int64_t> _perf;
#endif
};

}  // namespace ad
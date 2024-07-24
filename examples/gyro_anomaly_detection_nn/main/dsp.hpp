#pragma once

#define ENABLE_ASSERT
#define ENABLE_THROW
#define BUILD_TESTS

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#ifdef ENABLE_ASSERT
    #include <cassert>
    #define ENSURE_TRUE(expr) assert(expr)
#else
    #define ENSURE_TRUE(expr) ((void)0)
#endif

#ifdef BUILD_TESTS
    #include <cassert>
    #include <unordered_map>
#endif

namespace dsp {

// MARK: Constans
namespace constants {

static constexpr double DSP_EPS = 1.0e-20;
static constexpr double DSP_PI  = 3.141592653589793238462643383279502884197169399375105820974944;

}  // namespace constants

using namespace constants;

namespace math {

// MARK: Sinc
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) sinc(T x, T eps = static_cast<T>(DSP_EPS)) {
    if (std::abs(x) < eps) {
        x = static_cast<T>(eps);
    }
    x *= static_cast<T>(DSP_PI);
    return static_cast<T>(std::sin(x) / x);
}

}  // namespace math

using namespace math;

// MARK: Hamming Window
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
decltype(auto) hamming_window(size_t width, T alpha = static_cast<T>(0.54), bool sym = true) {
    if (width < 2) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The width must be greater than or equal to 2.");
#else
        return std::vector<T>();
#endif
    }

    std::vector<T> fac(sym ? width : width + 1);
    {
        const auto start = -DSP_PI;
        const auto end   = DSP_PI;
        const auto step  = std::abs(end - start);
        const auto size  = fac.size() - 1;
        ENSURE_TRUE(size != 0);
        std::generate(fac.begin(), fac.end(), [start, step, size, n = 0]() mutable {
            return start + (step * (static_cast<T>(n++) / static_cast<T>(size)));
        });
    }

    const std::array<T, 2> a = {alpha, static_cast<T>(1.0 - alpha)};
    std::vector<T>         w(fac.size(), static_cast<T>(0.0));
    {
        const auto size = w.size();
        ENSURE_TRUE(size == fac.size());
        size_t k = 0;
        for (const auto& ai : a) {
            for (size_t i = 0; i < size; ++i) {
                w[i] += ai * std::cos(static_cast<T>(k) * fac[i]);
            }
            ++k;
        }
    }

    if (!sym) {
        w.pop_back();
    }

    return w;
}

namespace types {

enum class window_t {
    HAMMING,
};

}  // namespace types

using namespace types;

namespace traits {

template <typename T> struct is_complex : std::false_type {};

template <typename T> struct is_complex<std::complex<T>> : std::true_type {};

template <typename T> inline constexpr bool is_complex_v = is_complex<T>::value;

template <typename T, typename = std::void_t<>> struct has_index_access_operator : std::false_type {};

template <typename T>
struct has_index_access_operator<T, std::void_t<decltype(std::declval<T>()[std::declval<size_t>()])>> : std::true_type {
};

template <typename T> constexpr bool has_index_access_operator_v = has_index_access_operator<T>::value;

template <typename T, typename = std::void_t<>> struct has_iterator_support : std::false_type {};

template <typename T>
struct has_iterator_support<T, std::void_t<typename T::iterator, typename T::const_iterator>> : std::true_type {};

template <typename T> constexpr bool has_iterator_support_v = has_iterator_support<T>::value;

template <typename T, typename = std::void_t<>> struct has_random_access_iterator : std::false_type {};

template <typename T>
struct has_random_access_iterator<
  T,
  std::void_t<typename std::iterator_traits<T>::iterator_category, decltype(std::declval<T>().operator+(1))>>
    : std::conditional_t<
        std::is_base_of_v<std::random_access_iterator_tag, typename std::iterator_traits<T>::iterator_category>,
        std::true_type,
        std::false_type> {};

template <typename T> constexpr bool has_random_access_iterator_v = has_random_access_iterator<T>::value;

template <typename T, typename = std::void_t<>> struct has_size_method_with_size_t : std::false_type {};

template <typename T>
struct has_size_method_with_size_t<T, std::void_t<decltype(std::declval<T>().size())>>
    : std::is_same<size_t, decltype(std::declval<T>().size())> {};

template <typename T> constexpr bool has_size_method_with_size_t_v = has_size_method_with_size_t<T>::value;

template <typename Container, typename T, typename = std::void_t<>>
struct has_contained_type_nothrow_convertible_to : std::false_type {};

template <typename Container, typename T>
struct has_contained_type_nothrow_convertible_to<
  Container,
  T,
  std::void_t<decltype(std::declval<typename Container::value_type>()),
              std::enable_if_t<std::is_nothrow_convertible_v<typename Container::value_type, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool has_contained_type_nothrow_convertible_to_v =
  has_contained_type_nothrow_convertible_to<Container, T>::value;

}  // namespace traits

using namespace traits;

// MARK: Firwin
template <typename T = double,
          typename Conatiner,
          std::enable_if_t<has_size_method_with_size_t_v<Conatiner> && has_iterator_support_v<Conatiner> &&
                             has_contained_type_nothrow_convertible_to_v<Conatiner, T>,
                           bool> = true>
decltype(auto) firwin(size_t           numtaps,
                      const Conatiner& cutoffs,
                      bool             pass_zero = true,
                      bool             scale     = true,
                      window_t         window    = window_t::HAMMING) {
    if (cutoffs.size() == 0) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("At least one cutoffs frequency must be given.");
#else
        return std::vector<T>();
#endif
    }

    const bool pass_nyquist = !pass_zero;
    if (pass_nyquist & !(numtaps & 1)) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The number of numtaps must be odd when pass_nyquist is true.");
#else
        return std::vector<T>();
#endif
    }

    std::vector<T> bands;
    {
        if (pass_zero) {
            bands.push_back(static_cast<T>(0.0));
        }
        std::copy(cutoffs.begin(), cutoffs.end(), std::back_inserter(bands));
        if (pass_nyquist) {
            bands.push_back(static_cast<T>(1.0));
        }
    }

    std::vector<T> m(numtaps);
    {
        const auto alpha = 0.5 * static_cast<T>(numtaps - 1);
        const auto size  = m.size();
        ENSURE_TRUE(size == numtaps);
        for (size_t i = 0; i < size; ++i) {
            m[i] = static_cast<T>(i) - static_cast<T>(alpha);
        }
    }

    std::vector<T> h(numtaps, static_cast<T>(0.0));
    {
        const auto size   = bands.size();
        const auto size_h = h.size();
        ENSURE_TRUE(size_h == m.size());
        for (size_t i = 1; i < size; i += 2) {
            const auto left  = bands[i - 1];
            const auto right = bands[i];

            for (size_t j = 0; j < size_h; ++j) {
                const auto& mj = m[j];
                h[j]           = (right * sinc(right * mj)) - (left * sinc(left * mj));
            }
        }
    }

    {
        std::vector<T> win;
        switch (window) {
        case window_t::HAMMING:
            win = hamming_window<T>(numtaps);
            break;

        default:
#ifdef ENABLE_THROW
            throw std::invalid_argument("Unsupported window type.");
#else
            return std::vector<T>();
#endif
        }
        const auto size = h.size();
        ENSURE_TRUE(size == win.size());
        for (size_t i = 0; i < size; ++i) {
            h[i] *= win[i];
        }
    }

    if (scale) {
        ENSURE_TRUE(bands.size() >= 2);
        const auto left  = bands[0];
        const auto right = bands[1];

        T scale_frequency;
        if (left == 0.0) {
            scale_frequency = 0.0;
        } else if (right == 1.0) {
            scale_frequency = 1.0;
        } else {
            scale_frequency = 0.5 * (left + right);
        }

        const auto size = h.size();
        ENSURE_TRUE(size == m.size());
        T sum = 0.0;
        for (size_t i = 0; i < size; ++i) {
            sum += h[i] * std::cos(DSP_PI * m[i] * scale_frequency);
        }
        if (std::abs(sum) < DSP_EPS) [[unlikely]] {
            sum = DSP_EPS;
        }
        for (auto& hi : h) {
            hi /= sum;
        }
    }

    return h;
}

template <typename T = double, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
decltype(auto) firwin(
  size_t numtaps, T cutoff, bool pass_zero = true, bool scale = true, window_t window = window_t::HAMMING) {
    std::array<T, 1> c = {cutoff};
    return firwin<T>(numtaps, c, pass_zero, scale, window);
}

// MARK: Lfilter
namespace traits {

template <typename Container, typename T, typename = std::void_t<>>
struct is_lfilter_container_fine : std::false_type {};

template <typename Container, typename T>
struct is_lfilter_container_fine<
  Container,
  T,
  std::void_t<std::enable_if_t<has_size_method_with_size_t_v<Container> && has_index_access_operator_v<Container> &&
                               has_contained_type_nothrow_convertible_to_v<Container, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool is_lfilter_container_fine_v = is_lfilter_container_fine<Container, T>::value;

}  // namespace traits

using namespace traits;

template <typename T = double> struct lfilter_ctx_t { std::vector<T> result; };

template <typename T = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          std::enable_if_t<std::is_floating_point_v<T> && is_lfilter_container_fine_v<Container_1, T> &&
                             is_lfilter_container_fine_v<Container_2, T> && is_lfilter_container_fine_v<Container_3, T>,
                           bool> = true>
void lfilter(lfilter_ctx_t<T>&  ctx,
             const Container_1& numerators,
             const Container_2& denominators,
             const Container_3& x) {
    const auto nx = x.size();
    auto&      y  = ctx.result;
    if (y.size() != nx) [[unlikely]] {
        y.resize(nx);
        std::fill(y.begin(), y.end(), static_cast<T>(0.0));
    }

    {
        const auto nn = numerators.size();
        const auto nd = denominators.size();
        ENSURE_TRUE(y.size() == nx);
        for (size_t i = 0; i < nx; ++i) {
            const auto i_p1   = i + 1;
            const auto nn_end = std::min(nn, i_p1);
            const auto nd_end = std::min(nd, i_p1);
            auto&      y_i    = y[i];
            T          sum    = 0.0;
            for (size_t j = 0; j < nn_end; ++j) {
                sum += numerators[j] * x[i - j];
            }
            y_i += sum;
            for (size_t j = 1; j < nd_end; ++j) {
                y_i -= denominators[j] * y[i - j];
            }
        }
    }
}

template <typename T = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          std::enable_if_t<std::is_floating_point_v<T> && is_lfilter_container_fine_v<Container_1, T> &&
                             is_lfilter_container_fine_v<Container_2, T> && is_lfilter_container_fine_v<Container_3, T>,
                           bool> = true>
decltype(auto) lfilter(const Container_1& numerators, const Container_2& denominators, const Container_3& x) {
    lfilter_ctx_t<T> ctx;
    lfilter(ctx, numerators, denominators, x);
    const auto r = std::move(ctx.result);
    return r;
}

template <typename T = double,
          typename Container_1,
          typename Container_2,
          std::enable_if_t<std::is_floating_point_v<T> && is_lfilter_container_fine_v<Container_1, T> &&
                             is_lfilter_container_fine_v<Container_2, T>,
                           bool> = true>
decltype(auto) lfilter(const Container_1& numerators, T denominator, const Container_2& x) {
    std::array<T, 1> d = {denominator};
    return lfilter<T>(numerators, d, x);
}

// MARK: Picewise Aggregate Approximation
template <typename T = double> struct paa_ctx_t { std::vector<T> result; };

template <typename T = double,
          typename Container,
          std::enable_if_t<std::is_floating_point_v<T> && has_size_method_with_size_t_v<Container> &&
                             has_index_access_operator_v<Container> &&
                             has_contained_type_nothrow_convertible_to_v<Container, T>,
                           bool> = true>
void paa(paa_ctx_t<T>& ctx, const Container& x, size_t segments) {
    const auto n = x.size();
    auto&      y = ctx.result;
    if (y.size() != segments) [[unlikely]] {
        y.resize(segments);
        std::fill(y.begin(), y.end(), static_cast<T>(0.0));
    }

    {
        for (size_t i = 0; i < segments; ++i) {
            const size_t start = (n * i) / segments;
            const size_t end   = (n * (i + 1)) / segments;
            T            sum   = 0.0;
            for (size_t j = start; (j < end) & (j < n); ++j) {
                sum += x[j];
            }
            auto dist = end - start;
            if (dist != 0) [[likely]] {
                y[i] = sum / static_cast<T>(dist);
            } else [[unlikely]] {
                y[i] = sum / DSP_EPS;
            }
        }
    }
}

template <typename T = double,
          typename Container,
          std::enable_if_t<std::is_floating_point_v<T> && has_size_method_with_size_t_v<Container> &&
                             has_index_access_operator_v<Container> &&
                             has_contained_type_nothrow_convertible_to_v<Container, T>,
                           bool> = true>
decltype(auto) paa(const Container& x, size_t segments) {
    paa_ctx_t<T> ctx;
    paa(ctx, x, segments);
    const auto r = std::move(ctx.result);
    return r;
}

// MARK: Minmax Scale
template <
  typename T = double,
  typename Container,
  std::enable_if_t<has_iterator_support_v<Container> && has_contained_type_nothrow_convertible_to_v<Container, T>,
                   bool> = true>
void minmax_scale(Container& v, T lower, T upper) {
    const auto [min, max] = std::minmax_element(v.begin(), v.end());
    if (min == v.end() || max == v.end()) [[unlikely]] {
        return;
    }

    const auto min_v = *min;
    auto       diff  = *max - min_v;
    if (std::abs(diff) < DSP_EPS) [[unlikely]] {
        diff = DSP_EPS;
    }
    const auto scale = upper - lower;
    for (auto& vi : v) {
        vi = ((vi - min_v) * scale / diff) + lower;
    }
}

template <typename T = double> struct minmax_scale_ctx_t { std::vector<T> result; };

template <
  typename T = double,
  typename Container,
  std::enable_if_t<has_iterator_support_v<Container> && has_contained_type_nothrow_convertible_to_v<Container, T>,
                   bool> = true>
constexpr void minmax_scale(minmax_scale_ctx_t<T>& ctx, const Container& v, T lower, T upper) {
    const auto n = v.size();
    auto&      y = ctx.result;
    if (y.size() != n) [[unlikely]] {
        y.resize(n);
    }
    if constexpr (std::is_nothrow_assignable_v<decltype(ctx.result), Container>) {
        y = v;
    } else {
        ENSURE_TRUE(y.size() == n);
        std::copy(v.begin(), v.end(), y.begin());
    }
    minmax_scale(y, lower, upper);
}

template <typename T = double> struct mtf_ctx_t {
    minmax_scale_ctx_t<T> minmax_ctx;
    std::vector<T>        bins;
    std::vector<size_t>   digitize;
    std::vector<T>        transition_matrix;

    std::vector<T>      result;
    std::vector<size_t> shape;
};

// MARK: Markov Transition Field
template <typename T = double,
          typename P = double,
          typename Container,
          std::enable_if_t<std::is_floating_point_v<P> && has_size_method_with_size_t_v<Container> &&
                             has_index_access_operator_v<Container> &&
                             has_contained_type_nothrow_convertible_to_v<Container, T>,
                           bool> = true>
void mtf(mtf_ctx_t<T>& ctx, const Container& x, size_t n_bins = 16) {
    auto& ctx_minmax_ctx = ctx.minmax_ctx;
    minmax_scale<T>(ctx_minmax_ctx, x, static_cast<T>(0.0), static_cast<T>(1.0));
    const auto& norm = ctx_minmax_ctx.result;

    auto& bins = ctx.bins;
    if (bins.size() != n_bins) [[unlikely]] {
        bins.resize(n_bins);
        auto space = static_cast<P>(n_bins) - 1.0;
        if (std::abs(space) < DSP_EPS) [[unlikely]] {
            space = DSP_EPS;
        }
        std::generate(bins.begin(), bins.end(), [space, n = 0]() mutable {
            return static_cast<T>(n++) * static_cast<T>(1.0) / static_cast<T>(space);
        });
    }

    auto&      digitize = ctx.digitize;
    const auto ns       = norm.size();
    if (digitize.size() != ns) [[unlikely]] {
        digitize.resize(ns);
    }

    {
        ENSURE_TRUE(digitize.size() == ns);
        ENSURE_TRUE(bins.size() == n_bins);
        for (size_t i = 0; i < ns; ++i) {
            const auto ni  = norm[i];
            auto       idx = n_bins - 1;
            for (size_t j = 0; j < n_bins; ++j) {
                if (ni < bins[j]) {
                    idx = j;
                    break;
                }
            }
            digitize[i] = idx >= 1 ? idx - 1 : idx;
        }
    }

    const auto ntm               = n_bins * n_bins;
    auto&      transition_matrix = ctx.transition_matrix;
    if (transition_matrix.size() != ntm) [[unlikely]] {
        transition_matrix.resize(ntm);
        std::fill(transition_matrix.begin(), transition_matrix.end(), static_cast<T>(0.0));
    }
    {
        const size_t start = 1;
        const size_t end   = digitize.size();
        for (size_t p = start; p < end; ++p) {
            const size_t i = digitize[p - 1];
            const size_t j = digitize[p];
            transition_matrix[(i * n_bins) + j] += static_cast<T>(1.0);
        }
    }
    {
        const auto stride = n_bins;
        const auto size   = transition_matrix.size();
        ENSURE_TRUE(size == ntm);
        for (size_t i = 0; i < size; i += stride) {
            T sum = 0.0;
            for (size_t j = 0; j < stride; ++j) {
                sum += transition_matrix[i + j];
            }
            if (std::abs(sum) < DSP_EPS) [[unlikely]] {
                sum = DSP_EPS;
            }
            for (size_t j = 0; j < stride; ++j) {
                transition_matrix[i + j] /= sum;
            }
        }
    }

    auto&      y     = ctx.result;
    auto&      shape = ctx.shape;
    const auto cols  = ns;
    const auto rows  = ns;
    const auto ny    = cols * rows;
    if (y.size() != ny) [[unlikely]] {
        y.resize(ny);
        shape = {cols, rows};
    }
    {
        ENSURE_TRUE(y.size() == ny);
        for (size_t i = 0; i < cols; ++i) {
            const auto i_mul_cols = i * cols;
            const auto digitize_i = digitize[i];

            for (size_t j = 0; j < rows; ++j) {
                const auto idx = i_mul_cols + j;
                y[idx]         = transition_matrix[(digitize_i * n_bins) + digitize[j]];
            }
        }
    }
}

template <typename T = double,
          typename Container,
          std::enable_if_t<has_size_method_with_size_t_v<Container> && has_index_access_operator_v<Container> &&
                             has_contained_type_nothrow_convertible_to_v<Container, T>,
                           bool> = true>
decltype(auto) mtf(const Container& x, size_t n_bins = 16) {
    mtf_ctx_t<T> ctx;
    mtf(ctx, x, n_bins);
    const auto r = std::move(ctx.result);
    return r;
}

// MARK: Resize
namespace types {

enum class resize_interpolation_t {
    BILINEAR,
};

}  // namespace types

using namespace types;

namespace traits {

template <typename Container, typename T, typename = std::void_t<>>
struct is_resize_container_fine : std::false_type {};

template <typename Container, typename T>
struct is_resize_container_fine<
  Container,
  T,
  std::void_t<std::enable_if_t<has_size_method_with_size_t_v<Container> && has_index_access_operator_v<Container> &&
                               has_contained_type_nothrow_convertible_to_v<Container, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool is_resize_container_fine_v = is_resize_container_fine<Container, T>::value;

template <typename Container, typename T, typename = std::void_t<>>
struct is_resize_shape_container_fine : std::false_type {};

template <typename Container, typename T>
struct is_resize_shape_container_fine<
  Container,
  T,
  std::void_t<std::enable_if_t<has_size_method_with_size_t_v<Container> && has_iterator_support_v<Container> &&
                               has_contained_type_nothrow_convertible_to_v<Container, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool is_resize_shape_container_fine_v = is_resize_shape_container_fine<Container, T>::value;

}  // namespace traits

using namespace traits;

template <typename T = double> struct resize_ctx_t {
    std::vector<T>      result;
    std::vector<size_t> shape;
};

template <typename T = double,
          typename P = int16_t,
          typename Q = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          std::enable_if_t<
            std::is_integral_v<P> && std::is_floating_point_v<Q> && is_resize_container_fine_v<Container_1, T> &&
              is_resize_shape_container_fine_v<Container_2, P> && is_resize_shape_container_fine_v<Container_3, P>,
            bool> = true>
void resize(resize_ctx_t<T>&       ctx,
            const Container_1&     m,
            const Container_2&     shape,
            const Container_3&     new_shape,
            resize_interpolation_t interpolation = resize_interpolation_t::BILINEAR) {
    if (shape.size() != 2 || new_shape.size() != 2) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("Unsupported dimensions of shape or new_shape.");
#else
        return;
#endif
    }
    for (const auto& s : shape) {
        if (s < 2) [[unlikely]] {
#ifdef ENABLE_THROW
            throw std::invalid_argument("The shape must be greater than or equal to 2.");
#else
            return;
#endif
        }
    }
    const auto s0        = *shape.begin();
    const auto s1        = *(std::next(shape.begin(), 1));
    const auto s0_mul_s1 = s0 * s1;
    if (static_cast<decltype(s0_mul_s1)>(m.size()) < s0_mul_s1) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of m must be greater than or equal to the product of shape.");
#else
        return;
#endif
    }
    if (s0_mul_s1 > std::numeric_limits<P>::max()) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The product of shape must be less than or equal to the maximum value of type P.");
#else
        return;
#endif
    }

    switch (interpolation) {
    case resize_interpolation_t::BILINEAR: {
        const P w       = static_cast<P>(s1);
        const P h       = static_cast<P>(s0);
        const P w_c     = w - static_cast<P>(2);
        const P h_c     = h - static_cast<P>(2);
        const P new_w   = static_cast<P>(*(std::next(new_shape.begin(), 1)));
        const P new_h   = static_cast<P>(*new_shape.begin());
        const Q scale_x = static_cast<Q>(w) / static_cast<Q>(new_w);
        const Q scale_y = static_cast<Q>(h) / static_cast<Q>(new_h);
        const Q q_0_5   = static_cast<Q>(0.5);
        const P p_1     = static_cast<P>(1);
        const T t_1     = static_cast<T>(1.0);

        if (new_w * new_w > std::numeric_limits<P>::max()) [[unlikely]] {
#ifdef ENABLE_THROW
            throw std::invalid_argument(
              "The product of new shape must be less than or equal to the maximum value of type P.");
#else
            return;
#endif
        }

        auto&   t     = ctx.result;
        const P new_n = new_w * new_h;
        if (static_cast<decltype(new_n)>(t.size()) != new_n) [[unlikely]] {
            t.resize(new_n);
            ctx.shape = {static_cast<size_t>(new_h), static_cast<size_t>(new_w)};
        }

        for (P i = 0; i < new_h; ++i) {
            const P i_mul_new_w = i * new_w;

            for (P j = 0; j < new_w; ++j) {
                const Q x = (static_cast<Q>(static_cast<Q>(j) + q_0_5) * scale_x) - q_0_5;
                const Q y = (static_cast<Q>(static_cast<Q>(i) + q_0_5) * scale_y) - q_0_5;

                const P x0 = static_cast<P>(std::floor(x));
                const P y0 = static_cast<P>(std::floor(y));

                const P x1 = std::min(x0, w_c);
                const P y1 = std::min(y0, h_c);

                const P y1_mul_w    = y1 * w;
                const P y1_p1_mul_w = (y1 + p_1) * w;
                const P x1_p1       = x1 + p_1;

                const T a = m[y1_mul_w + x1];
                const T b = m[y1_mul_w + x1_p1];
                const T c = m[y1_p1_mul_w + x1];
                const T d = m[y1_p1_mul_w + x1_p1];

                const T xd       = x - x1;
                const T yd       = y - y1;
                const T t_1_s_xd = t_1 - xd;
                const T t_1_s_yd = t_1 - yd;

                const T p = (a * t_1_s_xd * t_1_s_yd) + (b * xd * t_1_s_yd) + (c * t_1_s_xd * yd) + (d * xd * yd);

                t[i_mul_new_w + j] = p;
            }
        }
    } break;

    default:
#ifdef ENABLE_THROW
        throw std::invalid_argument("Unsupported interpolation type.");
#else
        return;
#endif
    }
}

template <typename T = double,
          typename P = int16_t,
          typename Q = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          std::enable_if_t<
            std::is_integral_v<P> && std::is_floating_point_v<Q> && is_resize_container_fine_v<Container_1, T> &&
              is_resize_shape_container_fine_v<Container_2, P> && is_resize_shape_container_fine_v<Container_3, P>,
            bool> = true>
decltype(auto) resize(const Container_1&     m,
                      const Container_2&     shape,
                      const Container_3&     new_shape,
                      resize_interpolation_t interpolation = resize_interpolation_t::BILINEAR) {
    resize_ctx_t<T> ctx;
    resize<T, P, Q>(ctx, m, shape, new_shape, interpolation);
    const auto r = std::move(ctx.result);
    return r;
}

// MARK: PSNR
namespace traits {

template <typename Container, typename T, typename = std::void_t<>> struct is_psnr_container_fine : std::false_type {};

template <typename Container, typename T>
struct is_psnr_container_fine<
  Container,
  T,
  std::void_t<std::enable_if_t<has_size_method_with_size_t_v<Container> && has_index_access_operator_v<Container> &&
                               has_contained_type_nothrow_convertible_to_v<Container, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool is_psnr_container_fine_v = is_psnr_container_fine<Container, T>::value;

}  // namespace traits

using namespace traits;

template <typename T = double,
          typename Container,
          std::enable_if_t<std::is_floating_point_v<T> && is_psnr_container_fine_v<Container, T> &&
                             is_psnr_container_fine_v<Container, T>,
                           bool> = true>
decltype(auto) psnr(const Container& target, const Container& preds) {
    const auto n_target = target.size();
    const auto n_preds  = preds.size();
    if (n_target != n_preds) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of target and preds must be equal.");
#else
        return std::numeric_limits<T>::infinity();
#endif
    }
    if (n_target == 0) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of target and preds must be greater than 0.");
#else
        return std::numeric_limits<T>::infinity();
#endif
    }

    T y;
    {
        using V = typename Container::value_type;
        V max   = std::numeric_limits<V>::min();
        T mse   = static_cast<T>(0.0);
        for (size_t i = 0; i < n_target; ++i) {
            const auto target_i = target[i];
            max                 = std::max(max, target_i);
            const auto diff     = preds[i] - target_i;
            mse += diff * diff;
        }
        mse /= static_cast<T>(n_target);
        if (mse < DSP_EPS) [[unlikely]] {
            mse = DSP_EPS;
        }
        y = static_cast<T>(10.0) * std::log10(static_cast<T>(max * max) / mse);
    }

    return y;
}

// MARK: CWT
namespace types {

enum class cwt_wavelet_t {
    MORLET,
};

}  // namespace types

using namespace types;

template <typename T = double, std::enable_if_t<std::is_floating_point_v<T> || is_complex_v<T>, bool> = true>
decltype(auto) integrate_wavelet(cwt_wavelet_t wavelet, size_t percision = 10, T lower = -8.0, T upper = 8.0) {
    if (percision <= 2 || percision >= sizeof(size_t) * 8) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The percision must be greater than 2 or less than the number of bits in size_t.");
#else
        return std::make_pair(std::vector<T>(), std::vector<T>());
#endif
    }
    if (lower >= upper) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The lower must be less than upper.");
#else
        return std::make_pair(std::vector<T>(), std::vector<T>());
#endif
    }

    const size_t   n = 1 << percision;
    std::vector<T> psi(n);
    std::vector<T> x(n);
    const auto     space = upper - lower;

    ENSURE_TRUE(psi.size() == n);
    ENSURE_TRUE(x.size() == n);
    const auto n_s_1 = static_cast<T>(n - 1);
    switch (wavelet) {
    case cwt_wavelet_t::MORLET: {
        for (size_t i = 0; i < n; ++i) {
            const auto t = lower + (space * (static_cast<T>(i) / n_s_1));
            psi[i]       = std::exp(static_cast<T>(-0.5) * t * t) * std::cos(static_cast<T>(5.0) * t);
            x[i]         = t;
        }
    } break;

    default:
#ifdef ENABLE_THROW
        throw std::invalid_argument("Unsupported wavelet type.");
#else
        return std::make_pair(std::vector<T>(), std::vector<T>());
#endif
    }

    const auto step = x[1] - x[0];
    T          sum  = 0.0;
    for (size_t i = 0; i < n; ++i) {
        auto& psi_i = psi[i];
        sum += psi_i;
        psi_i = sum * step;
    }

    return std::make_pair(std::move(psi), std::move(x));
}

template <typename T = double> struct cwt_ctx_t {
    std::vector<size_t> psi_arange;
    std::vector<size_t> psi_indices;
    std::vector<T>      coefficients;

    std::vector<T>      result;
    std::vector<size_t> shape;
};

namespace traits {

template <typename Container, typename T, typename = std::void_t<>> struct is_cwt_container_fine : std::false_type {};

template <typename Container, typename T>
struct is_cwt_container_fine<
  Container,
  T,
  std::void_t<std::enable_if_t<has_size_method_with_size_t_v<Container> && has_index_access_operator_v<Container> &&
                               has_contained_type_nothrow_convertible_to_v<Container, T>>>> : std::true_type {};

template <typename Container, typename T>
constexpr bool is_cwt_container_fine_v = is_cwt_container_fine<Container, T>::value;

}  // namespace traits

using namespace traits;

template <typename T = double,
          typename P = double,
          typename Container_1,
          typename Container_2,
          std::enable_if_t<(std::is_floating_point_v<T> || is_complex_v<T>)&&std::is_floating_point_v<P> &&
                             is_cwt_container_fine_v<Container_1, T> && is_cwt_container_fine_v<Container_2, T>,
                           bool> = true>
void cwt(cwt_ctx_t<T>&         ctx,
         const Container_1&    signal,
         const Container_2&    ascending_scales,
         const std::vector<T>& wavelet_psi,
         const std::vector<T>& wavelet_x) {
    const auto n_psi = wavelet_psi.size();
    const auto n_x   = wavelet_x.size();

    if (n_psi <= 2) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of wavelet_psi must be greater than 2.");
#else
        return;
#endif
    }
    if (n_x <= 2) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of wavelet_x must be greater than 2.");
#else
        return;
#endif
    }
    if (n_x > n_psi) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of wavelet_x must be less than or equal to wavelet_psi.");
#else
        return;
#endif
    }

    const auto x_0      = wavelet_x[0];
    const auto x_1      = wavelet_x[1];
    const auto x_n      = wavelet_x[n_x - 1];
    const auto x_step   = x_1 - x_0;
    const auto x_range  = x_n - x_0;
    const auto n_scales = ascending_scales.size();

    if (std::abs(x_step) <= DSP_EPS) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The step of wavelet_x must be greater than 0.");
#else
        return;
#endif
    }
    if (std::abs(x_range) <= DSP_EPS) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The range of wavelet_x must be greater than 0.");
#else
        return;
#endif
    }
    if (n_scales == 0) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of ascending_scales must be greater than 0.");
#else
        return;
#endif
    }

    const auto max_scale         = ascending_scales[n_scales - 1];
    const auto n_psi_indices_max = static_cast<size_t>(std::ceil((max_scale * x_range) + 1.0));
    auto&      psi_arange        = ctx.psi_arange;
    if (psi_arange.size() != n_psi_indices_max) [[unlikely]] {
        psi_arange.resize(n_psi_indices_max);
        std::iota(psi_arange.begin(), psi_arange.end(), 0);
    }

    auto& psi_indices = ctx.psi_indices;
    if (psi_indices.size() != n_psi_indices_max) [[unlikely]] {
        psi_indices.resize(n_psi_indices_max);
    }

    const auto n_signal = signal.size();
    if (n_signal == 0) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The size of signal must be greater than 0.");
#else
        ctx.result.clear();
        return;
#endif
    }

    const auto n_result = n_scales * n_signal;
    auto&      result   = ctx.result;
    if (result.size() != n_result) [[unlikely]] {
        result.resize(n_result);
        ctx.shape = {n_scales, n_signal};
    }
    const auto n_coefficients = n_signal + n_psi_indices_max - 1;
    auto&      coefficients   = ctx.coefficients;
    if (coefficients.size() != n_coefficients) [[unlikely]] {
        coefficients.resize(n_coefficients);
    }
    {
        ENSURE_TRUE(ascending_scales.size() >= n_scales);
        size_t result_pos = 0;
        for (size_t i = 0; i < n_scales; ++i) {
            const auto scale           = static_cast<P>(ascending_scales[i]);
            size_t     len_psi_indices = 0;

            {
                const auto psi_arange_end = static_cast<size_t>(std::ceil(scale * x_range)) + 1;
                ENSURE_TRUE(psi_arange.size() >= psi_arange_end);
                const auto scale_mul_step = std::max(scale * x_step, static_cast<P>(DSP_EPS));
                for (size_t j = 0; j < psi_arange_end; ++j) {
                    const auto idx = static_cast<size_t>(std::floor(static_cast<P>(psi_arange[j]) / scale_mul_step));
                    if (idx >= n_psi) {
                        break;
                    }
                    psi_indices[len_psi_indices++] = idx;
                }
            }
            if (len_psi_indices < 1) [[unlikely]] {
#ifdef ENABLE_THROW
                throw std::runtime_error("Selected scale is too large.");
#else
                auto it = result.begin() + result_pos;
                std::fill(it, it + n_signal, static_cast<T>(0.0));
                result_pos += n_signal;
                continue;
#endif
            }

            const auto len_psi_idx_s_1 = len_psi_indices - 1;
            const auto len_conv        = n_signal + len_psi_idx_s_1;
            ENSURE_TRUE(len_conv > 1);
            ENSURE_TRUE(coefficients.size() >= len_conv);
            for (size_t j = 0; j < len_conv; ++j) {
                coefficients[j] = static_cast<T>(0.0);
            }
            for (size_t j = 0; j < n_signal; ++j) {
                const auto signal_j = signal[j];
                for (size_t k = 0; k < len_psi_indices; ++k) {
                    const auto  psi_indices_rk = len_psi_idx_s_1 - k;
                    const auto  psi_index_rk   = psi_indices[psi_indices_rk];
                    const auto& psi_rk         = wavelet_psi[psi_index_rk];
                    coefficients[j + k] += signal_j * psi_rk;
                }
            }

            const auto len_diff = len_conv - 1;
            ENSURE_TRUE(len_diff > 1);
            ENSURE_TRUE(coefficients.size() >= len_diff);
            {
                const auto negtive_sqrt_scale = -std::sqrt(scale);
                for (size_t j = 1; j < len_conv; ++j) {
                    auto&       coefficient_prev    = coefficients[j - 1];
                    const auto& coefficient_current = coefficients[j];
                    coefficient_prev                = coefficient_current - coefficient_prev;
                    coefficient_prev *= negtive_sqrt_scale;
                }
            }

            ENSURE_TRUE(len_diff >= n_signal);
            const auto d = static_cast<P>(len_diff - n_signal) / static_cast<P>(2.0);
            if (d < DSP_EPS) [[unlikely]] {
#ifdef ENABLE_THROW
                throw std::runtime_error("Selected scale is too small.");
#else
                auto it = result.begin() + result_pos;
                std::fill(it, it + n_signal, static_cast<T>(0.0));
                result_pos += n_signal;
                continue;
#endif
            }
            const size_t start   = static_cast<size_t>(std::floor(d));
            const size_t end_max = start + n_signal;
            size_t       end     = len_diff - static_cast<size_t>(std::ceil(d));
            if (end > end_max) [[unlikely]] {
                end = end_max;
            }
            for (size_t j = start; j < end; ++j) {
                result[result_pos++] = coefficients[j];
            }
            for (size_t j = end; j < end_max; ++j) {
                result[result_pos++] = static_cast<T>(0.0);
            }
        }
    }
}

template <typename T = double,
          typename P = double,
          typename Container_1,
          typename Container_2,
          std::enable_if_t<(std::is_floating_point_v<T> || is_complex_v<T>)&&std::is_floating_point_v<P> &&
                             is_cwt_container_fine_v<Container_1, T> && is_cwt_container_fine_v<Container_2, T>,
                           bool> = true>
decltype(auto) cwt(const Container_1&    signal,
                   const Container_2&    ascending_scales,
                   const std::vector<T>& wavelet_psi,
                   const std::vector<T>& wavelet_x) {
    cwt_ctx_t<T> ctx;
    cwt<T, P>(ctx, signal, ascending_scales, wavelet_psi, wavelet_x);
    const auto r = std::move(ctx.result);
    return r;
}

// MARK: Tests
namespace tests {

#ifdef BUILD_TESTS

void test_eps() {
    assert(double(DSP_EPS) > 0.0);
    assert(-double(DSP_EPS) < -0.0);
    assert(float(DSP_EPS) > 0.0);
    assert(-float(DSP_EPS) < -0.0);

    double a    = 1.0;
    double b    = 2.0;
    double c    = 3.0;
    double diff = std::abs((a + b) - c);
    assert(diff < DSP_EPS);
}

void test_pi() {
    double math_pi = 2.0 * std::acos(0.0);
    double diff    = std::abs(math_pi - DSP_PI);
    assert(diff < DSP_EPS);
}

void test_sinc() {
    std::unordered_map<double, double> sinc_values = {
      {               -10.0, -3.898171832519376e-17},
      {  -9.797979797979798,  -0.019261976377391934},
      {  -9.595959595959595,   -0.03167529216345295},
      {  -9.393939393939394,    -0.0320209754858246},
      {  -9.191919191919192,   -0.01963689594706574},
      {   -8.98989898989899,  0.0011234069383833456},
      {  -8.787878787878787,   0.022390627055390324},
      {  -8.585858585858587,    0.03573323328381772},
      {  -8.383838383838384,    0.03546686916735175},
      {  -8.181818181818182,   0.021033383197518078},
      {  -7.979797979797979, -0.0025299463342708375},
      {  -7.777777777777778,   -0.02630644082738656},
      {  -7.575757575757576,   -0.04083251432108083},
      {  -7.373737373737374,   -0.03981623910599781},
      {  -7.171717171717171,  -0.022799085369922152},
      {   -6.96969696969697,  0.0043412616727512955},
      {  -6.767676767676768,   0.031360712390831075},
      {  -6.565656565656566,   0.047453364661312565},
      {  -6.363636363636363,    0.04549990608592487},
      {  -6.161616161616162,   0.025116986139960693},
      {  -5.959595959595959,  -0.006761470032864263},
      {  -5.757575757575758,   -0.03815129506783636},
      {  -5.555555555555555,  -0.056425327879361546},
      {  -5.353535353535354,   -0.05327389425526406},
      {  -5.151515151515151,  -0.028313617972092437},
      {   -4.94949494949495,   0.010161320878666369},
      {  -4.747474747474747,    0.04778489884461753},
      {  -4.545454545454546,    0.06931538911162695},
      {  -4.343434343434343,    0.06459757362729442},
      {  -4.141414141414142,    0.03303411947658101},
      { -3.9393939393939394,   -0.01529182990563534},
      {  -3.737373737373738,   -0.06256473652502492},
      { -3.5353535353535355,    -0.0894814635493308},
      {  -3.333333333333333,   -0.08269933431326874},
      { -3.1313131313131315,   -0.04075611340708284},
      {  -2.929292929292929,    0.02393991393455532},
      { -2.7272727272727275,    0.08820627236525579},
      {  -2.525252525252525,    0.12565425718891227},
      { -2.3232323232323235,     0.1164222803677198},
      {  -2.121212121212121,    0.05577180743829834},
      { -1.9191919191919187,  -0.041654451759341626},
      {  -1.717171717171718,   -0.14387325987267854},
      { -1.5151515151515156,   -0.20984657037171237},
      { -1.3131313131313131,     -0.201819279624739},
      { -1.1111111111111107,    -0.0979815536051013},
      { -0.9090909090909101,    0.09864608391270921},
      { -0.7070707070707076,     0.3582369603998354},
      { -0.5050505050505052,     0.6301742431604164},
      {-0.30303030303030276,     0.8556490093311446},
      {-0.10101010101010033,     0.9833009727996326},
      { 0.10101010101010033,     0.9833009727996326},
      { 0.30303030303030276,     0.8556490093311446},
      {  0.5050505050505052,     0.6301742431604164},
      {  0.7070707070707076,     0.3582369603998354},
      {  0.9090909090909083,    0.09864608391271118},
      {  1.1111111111111107,    -0.0979815536051013},
      {  1.3131313131313131,     -0.201819279624739},
      {  1.5151515151515156,   -0.20984657037171237},
      {  1.7171717171717162,   -0.14387325987267932},
      {  1.9191919191919187,  -0.041654451759341626},
      {   2.121212121212121,    0.05577180743829834},
      {  2.3232323232323235,     0.1164222803677198},
      {   2.525252525252524,    0.12565425718891232},
      {  2.7272727272727266,    0.08820627236525595},
      {   2.929292929292929,    0.02393991393455532},
      {  3.1313131313131315,   -0.04075611340708284},
      {   3.333333333333334,   -0.08269933431326888},
      {  3.5353535353535346,   -0.08948146354933081},
      {   3.737373737373737,   -0.06256473652502502},
      {  3.9393939393939394,   -0.01529182990563534},
      {   4.141414141414142,    0.03303411947658101},
      {  4.3434343434343425,    0.06459757362729436},
      {   4.545454545454545,    0.06931538911162698},
      {   4.747474747474747,    0.04778489884461753},
      {    4.94949494949495,   0.010161320878666369},
      {  5.1515151515151505,  -0.028313617972092246},
      {   5.353535353535353,   -0.05327389425526398},
      {   5.555555555555555,  -0.056425327879361546},
      {   5.757575757575758,   -0.03815129506783636},
      {  5.9595959595959584,  -0.006761470032864453},
      {   6.161616161616163,   0.025116986139960693},
      {   6.363636363636363,    0.04549990608592487},
      {   6.565656565656564,   0.047453364661312655},
      {   6.767676767676768,   0.031360712390831075},
      {   6.969696969696969,   0.004341261672751458},
      {   7.171717171717173,  -0.022799085369922416},
      {   7.373737373737374,   -0.03981623910599781},
      {   7.575757575757574,   -0.04083251432108087},
      {   7.777777777777779,   -0.02630644082738656},
      {   7.979797979797979, -0.0025299463342708375},
      {    8.18181818181818,   0.021033383197517852},
      {   8.383838383838384,    0.03546686916735175},
      {   8.585858585858585,   0.035733233283817806},
      {   8.787878787878789,   0.022390627055390116},
      {    8.98989898989899,  0.0011234069383833456},
      {    9.19191919191919,   -0.01963689594706564},
      {   9.393939393939394,    -0.0320209754858246},
      {   9.595959595959595,   -0.03167529216345295},
      {     9.7979797979798,  -0.019261976377391746},
      {                10.0, -3.898171832519376e-17},
    };

    for (const auto& [x, expected] : sinc_values) {
        double result = sinc<double>(x);
        double diff   = std::abs(result - expected);
        assert(diff < 1e-16);
    }

    for (const auto& [x, expected] : sinc_values) {
        double result = sinc<float>(x);
        double diff   = std::abs(result - expected);
        assert(diff < 1e-7);
    }
}

void test_hamming_window() {
    std::vector<double> expected = {
      0.08,       0.08092613, 0.08370079, 0.0883128,  0.0947436,  0.10296729, 0.11295075, 0.12465379, 0.13802929,
      0.15302337, 0.16957568, 0.18761956, 0.20708234, 0.22788567, 0.24994577, 0.27317382, 0.29747628, 0.32275531,
      0.34890909, 0.37583234, 0.40341663, 0.43155089, 0.46012184, 0.48901443, 0.51811232, 0.54729834, 0.57645498,
      0.60546483, 0.63421107, 0.66257795, 0.69045126, 0.71771876, 0.74427064, 0.77,       0.79480323, 0.81858046,
      0.84123594, 0.86267845, 0.88282165, 0.90158442, 0.91889123, 0.93467237, 0.94886431, 0.96140989, 0.97225861,
      0.98136677, 0.9886977,  0.99422189, 0.99791708, 0.99976841, 0.99976841, 0.99791708, 0.99422189, 0.9886977,
      0.98136677, 0.97225861, 0.96140989, 0.94886431, 0.93467237, 0.91889123, 0.90158442, 0.88282165, 0.86267845,
      0.84123594, 0.81858046, 0.79480323, 0.77,       0.74427064, 0.71771876, 0.69045126, 0.66257795, 0.63421107,
      0.60546483, 0.57645498, 0.54729834, 0.51811232, 0.48901443, 0.46012184, 0.43155089, 0.40341663, 0.37583234,
      0.34890909, 0.32275531, 0.29747628, 0.27317382, 0.24994577, 0.22788567, 0.20708234, 0.18761956, 0.16957568,
      0.15302337, 0.13802929, 0.12465379, 0.11295075, 0.10296729, 0.0947436,  0.0883128,  0.08370079, 0.08092613,
      0.08,
    };

    {
        auto w = hamming_window<double>(expected.size());
        assert(w.size() == expected.size());
        for (size_t i = 0; i < w.size(); ++i) {
            double diff = std::abs(w[i] - expected[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto w = hamming_window<float>(expected.size());
        assert(w.size() == expected.size());
        for (size_t i = 0; i < w.size(); ++i) {
            double diff = std::abs(w[i] - expected[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_firwin() {
    std::vector<double> expected = {
      3.63503901e-04,  3.75293735e-04,  -3.96332982e-04, -4.27164335e-04, 4.68341628e-04,  5.20431460e-04,
      -5.84015203e-04, -6.59691447e-04, 7.48078949e-04,  8.49820181e-04,  -9.65585561e-04, -1.09607851e-03,
      1.24204145e-03,  1.40426296e-03,  -1.58358628e-03, -1.78091942e-03, 1.99724724e-03,  2.23364585e-03,
      -2.49129997e-03, -2.77152368e-03, 3.07578565e-03,  3.40573964e-03,  -3.76326187e-03, -4.15049683e-03,
      4.56991398e-03,  5.02437836e-03,  -5.51723930e-03, -6.05244271e-03, 6.63467485e-03,  7.26954799e-03,
      -7.96384327e-03, -8.72583217e-03, 9.56570816e-03,  1.04961751e-02,  -1.15332634e-02, -1.26974835e-02,
      1.40154916e-02,  1.55225556e-02,  -1.72663055e-02, -1.93126286e-02, 2.17552855e-02,  2.47323148e-02,
      -2.84555598e-02, -3.32674503e-02, 3.97597270e-02,  4.90504641e-02,  -6.35359851e-02, -8.94473748e-02,
      1.49633035e-01,  4.49731899e-01,  4.49731899e-01,  1.49633035e-01,  -8.94473748e-02, -6.35359851e-02,
      4.90504641e-02,  3.97597270e-02,  -3.32674503e-02, -2.84555598e-02, 2.47323148e-02,  2.17552855e-02,
      -1.93126286e-02, -1.72663055e-02, 1.55225556e-02,  1.40154916e-02,  -1.26974835e-02, -1.15332634e-02,
      1.04961751e-02,  9.56570816e-03,  -8.72583217e-03, -7.96384327e-03, 7.26954799e-03,  6.63467485e-03,
      -6.05244271e-03, -5.51723930e-03, 5.02437836e-03,  4.56991398e-03,  -4.15049683e-03, -3.76326187e-03,
      3.40573964e-03,  3.07578565e-03,  -2.77152368e-03, -2.49129997e-03, 2.23364585e-03,  1.99724724e-03,
      -1.78091942e-03, -1.58358628e-03, 1.40426296e-03,  1.24204145e-03,  -1.09607851e-03, -9.65585561e-04,
      8.49820181e-04,  7.48078949e-04,  -6.59691447e-04, -5.84015203e-04, 5.20431460e-04,  4.68341628e-04,
      -4.27164335e-04, -3.96332982e-04, 3.75293735e-04,  3.63503901e-04,
    };

    {
        auto h = firwin<double>(100, 0.5);
        assert(h.size() == expected.size());
        for (size_t i = 0; i < h.size(); ++i) {
            double diff = std::abs(h[i] - expected[i]);
            assert(diff < 1e-9);
        }
    }

    {
        auto h = firwin<float>(100, 0.5);
        assert(h.size() == expected.size());
        for (size_t i = 0; i < h.size(); ++i) {
            double diff = std::abs(h[i] - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_lfilter() {
    std::vector<double> data = {
      0.,         0.01010101, 0.02020202, 0.03030303, 0.04040404, 0.05050505, 0.06060606, 0.07070707, 0.08080808,
      0.09090909, 0.1010101,  0.11111111, 0.12121212, 0.13131313, 0.14141414, 0.15151515, 0.16161616, 0.17171717,
      0.18181818, 0.19191919, 0.2020202,  0.21212121, 0.22222222, 0.23232323, 0.24242424, 0.25252525, 0.26262626,
      0.27272727, 0.28282828, 0.29292929, 0.3030303,  0.31313131, 0.32323232, 0.33333333, 0.34343434, 0.35353535,
      0.36363636, 0.37373737, 0.38383838, 0.39393939, 0.4040404,  0.41414141, 0.42424242, 0.43434343, 0.44444444,
      0.45454545, 0.46464646, 0.47474747, 0.48484848, 0.49494949, 0.50505051, 0.51515152, 0.52525253, 0.53535354,
      0.54545455, 0.55555556, 0.56565657, 0.57575758, 0.58585859, 0.5959596,  0.60606061, 0.61616162, 0.62626263,
      0.63636364, 0.64646465, 0.65656566, 0.66666667, 0.67676768, 0.68686869, 0.6969697,  0.70707071, 0.71717172,
      0.72727273, 0.73737374, 0.74747475, 0.75757576, 0.76767677, 0.77777778, 0.78787879, 0.7979798,  0.80808081,
      0.81818182, 0.82828283, 0.83838384, 0.84848485, 0.85858586, 0.86868687, 0.87878788, 0.88888889, 0.8989899,
      0.90909091, 0.91919192, 0.92929293, 0.93939394, 0.94949495, 0.95959596, 0.96969697, 0.97979798, 0.98989899,
      1.,
    };
    std::vector<double> expected = {
      0.00000000e+00,  3.67175657e-06,  1.11343590e-05,  1.45935979e-05,  1.37380456e-05, 1.76132167e-05,
      2.67452714e-05,  2.99781825e-05,  2.65475437e-05,  3.06732579e-05,  4.33830143e-05, 4.63393813e-05,
      3.82242481e-05,  4.26549882e-05,  6.12702026e-05,  6.38895960e-05,  4.85199042e-05, 5.33244271e-05,
      8.06910292e-05,  8.28929852e-05,  5.70997525e-05,  6.23750616e-05,  1.02051781e-04, 1.03715755e-04,
      6.34555178e-05,  6.93560282e-05,  1.26007835e-04,  1.26929952e-04,  6.67162844e-05, 7.35195342e-05,
      1.53752562e-04,  1.53542728e-04,  6.51931752e-05,  7.34669373e-05,  1.87762670e-04, 1.85560793e-04,
      5.51015065e-05,  6.62128426e-05,  2.34117669e-04,  2.27615370e-04,  2.60360137e-05, 4.42070166e-05,
      3.12199382e-04,  2.92761849e-04,  -6.27105342e-05, -1.65695136e-05, 5.25030740e-04, 4.24853367e-04,
      -5.78832843e-04, -7.10742495e-05, 4.97943080e-03,  1.45726823e-02,  2.56773786e-02, 3.58785661e-02,
      4.54379759e-02,  5.54928450e-02,  6.59493275e-02,  7.60697751e-02,  8.59027929e-02, 9.59856320e-02,
      1.06288221e-01,  1.16395734e-01,  1.26328839e-01,  1.36418738e-01,  1.46650207e-01, 1.56753419e-01,
      1.66740134e-01,  1.76832870e-01,  1.87022230e-01,  1.97123450e-01,  2.07144227e-01, 2.17238433e-01,
      2.27399657e-01,  2.37499745e-01,  2.47544104e-01,  2.57639213e-01,  2.67780483e-01, 2.77879830e-01,
      2.87941163e-01,  2.98036898e-01,  3.08163701e-01,  3.18262509e-01,  3.28336153e-01, 3.38432358e-01,
      3.48548738e-01,  3.58647129e-01,  3.68729524e-01,  3.78826103e-01,  3.88935228e-01, 3.99033282e-01,
      4.09121582e-01,  4.19218467e-01,  4.29322907e-01,  4.39420685e-01,  4.49512563e-01, 4.59609698e-01,
      4.69711563e-01,  4.79809114e-01,  4.89902662e-01,  5.00000000e-01,
    };

    auto w = firwin<double>(100, 0.5);

    {
        auto r = lfilter<double>(w, 1.0, data);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto r = lfilter<float>(w, 1.0, data);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_paa() {
    std::vector<double> data = {
      0.,          0.84147098,  0.90929743,  0.14112001,  -0.7568025,  -0.95892427, -0.2794155,  0.6569866,
      0.98935825,  0.41211849,  -0.54402111, -0.99999021, -0.53657292, 0.42016704,  0.99060736,  0.65028784,
      -0.28790332, -0.96139749, -0.75098725, 0.14987721,  0.91294525,  0.83665564,  -0.00885131, -0.8462204,
      -0.90557836, -0.13235175, 0.76255845,  0.95637593,  0.27090579,  -0.66363388, -0.98803162, -0.40403765,
      0.55142668,  0.99991186,  0.52908269,  -0.42818267, -0.99177885, -0.64353813, 0.29636858,  0.96379539,
      0.74511316,  -0.15862267, -0.91652155, -0.83177474, 0.01770193,  0.85090352,  0.90178835,  0.12357312,
      -0.76825466, -0.95375265, -0.26237485, 0.67022918,  0.98662759,  0.39592515,  -0.55878905, -0.99975517,
      -0.521551,   0.43616476,  0.99287265,  0.63673801,  -0.30481062, -0.96611777, -0.7391807,  0.1673557,
      0.92002604,  0.82682868,  -0.02655115, -0.85551998, -0.89792768, -0.11478481, 0.77389068,  0.95105465,
      0.25382336,  -0.67677196, -0.98514626, -0.38778164, 0.56610764,  0.99952016,  0.51397846,  -0.44411267,
      -0.99388865, -0.62988799, 0.31322878,  0.96836446,  0.73319032,  -0.17607562, -0.92345845, -0.82181784,
      0.0353983,   0.86006941,  0.89399666,  0.10598751,  -0.77946607, -0.94828214, -0.24525199, 0.68326171,
      0.98358775,  0.37960774,  -0.57338187, -0.99920683,
    };
    std::vector<double> expected_7 = {
      0.02105659,
      0.0975727,
      0.00562705,
      -0.12440166,
      0.0606417,
      0.08440562,
      -0.10166878,
    };
    std::vector<double> expected_10 = {
      0.19552095,
      -0.18699328,
      0.11828053,
      -0.01149837,
      -0.09898462,
      0.17760873,
      -0.19906823,
      0.15645624,
      -0.06348773,
      -0.04991475,
    };

    {
        auto r = paa<double>(data, 7);
        assert(r.size() == expected_7.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected_7[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto r = paa<float>(data, 10);
        assert(r.size() == expected_10.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected_10[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_minmax_scale() {
    std::vector<double> data = {
      1.,          0.54030231,  -0.41614684, -0.9899925,  -0.65364362, 0.28366219,  0.96017029,  0.75390225,
      -0.14550003, -0.91113026, -0.83907153, 0.0044257,   0.84385396,  0.90744678,  0.13673722,  -0.75968791,
      -0.95765948, -0.27516334, 0.66031671,  0.98870462,  0.40808206,  -0.54772926, -0.99996083, -0.53283302,
      0.42417901,  0.99120281,  0.64691932,  -0.29213881, -0.96260587, -0.74805753, 0.15425145,  0.91474236,
      0.83422336,  -0.01327675, -0.84857027, -0.90369221, -0.12796369, 0.76541405,  0.95507364,  0.26664293,
      -0.66693806, -0.98733928, -0.39998531, 0.5551133,   0.99984331,  0.52532199,  -0.43217794, -0.99233547,
      -0.64014434, 0.30059254,  0.96496603,  0.7421542,   -0.16299078, -0.91828279, -0.82930983, 0.02212676,
      0.85322011,  0.89986683,  0.11918014,  -0.77108022, -0.95241298, -0.25810164, 0.67350716,  0.98589658,
      0.39185723,  -0.56245385, -0.99964746, -0.5177698,  0.44014302,  0.99339038,  0.6333192,   -0.30902273,
      -0.96725059, -0.73619272, 0.17171734,  0.92175127,  0.82433133,  -0.03097503, -0.85780309, -0.89597095,
      -0.11038724, 0.77668598,  0.9496777,   0.24954012,  -0.6800235,  -0.98437664, -0.38369844, 0.56975033,
      0.99937328,  0.51017704,  -0.44807362, -0.99436746, -0.62644445, 0.3174287,   0.96945937,  0.73017356,
      -0.18043045, -0.92514754, -0.81928825, 0.03982088,
    };
    std::vector<double> expected = {
      1.00000000e+00, 7.70146651e-01, 2.91912713e-01, 4.98426252e-03, 1.73161994e-01, 6.41824077e-01, 9.80084753e-01,
      8.76948717e-01, 4.27238765e-01, 4.44161522e-02, 8.04462243e-02, 5.02203099e-01, 9.21925450e-01, 9.53722484e-01,
      5.68360155e-01, 1.20138810e-01, 2.11510873e-02, 3.62405843e-01, 8.30155027e-01, 9.94352198e-01, 7.04035234e-01,
      2.26120212e-01, 0.00000000e+00, 2.33568478e-01, 7.12083864e-01, 9.95601320e-01, 8.23456203e-01, 3.53917941e-01,
      1.86778459e-02, 1.25954115e-01, 5.77117442e-01, 9.57370344e-01, 9.17110057e-01, 4.93351703e-01, 7.56967585e-02,
      4.81352535e-02, 4.36007108e-01, 8.82704729e-01, 9.77536382e-01, 6.33314284e-01, 1.66514644e-01, 6.31089805e-03,
      2.99993632e-01, 7.77552294e-01, 9.99921653e-01, 7.62656346e-01, 2.83897001e-01, 3.81275330e-03, 1.79911767e-01,
      6.50289422e-01, 9.82482671e-01, 8.71074573e-01, 4.18493220e-01, 4.08398200e-02, 8.53271681e-02, 5.11053801e-01,
      9.26608616e-01, 9.49932433e-01, 5.59581441e-01, 1.14442543e-01, 2.37743887e-02, 3.70936861e-01, 8.36750384e-01,
      9.92948153e-01, 6.95922659e-01, 2.18757772e-01, 1.56688283e-04, 2.41100236e-01, 7.20066028e-01, 9.96695125e-01,
      8.16656010e-01, 3.45475816e-01, 1.63554394e-02, 1.31886637e-01, 5.85850559e-01, 9.60874869e-01, 9.12163945e-01,
      4.84502387e-01, 7.10802588e-02, 5.19959582e-02, 4.44795503e-01, 8.88340804e-01, 9.74838356e-01, 6.24762709e-01,
      1.59971799e-01, 7.79224413e-03, 3.08137226e-01, 7.84870953e-01, 9.99686636e-01, 7.55083725e-01, 2.75949010e-01,
      2.79673751e-03, 1.86761847e-01, 6.58707666e-01, 9.84729384e-01, 8.65084138e-01, 4.09773215e-01, 3.74073776e-02,
      9.03380600e-02, 5.19901037e-01,
    };

    {
        auto copy = data;
        minmax_scale<double>(copy, 0.0, 1.0);
        assert(copy.size() == expected.size());
        for (size_t i = 0; i < copy.size(); ++i) {
            double diff = std::abs(copy[i] - expected[i]);
            assert(diff < 1e-8);
        }
    }

    {
        minmax_scale_ctx_t<float> ctx;
        minmax_scale(ctx, data, 0.0f, 1.0f);
        auto r = ctx.result;
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_mtf() {
    std::vector<double> data = {
      0.,
      0.09983342,
      0.19866933,
      0.29552021,
      0.38941834,
      0.47942554,
      0.56464247,
      0.64421769,
      0.71735609,
      0.78332691,
    };
    std::vector<double> expected = {
      0.66666667, 0.66666667, 0.66666667, 0.33333333, 0.33333333, 0.33333333, 0.,         0.,         0.,
      0.,         0.66666667, 0.66666667, 0.66666667, 0.33333333, 0.33333333, 0.33333333, 0.,         0.,
      0.,         0.,         0.66666667, 0.66666667, 0.66666667, 0.33333333, 0.33333333, 0.33333333, 0.,
      0.,         0.,         0.,         0.,         0.,         0.,         0.66666667, 0.66666667, 0.66666667,
      0.33333333, 0.33333333, 0.33333333, 0.33333333, 0.,         0.,         0.,         0.66666667, 0.66666667,
      0.66666667, 0.33333333, 0.33333333, 0.33333333, 0.33333333, 0.,         0.,         0.,         0.66666667,
      0.66666667, 0.66666667, 0.33333333, 0.33333333, 0.33333333, 0.33333333, 0.,         0.,         0.,
      0.,         0.,         0.,         1.,         1.,         1.,         1.,         0.,         0.,
      0.,         0.,         0.,         0.,         1.,         1.,         1.,         1.,         0.,
      0.,         0.,         0.,         0.,         0.,         1.,         1.,         1.,         1.,
      0.,         0.,         0.,         0.,         0.,         0.,         1.,         1.,         1.,
      1.,
    };

    {
        auto r = mtf<double>(data, 4);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            auto diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto r = mtf<float>(data, 4);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            auto diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_resize() {
    std::vector<double> data = {
      0.,         0.1010101,  0.2020202,  0.3030303,  0.4040404,  0.50505051, 0.60606061, 0.70707071, 0.80808081,
      0.90909091, 1.01010101, 1.11111111, 1.21212121, 1.31313131, 1.41414141, 1.51515152, 1.61616162, 1.71717172,
      1.81818182, 1.91919192, 2.02020202, 2.12121212, 2.22222222, 2.32323232, 2.42424242, 2.52525253, 2.62626263,
      2.72727273, 2.82828283, 2.92929293, 3.03030303, 3.13131313, 3.23232323, 3.33333333, 3.43434343, 3.53535354,
      3.63636364, 3.73737374, 3.83838384, 3.93939394, 4.04040404, 4.14141414, 4.24242424, 4.34343434, 4.44444444,
      4.54545455, 4.64646465, 4.74747475, 4.84848485, 4.94949495, 5.05050505, 5.15151515, 5.25252525, 5.35353535,
      5.45454545, 5.55555556, 5.65656566, 5.75757576, 5.85858586, 5.95959596, 6.06060606, 6.16161616, 6.26262626,
      6.36363636, 6.46464646, 6.56565657, 6.66666667, 6.76767677, 6.86868687, 6.96969697, 7.07070707, 7.17171717,
      7.27272727, 7.37373737, 7.47474747, 7.57575758, 7.67676768, 7.77777778, 7.87878788, 7.97979798, 8.08080808,
      8.18181818, 8.28282828, 8.38383838, 8.48484848, 8.58585859, 8.68686869, 8.78787879, 8.88888889, 8.98989899,
      9.09090909, 9.19191919, 9.29292929, 9.39393939, 9.49494949, 9.5959596,  9.6969697,  9.7979798,  9.8989899,
      10.,
    };
    std::vector<int16_t> data_shape = {10, 10};
    std::vector<double>  expected   = {
         1.2962963,
         1.63299663,
         1.96969697,
         4.66329966,
         5.,
         5.33670034,
         8.03030303,
         8.36700337,
         8.7037037,
    };
    std::vector<int16_t> expected_shape = {3, 3};

    {
        auto r = resize<double>(data, data_shape, expected_shape);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto r = resize<float>(data, data_shape, expected_shape);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_psnr() {
    std::vector<uint8_t> original = {
      0,   2,   5,   7,   10,  12,  15,  18,  20,  23,  25,  28,  30,  33,  36,  38,  41,  43,  46,  48,
      51,  54,  56,  59,  61,  64,  66,  69,  72,  74,  77,  79,  82,  85,  87,  90,  92,  95,  97,  100,
      103, 105, 108, 110, 113, 115, 118, 121, 123, 126, 128, 131, 133, 136, 139, 141, 144, 146, 149, 151,
      154, 157, 159, 162, 164, 167, 170, 172, 175, 177, 180, 182, 185, 188, 190, 193, 195, 198, 200, 203,
      206, 208, 211, 213, 216, 218, 221, 224, 226, 229, 231, 234, 236, 239, 242, 244, 247, 249, 252, 255,
    };
    std::vector<uint8_t> compressed = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
      25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
      50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74,
      75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 100,
    };
    double expected = 9.059609569978468;

    {
        auto r    = psnr<double>(original, compressed);
        auto diff = std::abs(r - expected);
        assert(diff < DSP_EPS);
    }

    {
        auto r    = psnr<float>(original, compressed);
        auto diff = std::abs(r - expected);
        assert(diff < 1e-6);
    }
}

void test_integrate_wavelet() {
    std::vector<double> expected_psi = {
      -5.29958527e-16, -1.05981018e-15, -1.28025089e-15, -5.42880983e-16, 2.33023500e-15,  9.25101796e-15,
      2.29968009e-14,  4.70885454e-14,  8.50975089e-14,  1.38883521e-13,  2.05080119e-13,  2.69022645e-13,
      2.95416632e-13,  2.15616472e-13,  -8.72069396e-14, -7.91028836e-13, -2.13762447e-12, -4.40999208e-12,
      -7.86139656e-12, -1.25642984e-11, -1.81416986e-11, -2.33464123e-11, -2.54746561e-11, -1.96516306e-11,
      1.87684328e-12,  5.01578120e-11,  1.39142574e-10,  2.83720241e-10,  4.95126440e-10,  7.72504563e-10,
      1.08948355e-09,  1.37522707e-09,  1.49081050e-09,  1.20431888e-09,  1.71975835e-10,  -2.06208357e-09,
      -6.02855901e-09, -1.22334779e-08, -2.09682507e-08, -3.20036556e-08, -4.41552101e-08, -5.47362737e-08,
      -5.89646759e-08, -4.94611531e-08, -1.60707770e-08, 5.36590965e-08,  1.72921770e-07,  3.52553588e-07,
      5.95996746e-07,  8.92148397e-07,  1.20637002e-06,  1.47064530e-06,  1.57485470e-06,  1.36230654e-06,
      6.33848519e-07,  -8.34226792e-07, -3.25311011e-06, -6.76097354e-06, -1.13377068e-05, -1.66987576e-05,
      -2.21795497e-05, -2.66315880e-05, -2.83619075e-05, -2.51569667e-05, -1.44375009e-05, 6.41139045e-06,
      3.95043693e-05,  8.57120677e-05,  1.43751842e-04,  2.09214799e-04,  2.73699068e-04,  3.24285867e-04,
      3.43644275e-04,  3.11063471e-04,  2.04667153e-04,  4.94665423e-06,  -3.00452147e-04, -7.11032565e-04,
      -1.20751961e-03, -1.74672136e-03, -2.25848283e-03, -2.64617967e-03, -2.79211776e-03, -2.56882682e-03,
      -1.85652457e-03, -5.65997147e-04, 1.33512174e-03,  3.79602394e-03,  6.66087630e-03,  9.65670511e-03,
      1.23962908e-02,  1.44004162e-02,  1.51418152e-02,  1.41101578e-02,  1.08936420e-02,  5.26871848e-03,
      -2.71419945e-03, -1.26637324e-02, -2.38146009e-02, -3.50423518e-02, -4.49348469e-02, -5.19225721e-02,
      -5.44608897e-02, -5.12476618e-02, -4.14507287e-02, -2.49131982e-02, -2.30195463e-03, 2.48326340e-02,
      5.41096816e-02,  8.24940131e-02,  1.06589217e-01,  1.23022555e-01,  1.28879542e-01,  1.22133101e-01,
      1.02006195e-01,  6.92095655e-02,  2.60081888e-02,  -2.39096743e-02, -7.57607791e-02, -1.24164054e-01,
      -1.63751668e-01, -1.89818936e-01, -1.98927685e-01, -1.89379531e-01, -1.61489711e-01, -1.17617235e-01,
      -6.19395671e-02, 4.67066711e-06,  6.19489084e-02,  1.17626576e-01,  1.61499053e-01,  1.89388872e-01,
      1.98937027e-01,  1.89828277e-01,  1.63761009e-01,  1.24173395e-01,  7.57701205e-02,  2.39190156e-02,
      -2.59988474e-02, -6.92002242e-02, -1.01996854e-01, -1.22123760e-01, -1.28870201e-01, -1.23013214e-01,
      -1.06579876e-01, -8.24846718e-02, -5.41003403e-02, -2.48232927e-02, 2.31129596e-03,  2.49225396e-02,
      4.14600700e-02,  5.12570032e-02,  5.44702310e-02,  5.19319134e-02,  4.49441882e-02,  3.50516931e-02,
      2.38239422e-02,  1.26730738e-02,  2.72354079e-03,  -5.25937714e-03, -1.08843006e-02, -1.41008165e-02,
      -1.51324739e-02, -1.43910748e-02, -1.23869494e-02, -9.64736377e-03, -6.65153497e-03, -3.78668261e-03,
      -1.32578041e-03, 5.75338481e-04,  1.86586591e-03,  2.57816815e-03,  2.80145910e-03,  2.65552101e-03,
      2.26782416e-03,  1.75606269e-03,  1.21686095e-03,  7.20373899e-04,  3.09793481e-04,  4.39467998e-06,
      -1.95325819e-04, -3.01722137e-04, -3.34302941e-04, -3.14944532e-04, -2.64357734e-04, -1.99873464e-04,
      -1.34410507e-04, -7.63707334e-05, -3.01630351e-05, 2.92994376e-06,  2.37788351e-05,  3.44983009e-05,
      3.77032417e-05,  3.59729222e-05,  3.15208840e-05,  2.60400918e-05,  2.06790410e-05,  1.61023078e-05,
      1.25944443e-05,  1.01755610e-05,  8.70748569e-06,  7.97902768e-06,  7.76647952e-06,  7.87068891e-06,
      8.13496419e-06,  8.44918582e-06,  8.74533747e-06,  8.98878063e-06,  9.16841244e-06,  9.28767512e-06,
      9.35740499e-06,  9.39079537e-06,  9.40029889e-06,  9.39607049e-06,  9.38548942e-06,  9.37333787e-06,
      9.36230246e-06,  9.35356769e-06,  9.34736277e-06,  9.34339630e-06,  9.34116224e-06,  9.34012989e-06,
      9.33984340e-06,  9.33995899e-06,  9.34024473e-06,  9.34056171e-06,  9.34083909e-06,  9.34105049e-06,
      9.34119507e-06,  9.34128406e-06,  9.34133234e-06,  9.34135386e-06,  9.34135969e-06,  9.34135756e-06,
      9.34135236e-06,  9.34134678e-06,  9.34134207e-06,  9.34133862e-06,  9.34133635e-06,  9.34133500e-06,
      9.34133430e-06,  9.34133400e-06,  9.34133392e-06,  9.34133394e-06,  9.34133401e-06,  9.34133407e-06,
      9.34133413e-06,  9.34133417e-06,  9.34133419e-06,  9.34133420e-06,  9.34133421e-06,  9.34133421e-06,
      9.34133421e-06,  9.34133421e-06,  9.34133421e-06,  9.34133421e-06,
    };
    std::vector<double> expected_x = {
      -8.,         -7.9372549,  -7.8745098,  -7.81176471, -7.74901961, -7.68627451, -7.62352941, -7.56078431,
      -7.49803922, -7.43529412, -7.37254902, -7.30980392, -7.24705882, -7.18431373, -7.12156863, -7.05882353,
      -6.99607843, -6.93333333, -6.87058824, -6.80784314, -6.74509804, -6.68235294, -6.61960784, -6.55686275,
      -6.49411765, -6.43137255, -6.36862745, -6.30588235, -6.24313725, -6.18039216, -6.11764706, -6.05490196,
      -5.99215686, -5.92941176, -5.86666667, -5.80392157, -5.74117647, -5.67843137, -5.61568627, -5.55294118,
      -5.49019608, -5.42745098, -5.36470588, -5.30196078, -5.23921569, -5.17647059, -5.11372549, -5.05098039,
      -4.98823529, -4.9254902,  -4.8627451,  -4.8,        -4.7372549,  -4.6745098,  -4.61176471, -4.54901961,
      -4.48627451, -4.42352941, -4.36078431, -4.29803922, -4.23529412, -4.17254902, -4.10980392, -4.04705882,
      -3.98431373, -3.92156863, -3.85882353, -3.79607843, -3.73333333, -3.67058824, -3.60784314, -3.54509804,
      -3.48235294, -3.41960784, -3.35686275, -3.29411765, -3.23137255, -3.16862745, -3.10588235, -3.04313725,
      -2.98039216, -2.91764706, -2.85490196, -2.79215686, -2.72941176, -2.66666667, -2.60392157, -2.54117647,
      -2.47843137, -2.41568627, -2.35294118, -2.29019608, -2.22745098, -2.16470588, -2.10196078, -2.03921569,
      -1.97647059, -1.91372549, -1.85098039, -1.78823529, -1.7254902,  -1.6627451,  -1.6,        -1.5372549,
      -1.4745098,  -1.41176471, -1.34901961, -1.28627451, -1.22352941, -1.16078431, -1.09803922, -1.03529412,
      -0.97254902, -0.90980392, -0.84705882, -0.78431373, -0.72156863, -0.65882353, -0.59607843, -0.53333333,
      -0.47058824, -0.40784314, -0.34509804, -0.28235294, -0.21960784, -0.15686275, -0.09411765, -0.03137255,
      0.03137255,  0.09411765,  0.15686275,  0.21960784,  0.28235294,  0.34509804,  0.40784314,  0.47058824,
      0.53333333,  0.59607843,  0.65882353,  0.72156863,  0.78431373,  0.84705882,  0.90980392,  0.97254902,
      1.03529412,  1.09803922,  1.16078431,  1.22352941,  1.28627451,  1.34901961,  1.41176471,  1.4745098,
      1.5372549,   1.6,         1.6627451,   1.7254902,   1.78823529,  1.85098039,  1.91372549,  1.97647059,
      2.03921569,  2.10196078,  2.16470588,  2.22745098,  2.29019608,  2.35294118,  2.41568627,  2.47843137,
      2.54117647,  2.60392157,  2.66666667,  2.72941176,  2.79215686,  2.85490196,  2.91764706,  2.98039216,
      3.04313725,  3.10588235,  3.16862745,  3.23137255,  3.29411765,  3.35686275,  3.41960784,  3.48235294,
      3.54509804,  3.60784314,  3.67058824,  3.73333333,  3.79607843,  3.85882353,  3.92156863,  3.98431373,
      4.04705882,  4.10980392,  4.17254902,  4.23529412,  4.29803922,  4.36078431,  4.42352941,  4.48627451,
      4.54901961,  4.61176471,  4.6745098,   4.7372549,   4.8,         4.8627451,   4.9254902,   4.98823529,
      5.05098039,  5.11372549,  5.17647059,  5.23921569,  5.30196078,  5.36470588,  5.42745098,  5.49019608,
      5.55294118,  5.61568627,  5.67843137,  5.74117647,  5.80392157,  5.86666667,  5.92941176,  5.99215686,
      6.05490196,  6.11764706,  6.18039216,  6.24313725,  6.30588235,  6.36862745,  6.43137255,  6.49411765,
      6.55686275,  6.61960784,  6.68235294,  6.74509804,  6.80784314,  6.87058824,  6.93333333,  6.99607843,
      7.05882353,  7.12156863,  7.18431373,  7.24705882,  7.30980392,  7.37254902,  7.43529412,  7.49803922,
      7.56078431,  7.62352941,  7.68627451,  7.74901961,  7.81176471,  7.8745098,   7.9372549,   8.,
    };

    {
        auto [psi, x] = integrate_wavelet<double>(cwt_wavelet_t::MORLET, 8);
        assert(psi.size() == expected_psi.size());
        assert(x.size() == expected_x.size());
        for (size_t i = 0; i < psi.size(); ++i) {
            auto diff = std::abs(psi[i] - expected_psi[i]);
            assert(diff < 1e-9);
        }
        for (size_t i = 0; i < x.size(); ++i) {
            auto diff = std::abs(x[i] - expected_x[i]);
            assert(diff < 1e-8);
        }
    }

    {
        auto [psi, x] = integrate_wavelet<float>(cwt_wavelet_t::MORLET, 8);
        assert(psi.size() == expected_psi.size());
        assert(x.size() == expected_x.size());
        for (size_t i = 0; i < psi.size(); ++i) {
            auto diff = std::abs(psi[i] - expected_psi[i]);
            assert(diff < 1e-6);
        }
        for (size_t i = 0; i < x.size(); ++i) {
            auto diff = std::abs(x[i] - expected_x[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_cwt() {
    std::vector<double> data = {
      0.00000000e+00,  1.95090322e-01,  3.82683432e-01,  5.55570233e-01,  7.07106781e-01,  8.31469612e-01,
      9.23879533e-01,  9.80785280e-01,  1.00000000e+00,  9.80785280e-01,  9.23879533e-01,  8.31469612e-01,
      7.07106781e-01,  5.55570233e-01,  3.82683432e-01,  1.95090322e-01,  1.22464680e-16,  -1.95090322e-01,
      -3.82683432e-01, -5.55570233e-01, -7.07106781e-01, -8.31469612e-01, -9.23879533e-01, -9.80785280e-01,
      -1.00000000e+00, -9.80785280e-01, -9.23879533e-01, -8.31469612e-01, -7.07106781e-01, -5.55570233e-01,
      -3.82683432e-01, -1.95090322e-01, -2.44929360e-16, 1.95090322e-01,  3.82683432e-01,  5.55570233e-01,
      7.07106781e-01,  8.31469612e-01,  9.23879533e-01,  9.80785280e-01,  1.00000000e+00,  9.80785280e-01,
      9.23879533e-01,  8.31469612e-01,  7.07106781e-01,  5.55570233e-01,  3.82683432e-01,  1.95090322e-01,
      3.67394040e-16,  -1.95090322e-01, -3.82683432e-01, -5.55570233e-01, -7.07106781e-01, -8.31469612e-01,
      -9.23879533e-01, -9.80785280e-01, -1.00000000e+00, -9.80785280e-01, -9.23879533e-01, -8.31469612e-01,
      -7.07106781e-01, -5.55570233e-01, -3.82683432e-01, -1.95090322e-01,
    };
    std::vector<int> scales = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    };
    std::vector<double> expected = {
      2.454313169117099472e-02,  2.359403512589557453e-02,  -2.282946232514224127e-03, -4.932723973796439287e-03,
      -6.155080483483177378e-03, -7.491559109661465737e-03, -8.544802459484891755e-03, -9.269594806516439495e-03,
      -9.638161545434767788e-03, -9.636339141534932054e-03, -9.264197628547209129e-03, -8.536038196116320365e-03,
      -7.479843602874045733e-03, -6.136202814717869880e-03, -4.556751193614309858e-03, -2.802186179595780868e-03,
      -9.399347221793891149e-04, 9.584378994889619641e-04,  2.819978290178062896e-03,  4.573148496637528815e-03,
      6.150575171022583387e-03,  7.491638690867576805e-03,  8.544802737181788482e-03,  9.269594806286666697e-03,
      9.638161545434755645e-03,  9.636339141534958075e-03,  9.264197628547183108e-03,  8.536038196116306487e-03,
      7.479843602874080427e-03,  6.136202814717814369e-03,  4.556751193614344553e-03,  2.802186179595843318e-03,
      9.399347221793474816e-04,  -9.584378994889203307e-04, -2.819978290178111469e-03, -4.573148496637577387e-03,
      -6.150575171022520937e-03, -7.491638690867576805e-03, -8.544802737181802360e-03, -9.269594806286664962e-03,
      -9.638161545434729624e-03, -9.636339141534973687e-03, -9.264197628547209129e-03, -8.536038196116209342e-03,
      -7.479843602874163694e-03, -6.136202814717904575e-03, -4.556751193614365369e-03, -2.802186179595711479e-03,
      -9.399347221791531926e-04, 9.584378994887121639e-04,  2.819978290177986568e-03,  4.573148496637605143e-03,
      6.150575171022611143e-03,  7.491638690867656603e-03,  8.544802737181680929e-03,  9.269594806286692718e-03,
      9.638161545434771257e-03,  9.638161545434719216e-03,  9.267772402616603761e-03,  8.541227685645001039e-03,
      7.486369342435770374e-03,  6.148475162715355546e-03,  4.924956938259436590e-03,  2.274315965027896846e-03,
      -1.265071200573787691e-02, -1.264379477823907201e-02, 2.229987443386890686e-02,  -1.203536355112844528e-02,
      2.002424401495061200e-03,  7.769007428945010887e-04,  -3.698576744507650594e-04, 1.395738039335024650e-04,
      5.054296281022933499e-05,  5.676248459041967445e-05,  5.467920871797625465e-05,  5.002424285227640929e-05,
      4.356873494114386571e-05,  3.542881756900748271e-05,  2.592778763435012361e-05,  1.543036369357815251e-05,
      4.339959188153085672e-06,  -6.917227498647396637e-06, -1.790858901193043520e-05, -2.821173349251066809e-05,
      -3.743071687647421775e-05, -4.521125880236994728e-05, -5.125435740702745683e-05, -5.532777980035112343e-05,
      -5.727498664432768816e-05, -5.702114787160099949e-05, -5.457601836383291585e-05, -5.003356307713504886e-05,
      -4.356834602077051512e-05, -3.542882185950974879e-05, -2.592778794351669873e-05, -1.543036367734306862e-05,
      -4.339959187936967757e-06, 6.917227498451134868e-06,  1.790858901200894160e-05,  2.821173349258917449e-05,
      3.743071687633683578e-05,  4.521125880244845029e-05,  5.125435740698820194e-05,  5.532777980040509637e-05,
      5.727498664426875500e-05,  5.702114787157161761e-05,  5.457601836404389481e-05,  5.003356307680140597e-05,
      4.356834602104527906e-05,  3.542882185939199766e-05,  2.592778794351669873e-05,  1.543036367753932955e-05,
      4.339959187426687158e-06,  -4.339959186771358095e-06, -1.285309538210851445e-05, -2.087229398737149122e-05,
      -2.808937837202118244e-05, -3.422740072761413962e-05, -3.903999585587005411e-05, -4.247416739209043023e-05,
      -4.380568182770860317e-05, -3.733232127481992692e-05, -1.266170011713332779e-04, 3.820627157767987961e-04,
      -7.659164958996960391e-04, -1.993083067281832110e-03, 1.204270299032400397e-02,  -2.229481894022329602e-02,
      -3.607096730215779423e-02, -3.629224309812160293e-02, 3.168976222036676976e-02,  2.751497678801136104e-02,
      -1.519271630357230639e-02, -1.342614220472376774e-02, 2.622009142797590391e-03,  9.921002994632626640e-04,
      -3.208008867951361864e-03, -2.853036639469433769e-03, -2.144113360736990201e-03, -2.040652814873087414e-03,
      -1.849536962772492304e-03, -1.516076976681579678e-03, -1.128528546926589474e-03, -7.018628622137755655e-04,
      -2.478733021979859447e-04, 2.157990119431144917e-04,  6.711621452649391790e-04,  1.100728812225780593e-03,
      1.487995519997881546e-03,  1.818079451583428860e-03,  2.078295602262028412e-03,  2.258644017997822606e-03,
      2.352194010852411577e-03,  2.355350506998539917e-03,  2.267992204056932300e-03,  2.093476232617419615e-03,
      1.838509143593533221e-03,  1.512889179229795185e-03,  1.129129732145786689e-03,  7.019784626782024119e-04,
      2.478505545645440060e-04,  -2.158021113645307342e-04, -6.711616231774029983e-04, -1.100728770203930401e-03,
      -1.487995527886879594e-03, -1.818079451910086181e-03, -2.078295602187677296e-03, -2.258644017995096055e-03,
      -2.352194010852523033e-03, -2.352194010852083714e-03, -2.261800514143732371e-03, -2.084487292177605346e-03,
      -1.827068392097125002e-03, -1.499436271166396887e-03, -1.114181712398296933e-03, -6.861102070330996235e-04,
      -2.316677896754199331e-04, 2.316936367316512022e-04,  6.859940844825126657e-04,  1.113580485158469687e-03,
      1.502624076508280544e-03,  1.838096211603496444e-03,  2.031663874359721303e-03,  2.137921670821639215e-03,
      2.849880143323007056e-03,  3.316708515876671051e-03,  -8.844214219017146631e-04, -2.519489074478189865e-03,
      1.351956367506934945e-02,  1.527344904123047013e-02,  -2.745003529687339155e-02, -3.164310764083369532e-02,
      -6.659870122702231388e-02, -6.656835966704990892e-02, 9.545291485342680127e-03,  6.048786825502398823e-02,
      3.407246429047183778e-02,  -1.538208867778286318e-02, -2.657889230578264206e-02, -6.665210100306503960e-03,
      7.788507857762231104e-03,  6.182266701256450975e-03,  4.476294135715959757e-04,  -1.259781498183456475e-03,
      -3.367956989003539079e-04, 3.392543524743939524e-04,  3.069392138011696903e-04,  1.193883581083366070e-04,
      1.823685013955123752e-05,  -3.715162974588588803e-05, -9.332689782372369930e-05, -1.523581947224840211e-04,
      -2.066283680674271643e-04, -2.525292568548249874e-04, -2.885502357603530197e-04, -3.134854048254965414e-04,
      -3.263860401349423144e-04, -3.267455741675848722e-04, -3.145480363258620210e-04, -2.902624550663587344e-04,
      -2.548222505069472517e-04, -2.095893748864908167e-04, -1.563020976985138658e-04, -9.700821846398734183e-05,
      -3.398636777334883979e-05, 3.398636777127909785e-05,  1.006530262615398449e-04,  1.634516453760165213e-04,
      2.199689093363905480e-04,  2.680328920410163284e-04,  3.057965261391226101e-04,  3.318085712976736159e-04,
      3.450692603593579193e-04,  3.450687085621907159e-04,  3.318090910177730920e-04,  3.058107707334839801e-04,
      2.680633279302185954e-04,  2.198390096050234759e-04,  1.627377291757725348e-04,  1.004764455042965986e-04,
      4.079643754699435743e-05,  -1.823685013690535053e-05, -1.230331659051353728e-04, -3.140887614787781640e-04,
      -3.496338869249508187e-04, 3.235850573659662926e-04,  1.244247427111152193e-03,  -4.648899485431429168e-04,
      -6.200590387448431592e-03, -7.807190526189783895e-03, 6.646886414114018538e-03,  2.656163177080954341e-02,
      1.536655460670778919e-02,  -3.408567493200916065e-02, -6.049824778947718307e-02, -9.552441033023217898e-03,
      -1.027400448781746467e-01, -1.020775794477147280e-01, -2.371482464262277684e-02, 5.970114082060457711e-02,
      8.530184246913172230e-02,  4.451545612488789355e-02,  -1.098402395793143110e-02, -3.346113002228146699e-02,
      -1.929856055005904625e-02, 4.840936138166859574e-03,  1.736493367496132201e-02,  1.520316040133374604e-02,
      7.858526318966373506e-03,  2.752514291302034753e-03,  1.324268337265132461e-03,  1.288938416595580404e-03,
      8.026536046535756499e-04,  -3.913231469583234473e-04, -1.824707250624079281e-03, -3.161297188532255758e-03,
      -4.291513647856075482e-03, -5.221171877834666214e-03, -5.950889137149362185e-03, -6.458566109831104934e-03,
      -6.721318404129386637e-03, -6.721899914273374819e-03, -6.463755294745293992e-03, -5.957034218292509009e-03,
      -5.221364488318371889e-03, -4.285056248180553126e-03, -3.184083504158541616e-03, -1.960749213261625719e-03,
      -6.620640512816549977e-04, 6.620640512786216169e-04,  1.960749213258708781e-03,  3.184083504155942566e-03,
      4.285056248178240740e-03,  5.221364488316645840e-03,  5.957034218291040566e-03,  6.463755294744496886e-03,
      6.721899914272868280e-03,  6.721318404129624294e-03,  6.462641128834200124e-03,  5.958882574460839555e-03,
      5.232776550141118171e-03,  4.306283594108459636e-03,  3.178664807978706392e-03,  1.844005115389584729e-03,
      4.118096509227534832e-04,  -7.817657463483282127e-04, -1.268451912630510310e-03, -1.304970472499049133e-03,
      -2.735146671855913716e-03, -7.843756372715284322e-03, -1.519155572902862635e-02, -1.735694023765070940e-02,
      -4.836861119164263118e-03, 1.929856055005886237e-02,  3.345705500327839782e-02,  1.097603052062027032e-02,
      -4.452706079719400550e-02, -8.531661241538388007e-02, -5.971850844005084646e-02, 2.369552677785793232e-02,
      -1.395415403584168990e-01, -1.393397775353128198e-01, -6.163890576651957082e-02, 4.076797000594761106e-02,
      1.067971771869018205e-01,  1.032103522750511954e-01,  5.044109906914216734e-02,  -1.204885717068828431e-02,
      -4.531118079350679451e-02, -4.352639625307638815e-02, -1.915589951770738791e-02, 4.505254354996300788e-03,
      1.578938013809653984e-02,  1.381256957933937705e-02,  6.332696319608381932e-03,  8.204392675391875522e-07,
      -2.562055653508936784e-03, -2.523529904023148057e-03, -1.585248894462842157e-03, -1.043317925058132294e-03,
      -1.064280690183520492e-03, -1.414136555961411539e-03, -1.764144147435048370e-03, -1.978801389005055249e-03,
      -2.045657915223446279e-03, -2.004870015918406526e-03, -1.886125561034268303e-03, -1.701711247639133874e-03,
      -1.456601769642694270e-03, -1.156828347163693128e-03, -8.124786336163320865e-04, -4.365227267599352786e-04,
      -4.360140664148709613e-05, 3.510504392809871772e-04,  7.321566654944293949e-04,  1.084936431974025881e-03,
      1.395639604749476792e-03,  1.652589103754445671e-03,  1.847342974937961606e-03,  1.975805135910922456e-03,
      2.035151102603862706e-03,  2.015376828537608894e-03,  1.889121814128260149e-03,  1.618512420136607264e-03,
      1.218149221849953306e-03,  8.254694325980553669e-04,  7.708601267002939761e-04,  1.289614955727846598e-03,
      2.216080871383358745e-03,  2.408331137191330445e-03,  -1.441734489729653223e-04, -6.460168846932139511e-03,
      -1.391926292654433253e-02, -1.587119413968350601e-02, -4.559044944765592959e-03, 1.913219948195414924e-02,
      4.353369755042247463e-02,  4.534920283918981304e-02,  1.211613879881512368e-02,  -5.034714345380855716e-02,
      -1.030933333341170766e-01, -1.066615918926418632e-01, -4.061902882518294428e-02, 6.179547910773983826e-02,
      -1.815052161852935275e-01, -1.821481758330381062e-01, -1.091741299631164008e-01, 2.366525245660373519e-03,
      1.014616104424299198e-01,  1.411753264481249992e-01,  1.181055199122307692e-01,  5.424281324228594908e-02,
      -1.479745551564122692e-02, -5.945496271272762007e-02, -6.858650267249814803e-02, -4.965026182220751844e-02,
      -1.975358467618883174e-02, 4.066104104957078691e-03,  1.503867390186656820e-02,  1.464669785040796472e-02,
      8.640173964093667003e-03,  2.620835444909080676e-03,  -4.336565489336144754e-04, -3.082151496866178397e-04,
      1.784829829726170946e-03,  4.099607682182385138e-03,  5.789454815958590059e-03,  6.661720139325685852e-03,
      6.891099860846695499e-03,  6.724629233720386123e-03,  6.326849556299560078e-03,  5.760067101911052305e-03,
      5.033721823078733323e-03,  4.136113102839742220e-03,  3.080845746085694985e-03,  1.901033953848296244e-03,
      6.425832657729487439e-04,  -6.425832657773089714e-04, -1.901033953852385638e-03, -3.080845746089403391e-03,
      -4.136113102843052072e-03, -5.033721823081367501e-03, -5.760067101913291833e-03, -6.326849556300770915e-03,
      -6.724629233720991542e-03, -6.891099860846162939e-03, -6.661720139324553078e-03, -5.789454815956409511e-03,
      -4.099607682179305136e-03, -1.784829829722935253e-03, 3.082151496901932132e-04,  4.336565489375477983e-04,
      -2.620835444904922543e-03, -8.640173964089305908e-03, -1.464669785040379792e-02, -1.503867390186293396e-02,
      -4.066104104953994353e-03, 1.975358467619085096e-02,  4.965026182220957235e-02,  6.858650267249902233e-02,
      5.945496271272870947e-02,  1.479745551564152009e-02,  -5.423799161477936559e-02, -1.180960619496590897e-01,
      -1.411615956146855533e-01, -1.014441344063511663e-01, -2.345975601208108614e-03, 1.091969635046304565e-01,
      -2.365240235413067771e-01, -2.355592940931444390e-01, -1.610585082125363665e-01, -4.087175029203778814e-02,
      8.145402185364163106e-02,  1.650027131652464230e-01,  1.873818470793304491e-01,  1.512752522819552081e-01,
      7.977790874706068569e-02,  4.061544569396343708e-03,  -5.006756889015692019e-02, -7.074729641431599936e-02,
      -6.112633566471776386e-02, -3.418585706394095675e-02, -5.052389109698552337e-03, 1.531220053474436314e-02,
      2.283076260972450797e-02,  1.944061240012041891e-02,  1.018179439739504318e-02,  1.508306878792814128e-04,
      -7.389101572465242226e-03, -1.142219919498642378e-02, -1.254920209674521663e-02, -1.200961113742784157e-02,
      -1.090608688039096386e-02, -9.853630348702884650e-03, -8.995830492983246038e-03, -8.202948041475350768e-03,
      -7.279807630747780513e-03, -6.093239627818674946e-03, -4.609220467180659420e-03, -2.873576232429955139e-03,
      -9.762155893717791404e-04, 9.762155893671571865e-04,  2.873576232425568457e-03,  4.609220467176705117e-03,
      6.093239627814857687e-03,  7.279807630744762961e-03,  8.202948041472981136e-03,  8.995830492981663970e-03,
      9.853630348702248007e-03,  1.090608688039142876e-02,  1.200961113742922588e-02,  1.254920209674750993e-02,
      1.142219919498954454e-02,  7.389101572469137548e-03,  -1.508306878752482350e-04, -1.018179439739074453e-02,
      -1.944061240011566577e-02, -2.283076260972039667e-02, -1.531220053473986674e-02, 5.052389109702236890e-03,
      3.418585706394413476e-02,  6.112633566472081698e-02,  7.074729641431797000e-02,  5.006756889015909900e-02,
      -4.061544569395362722e-03, -7.977790874706040813e-02, -1.512752522819557355e-01, -1.873818470793316981e-01,
      -1.650027131652484769e-01, -8.145402185364414294e-02, 4.087175029203543586e-02,  1.610585082125340906e-01,
      -2.988488148191981164e-01, -2.982954181119848447e-01, -2.254551792204678995e-01, -1.013816473458237777e-01,
      3.891110419464099968e-02,  1.579222474663561637e-01,  2.199838154893226649e-01,  2.213886150937377784e-01,
      1.714526489232104933e-01,  9.227917576780424180e-02,  1.013155932198898934e-02,  -5.311535766367238287e-02,
      -8.559063713121095596e-02, -8.677677154039985241e-02, -6.500791030562108608e-02, -3.278943813332608342e-02,
      -3.039716358485666098e-03, 1.683113472469140171e-02,  2.469598690162024338e-02,  2.252179601088498059e-02,
      1.442640255151918585e-02,  4.717417626965989333e-03,  -3.444432368172545278e-03, -8.541434624942129822e-03,
      -1.049911766083728125e-02, -1.011762355207085004e-02, -8.414324993274673384e-03, -6.430113584907640005e-03,
      -4.652266752087600050e-03, -3.235902728383296956e-03, -2.131108862981468885e-03, -1.216762449100471793e-03,
      -3.979287724663153891e-04, 3.979287724615900024e-04,  1.216762449096006615e-03,  2.131108862977550145e-03,
      3.235902728379427655e-03,  4.652266752084258973e-03,  6.430113584905454253e-03,  8.414324993273221420e-03,
      1.011762355207026023e-02,  1.068739097380968367e-02,  8.722472700082840869e-03,  3.611278013781226912e-03,
      -4.571176195481656615e-03, -1.430638531035833758e-02, -2.243261515531968023e-02, -2.464106960191534551e-02,
      -1.681259142187765260e-02, 3.021173055680736325e-03,  3.273452083363002224e-02,  6.491872945006334217e-02,
      8.665675429924633855e-02,  8.544439569973327764e-02,  5.294851201806866081e-02,  -1.031259739712622747e-02,
      -9.246744908077492164e-02, -1.716409222361822973e-01, -2.215696531688775561e-01, -2.201506611349297871e-01,
      -1.580684888978388103e-01, -3.903112143580012017e-02, 1.012924664902601357e-01,  2.254002619207653157e-01,
      -3.693563438822927147e-01, -3.656083216777074596e-01, -2.920821380263680656e-01, -1.587222920996241016e-01,
      -1.750793395080426544e-03, 1.436334218636089177e-01,  2.503377182110917976e-01,  3.020633185309265500e-01,
      2.884219252170811543e-01,  2.282059358624832501e-01,  1.388499674482791546e-01,  4.614009174563320909e-02,
      -3.178118836058548002e-02, -7.893916575365585164e-02, -9.684140881387429123e-02, -8.724668948103825317e-02,
      -6.326979472395267978e-02, -3.483070877070088001e-02, -1.153147039874018176e-02, 2.525868001433688795e-03,
      6.244014551281563789e-03,  1.438853118607946301e-04,  -1.080122500803966726e-02, -2.298581477872103235e-02,
      -3.312531715241463781e-02, -3.955960911624342019e-02, -4.133796982418819904e-02, -3.936307386041185824e-02,
      -3.437066648418214687e-02, -2.764042067919092113e-02, -1.998012671849465696e-02, -1.202526547543411942e-02,
      -3.986408016948979859e-03, 3.986408016944547640e-03,  1.202526547542988496e-02,  1.998012671849044505e-02,
      2.764042067918760781e-02,  3.437066648417844150e-02,  3.936307386041000556e-02,  4.133796982418594390e-02,
      3.955960911624303161e-02,  3.312531715241464475e-02,  2.298581477872336729e-02,  1.080122500804128055e-02,
      -1.438853118577226517e-04, -6.244014551277888778e-03, -2.525868001429618700e-03, 1.153147039874432948e-02,
      3.483070877070516130e-02,  6.326979472395685700e-02,  8.724668948104231936e-02,  9.684140881387853783e-02,
      7.893916575365966803e-02,  3.178118836058890784e-02,  -4.614009174563048904e-02, -1.388499674482763513e-01,
      -2.282059358624821954e-01, -2.884219252170801551e-01, -3.020633185309265500e-01, -2.503377182110919086e-01,
      -1.436334218636099724e-01, 1.750793395077968875e-03,  1.587222920996226860e-01,  2.920821380263662892e-01,
      -4.633194390917706462e-01, -4.567443101594595989e-01, -3.741686903554667376e-01, -2.317830727316253503e-01,
      -5.973193040189563735e-02, 1.150666122387526136e-01,  2.608052696081371025e-01,  3.539750697408882152e-01,
      3.838613977674615874e-01,  3.604266307204532227e-01,  2.884730618674923863e-01,  1.886979215311010705e-01,
      8.374967409346133207e-02,  -7.047121429347205299e-03, -7.131359636743996033e-02, -1.094244639989458701e-01,
      -1.189285939095734296e-01, -1.075013882749066702e-01, -8.521328247740257711e-02, -6.161184539566526286e-02,
      -4.202356940480433956e-02, -3.129018862758727731e-02, -2.994199506708371261e-02, -3.586502754196056741e-02,
      -4.553972658349341590e-02, -5.516821524971915469e-02, -6.222025855238102338e-02, -6.467845291579668587e-02,
      -6.175094779793611349e-02, -5.366091580904532726e-02, -4.129284921389157759e-02, -2.604669308816242451e-02,
      -8.892244743392422809e-03, 8.892244743388325393e-03,  2.604669308815846934e-02,  4.129284921388794161e-02,
      5.366091580904260722e-02,  6.175094779793270650e-02,  6.467845291579447931e-02,  6.222025855237950376e-02,
      5.516821524971880081e-02,  4.544815462549452872e-02,  3.577697464295447743e-02,  2.986084505059847030e-02,
      3.121906004324610101e-02,  4.196519568421525764e-02,  6.156846980817347326e-02,  8.518657192251075094e-02,
      1.074923692242585604e-01,  1.189376129602295468e-01,  1.094511745538452457e-01,  7.135697195493934109e-02,
      7.105495149942821914e-03,  -8.367854550911388301e-02, -1.886167715146113977e-01, -2.883850089684822371e-01,
      -3.603350587624524759e-01, -3.906064864282213778e-01, -3.604609484271053543e-01, -2.667826896392219149e-01,
      -1.203058647150867383e-01, 5.543218701477095611e-02,  2.285880751607534123e-01,  3.722012205657210804e-01,
      -5.641828893201417117e-01, -5.513458363914138793e-01, -4.658455945520287855e-01, -3.118909757770713109e-01,
      -1.206289362106392482e-01, 8.153657750541468130e-02,  2.629999032742941778e-01,  4.026943810076059616e-01,
      4.859316531527035909e-01,  4.988287777058058592e-01,  4.552427452772458416e-01,  3.675320525415303741e-01,
      2.515692407527901908e-01,  1.290676740865793204e-01,  1.588120296726105996e-02,  -7.150174301279860201e-02,
      -1.315738106652309392e-01, -1.640524154118231659e-01, -1.700277289840377848e-01, -1.601334045240648540e-01,
      -1.416498469972885810e-01, -1.223038767001445593e-01, -1.061688013720447077e-01, -9.539720374931401348e-02,
      -9.175635610089735716e-02, -9.161116659209513069e-02, -9.244975329657709184e-02, -9.174006804223656941e-02,
      -8.681000847800021680e-02, -7.641831104991132773e-02, -5.973646923589918944e-02, -3.813552694092765122e-02,
      -1.320337551186115572e-02, 1.298597621600632197e-02,  3.792648217809353045e-02,  5.954381247908784730e-02,
      7.624944599024428837e-02,  8.667142450501917617e-02,  9.163709086030440154e-02,  9.238634026104490526e-02,
      9.158975463035841369e-02,  9.177776806263412968e-02,  9.546061678484531188e-02,  1.062717785539768339e-01,
      1.224424606731253917e-01,  1.418187120569551485e-01,  1.603260612808766750e-01,  1.702367737468703512e-01,
      1.642698147076792625e-01,  1.317912099610854815e-01,  7.171078777563320850e-02,  -1.568854621044885045e-02,
      -1.288988090269115039e-01, -2.514306567798085812e-01, -3.674290753595972348e-01, -4.551793322417134191e-01,
      -4.988073657440669773e-01, -4.859530651144392532e-01, -4.027577940431353865e-01, -2.631028804562233758e-01,
      -8.167516147839264107e-02, 1.204600711509733746e-01,  3.116983190202621823e-01,  4.656365497891971073e-01,
      -6.830475241573693568e-01, -6.595694115391986800e-01, -5.571675093823884595e-01, -3.881338441704972886e-01,
      -1.741827849579860865e-01, 5.707410686513980647e-02,  2.765428669765783165e-01,  4.581320237896309711e-01,
      5.910971727718092872e-01,  6.568288789474163725e-01,  6.532726483473580981e-01,  5.876504015839176809e-01,
      4.743813182285904761e-01,  3.320803293252004851e-01,  1.802584398423742462e-01,  3.633844374806621119e-02,
      -8.653610579026237148e-02, -1.806325008263444665e-01, -2.436703723675167910e-01, -2.779502180621493679e-01,
      -2.908363460343260098e-01, -2.857091635956021114e-01, -2.695088719995631377e-01, -2.481407066848297649e-01,
      -2.255865536176020758e-01, -2.038130085511145606e-01, -1.830007494310703364e-01, -1.621197546759329811e-01,
      -1.396365974256126019e-01, -1.141804773464206585e-01, -8.503414738702613773e-02, -5.237243351117815832e-02,
      -1.774479540690028848e-02, 1.774479540689708618e-02,  5.237243351117535500e-02,  8.503414738702294584e-02,
      1.141804773464198675e-01,  1.396365974256101872e-01,  1.621197546759318708e-01,  1.830007494310687266e-01,
      2.038130085511142553e-01,  2.255865536176019648e-01,  2.481407066848311527e-01,  2.695088719995640814e-01,
      2.857091635956034992e-01,  2.908363460343286189e-01,  2.779502180621517549e-01,  2.436703723675195943e-01,
      1.806325008263476861e-01,  8.653610579026636829e-02,  -3.633844374806340788e-02, -1.802584398423700551e-01,
      -3.320803293251983201e-01, -4.743813182285875896e-01, -5.876504015839149053e-01, -6.532726483473558776e-01,
      -6.568288789474148182e-01, -5.910971727718078439e-01, -4.581320237896293612e-01, -2.765428669765775394e-01,
      -5.707410686513980647e-02, 1.741827849579868914e-01,  3.881338441704964559e-01,  5.571675093823876823e-01,
      -8.300172709497956047e-01, -7.939438725002304631e-01, -6.717587808734182264e-01, -4.851446312337239219e-01,
      -2.451412661146483429e-01, 1.815296525570635894e-02,  2.790551329676959624e-01,  5.134639761767196831e-01,
      6.939434111113134129e-01,  8.034864251464046703e-01,  8.445347898375302753e-01,  8.095970351287634648e-01,
      7.153991769228339459e-01,  5.704981456268068962e-01,  3.958761796560363866e-01,  2.093588811816290141e-01,
      3.017653639784790079e-02,  -1.319128974524745035e-01, -2.626679142276050483e-01, -3.623984405588678848e-01,
      -4.255266653647805652e-01, -4.584642050434857841e-01, -4.632212378149838106e-01, -4.470952966811182483e-01,
      -4.158147035455460605e-01, -3.755343490843560672e-01, -3.285504697692603338e-01, -2.795706968773903900e-01,
      -2.284762405401966068e-01, -1.779929512920287726e-01, -1.267588446326404494e-01, -7.618181083016326016e-02,
      -2.529643637305228551e-02, 2.529643637304937812e-02,  7.618181083016181687e-02,  1.267588446326377571e-01,
      1.779929512920275236e-01,  2.284762405401953578e-01,  2.795706968773891687e-01,  3.285504697692602227e-01,
      3.755343490843549570e-01,  4.158147035455467821e-01,  4.470952966811194695e-01,  4.632212378149838106e-01,
      4.584642050434876714e-01,  4.255266653647820085e-01,  3.623984405588699942e-01,  2.626679142276079904e-01,
      1.319128974524770015e-01,  -3.017653639784457706e-02, -2.093588811816265161e-01, -3.958761796560334445e-01,
      -5.704981456268043427e-01, -7.153991769228320585e-01, -8.095970351287611333e-01, -8.445347898375288320e-01,
      -8.034864251464018947e-01, -6.939434111113129688e-01, -5.134639761767171295e-01, -2.790551329676951298e-01,
      -1.815296525570552974e-02, 2.451412661146487593e-01,  4.851446312337243660e-01,  6.717587808734182264e-01,
      -9.820448284965809238e-01, -9.244718897168707761e-01, -7.848826372182543532e-01, -5.640945571093421984e-01,
      -2.913591171512085465e-01, 1.032816658039428792e-02,  3.154492658814654904e-01,  5.939999893236086459e-01,
      8.249023047805025266e-01,  9.907429709042732124e-01,  1.080400405794431862e+00,  1.080458309598815836e+00,
      1.004769516693765352e+00,  8.633503058966572885e-01,  6.707797417594069112e-01,  4.440295409630686807e-01,
      2.043083220000702915e-01,  -3.108627435839483866e-02, -2.470812154630847579e-01, -4.265091120222441190e-01,
      -5.673168645162489643e-01, -6.660967520594300861e-01, -7.194112055755975677e-01, -7.347908049357794180e-01,
      -7.170178838534548849e-01, -6.718849917093451252e-01, -6.072072069678444706e-01, -5.277114048894110399e-01,
      -4.381970289309064515e-01, -3.431326230180949399e-01, -2.466140395021731435e-01, -1.483703693925004785e-01,
      -4.954110264559263843e-02, 4.954110264559177801e-02,  1.483703693924970368e-01,  2.466140395021722831e-01,
      3.431326230180927750e-01,  4.381970289309064515e-01,  5.277114048894108178e-01,  6.072072069678450257e-01,
      6.718849917093432378e-01,  7.167748240515646740e-01,  7.345570857857650626e-01,  7.191958087733238969e-01,
      6.659079551832143284e-01,  5.671619229241632310e-01,  4.263939800327817831e-01,  2.470103175340358226e-01,
      3.106233505741029521e-02,  -2.042843826990810296e-01, -4.439586430340132783e-01, -6.706646097699402453e-01,
      -8.631953643045704450e-01, -1.004580719817546042e+00, -1.080242912796540500e+00, -1.080166686644415286e+00,
      -9.904999111023801150e-01, -8.246592449786086521e-01, -5.937662701735900717e-01, -3.152338690791900988e-01,
      -1.013936970417511976e-02, 2.915140587432939467e-01,  5.642096890988053115e-01,  7.849535351473063693e-01,
      -1.138056598727003266e+00, -1.051284612317330058e+00, -8.761123073610068701e-01, -6.241055564424700108e-01,
      -3.140532823444575072e-01, 2.979772684560266782e-02,  3.800696389748541293e-01,  7.088759923254004391e-01,
      9.904440314897522413e-01,  1.203449669023304747e+00,  1.332821858870616882e+00,  1.370845732241252835e+00,
      1.317484002118036734e+00,  1.179931091805313415e+00,  9.715014560191166382e-01,  7.100218299106186848e-01,
      4.159392149694847163e-01,  1.103690359255518061e-01,  -1.867078575068772395e-01, -4.579299779770105161e-01,
      -6.896077217518978841e-01, -8.722920066067763045e-01, -1.000868196337813210e+00, -1.074236188031850947e+00,
      -1.094674966104663527e+00, -1.067010535740203325e+00, -9.977100033470953644e-01, -8.940138970389812645e-01,
      -7.631972864046598914e-01, -6.120221733405486475e-01, -4.464131738444345743e-01, -2.713592035154483995e-01,
      -9.101823986574597569e-02, 9.101823986574419934e-02,  2.713592035154466231e-01,  4.464131738444327979e-01,
      6.120221733405513120e-01,  7.631972864046576710e-01,  8.940138970389810424e-01,  9.977100033470958085e-01,
      1.067010535740201993e+00,  1.094674966104664637e+00,  1.074236188031852723e+00,  1.000868196337812766e+00,
      8.722920066067771927e-01,  6.896077217518987723e-01,  4.579299779770131806e-01,  1.867078575068799040e-01,
      -1.103690359255491416e-01, -4.159392149694838281e-01, -7.100218299106169084e-01, -9.715014560191139736e-01,
      -1.179931091805313415e+00, -1.317484002118035180e+00, -1.370845732241251502e+00, -1.332821858870614662e+00,
      -1.203449669023303414e+00, -9.904440314897495767e-01, -7.088759923253995510e-01, -3.800696389748559056e-01,
      -2.979772684559911511e-02, 3.140532823444583954e-01,  6.241055564424708990e-01,  8.761123073610082024e-01,
      -1.300896404167874332e+00, -1.174009519918232547e+00, -9.525645605048190800e-01, -6.500521549246642827e-01,
      -2.951735396766977382e-01, 9.686619715849838219e-02,  4.967427098963173582e-01,  8.741746735303863591e-01,
      1.206246484725670642e+00,  1.465662408757159474e+00,  1.633473309650083971e+00,  1.698281976924917336e+00,
      1.667109123824666250e+00,  1.533161513991615266e+00,  1.307769321661833573e+00,  1.008325922376715367e+00,
      6.590012481693540147e-01,  2.785508390602872142e-01,  -1.085423954027796983e-01, -4.792583631681587297e-01,
      -8.187491556854152863e-01, -1.106331260333969446e+00, -1.329357704675044216e+00, -1.481040417615539395e+00,
      -1.558813701809124597e+00, -1.567365639104713715e+00, -1.506209691355907543e+00, -1.382896401130527364e+00,
      -1.206025457631599185e+00, -9.837873695647965100e-01, -7.275350774437583379e-01, -4.467812709236251845e-01,
      -1.513300586314529905e-01, 1.513300586314539065e-01,  4.467812709236233526e-01,  7.275350774437555623e-01,
      9.837873695648001737e-01,  1.206025457631596964e+00,  1.382896401130526920e+00,  1.506209691355908209e+00,
      1.567365639104714159e+00,  1.558813701809125263e+00,  1.481040417615540283e+00,  1.329357704675044216e+00,
      1.106331260333972111e+00,  8.187491556854161745e-01,  4.792583631681605616e-01,  1.085423954027833621e-01,
      -2.785508390602890461e-01, -6.590012481693521273e-01, -1.008325922376716255e+00, -1.307769321661829798e+00,
      -1.533161513991616154e+00, -1.667109123824663808e+00, -1.698281976924917780e+00, -1.633473309650080862e+00,
      -1.465662408757158586e+00, -1.206246484725668422e+00, -8.741746735303872473e-01, -4.967427098963136944e-01,
      -9.686619715849838219e-02, 2.951735396766959063e-01,  6.500521549246679465e-01,  9.525645605048218556e-01,
      -1.438458323047548282e+00, -1.261265997610114153e+00, -9.847085443110367153e-01, -6.352234475040001715e-01,
      -2.223649867536113822e-01, 2.203825859763558215e-01,  6.704933611011649752e-01,  1.096764214587731212e+00,
      1.470302142462053929e+00,  1.770532871086579751e+00,  1.970838039755434590e+00,  2.056094343484006437e+00,
      2.030828746741466073e+00,  1.885848054649710503e+00,  1.639840060699229385e+00,  1.298862064315583931e+00,
      8.897967059759588349e-01,  4.316324841711943749e-01,  -4.788987668917514540e-02, -5.229925245967469438e-01,
      -9.704705580800382370e-01, -1.364874293650065740e+00, -1.686134790389374638e+00, -1.926806117760024639e+00,
      -2.071244920278791035e+00, -2.121120322063130725e+00, -2.071689727914622914e+00, -1.931193098875919167e+00,
      -1.704667555617551455e+00, -1.406052289867352645e+00, -1.047281833722746969e+00, -6.468046158103399890e-01,
      -2.178161789492095157e-01, 2.178161789492114031e-01,  6.468046158103362142e-01,  1.047281833722748745e+00,
      1.406052289867348870e+00,  1.704667555617557229e+00,  1.931193098875917391e+00,  2.071689727914623358e+00,
      2.121120322063131614e+00,  2.071244920278791479e+00,  1.926806117760026416e+00,  1.686134790389375526e+00,
      1.364874293650066628e+00,  9.704705580800363496e-01,  5.229925245967507186e-01,  4.788987668917514540e-02,
      -4.316324841711868809e-01, -8.900629646486012403e-01, -1.299118090816750559e+00, -1.640076016074067500e+00,
      -1.886054871265491206e+00, -2.030998476751630477e+00, -2.056220464259445269e+00, -1.970915704545477043e+00,
      -1.770559095276904360e+00, -1.470275918271729765e+00, -1.096686549797680321e+00, -6.703672403257265877e-01,
      -2.202128559661906959e-01, 2.225718033693930009e-01,  6.354594028788391746e-01,  9.849645708122052312e-01,
      -1.512385899201345252e+00, -1.273560453454055885e+00, -9.461777674609649358e-01, -5.334619581073852812e-01,
      -6.892962672482678965e-02, 4.228764503851324963e-01,  9.187097679303397602e-01,  1.384411001770297966e+00,
      1.789832883858928358e+00,  2.111798383724375228e+00,  2.330817750775484587e+00,  2.422897463482826197e+00,
      2.389930916167095631e+00,  2.231899993293624096e+00,  1.945276074689217038e+00,  1.554462534875628688e+00,
      1.078941437761621769e+00,  5.375238350559616851e-01,  -3.681622866412235379e-02, -6.150797015946993795e-01,
      -1.167911127943787042e+00, -1.668145798435166638e+00, -2.086894782061714526e+00, -2.409134138318147045e+00,
      -2.620051226071191675e+00, -2.704424253000325518e+00, -2.664358022031214634e+00, -2.502998961247017817e+00,
      -2.224714626271645290e+00, -1.844611362905790530e+00, -1.380828088998091507e+00, -8.535177211536448816e-01,
      -2.883149344054549146e-01, 2.883149344054568575e-01,  8.535177211536468800e-01,  1.380828088998087733e+00,
      1.844611362905793417e+00,  2.224714626271642626e+00,  2.502998961247020038e+00,  2.664358022031214634e+00,
      2.704424253000326850e+00,  2.620051226071193007e+00,  2.409134138318147933e+00,  2.086894782061720299e+00,
      1.668145798435164640e+00,  1.167911127943789040e+00,  6.150797015946954938e-01,  3.681622866412622569e-02,
      -5.375238350559616851e-01, -1.078941437761621769e+00, -1.554462534875632684e+00, -1.945276074689211043e+00,
      -2.231899993293622320e+00, -2.389930916167096075e+00, -2.422897463482827529e+00, -2.330817750775482811e+00,
      -2.111798383724374784e+00, -1.789832883858925472e+00, -1.384411001770297966e+00, -9.187097679303397602e-01,
      -4.228764503851324963e-01, 6.892962672483259057e-02,  5.334619581073822836e-01,  9.461777674609669342e-01,
      -1.549610113096189901e+00, -1.241300686745090642e+00, -8.443824719989027061e-01, -3.764497850523230160e-01,
      1.500238070820625602e-01,  6.960949942193490614e-01,  1.232599690365694611e+00,  1.730000238835760040e+00,
      2.159812549727315290e+00,  2.491368327642728531e+00,  2.706570057583665978e+00,  2.790787685799128948e+00,
      2.735834060522944977e+00,  2.541321428232243385e+00,  2.203017914785318609e+00,  1.747048838654639225e+00,
      1.194470866350926253e+00,  5.708099217666626757e-01,  -9.328617787182014176e-02, -7.675495441188486812e-01,
      -1.415050068917695025e+00, -2.004271925144963884e+00, -2.506199532440170685e+00, -2.896317651296802076e+00,
      -3.149413823755089936e+00, -3.258568969231475432e+00, -3.218318219910480504e+00, -3.027935944900973286e+00,
      -2.697916633952610166e+00, -2.239382875074837909e+00, -1.678135716844817660e+00, -1.039758340517949131e+00,
      -3.519468854228578469e-01, 3.519468854228558485e-01,  1.039758340517949131e+00,  1.678135716844819658e+00,
      2.239382875074840129e+00,  2.697916633952610166e+00,  3.027935944900975951e+00,  3.218318219910480060e+00,
      3.258568969231477208e+00,  3.149413823755091268e+00,  2.896317651296804740e+00,  2.506199532440172462e+00,
      2.004271925144967881e+00,  1.415050068917689030e+00,  7.675495441188486812e-01,  9.328617787182014176e-02,
      -5.708099217666566805e-01, -1.194470866350926253e+00, -1.747048838654639225e+00, -2.203017914785321718e+00,
      -2.541321428232239388e+00, -2.735834060522945421e+00, -2.790787685799130280e+00, -2.706570057583664646e+00,
      -2.491368327642728975e+00, -2.159812549727314401e+00, -1.730000238835760040e+00, -1.232599690365694611e+00,
      -6.960949942193470630e-01, -1.500238070820625602e-01, 3.764497850523270128e-01,  8.443824719989027061e-01,
      -1.491362090046195021e+00, -1.106560124398065170e+00, -6.373137163284032791e-01, -1.040381180833427760e-01,
      4.681463063440879369e-01,  1.050670694444678421e+00,  1.613030226768328745e+00,  2.124465084004373061e+00,
      2.555709591679601989e+00,  2.880703258686545531e+00,  3.077768849214347568e+00,  3.132024464824600063e+00,
      3.035416376486893331e+00,  2.776230213053120899e+00,  2.373754959516900520e+00,  1.844346100356397233e+00,
      1.211230937878224001e+00,  5.033680894525252025e-01,  -2.460069135454019673e-01, -1.001068069388295401e+00,
      -1.725224024046435156e+00, -2.382982730414933137e+00, -2.941756540682623822e+00, -3.373514112467706827e+00,
      -3.656195823412041968e+00, -3.774823655371260855e+00, -3.722253705911304777e+00, -3.499538526034562391e+00,
      -3.115886298296040025e+00, -2.588223466837219267e+00, -1.940385942691142063e+00, -1.201980716250171888e+00,
      -4.069526869461274066e-01, 4.069526869461274066e-01,  1.201980716250169889e+00,  1.940385942691144061e+00,
      2.588223466837221043e+00,  3.115886298296041801e+00,  3.499538526034566388e+00,  3.722253705911304333e+00,
      3.774823655371261300e+00,  3.656195823412045076e+00,  3.373514112467709936e+00,  2.941756540682626042e+00,
      2.382982730414934913e+00,  1.725224024046424942e+00,  1.001068069388301396e+00,  2.460069135454039935e-01,
      -5.033680894525272009e-01, -1.211230937878218006e+00, -1.844346100356399232e+00, -2.373754959516902296e+00,
      -2.776230213053120899e+00, -3.035416376486893331e+00, -3.132024464824599619e+00, -3.077768849214347568e+00,
      -2.880703258686544643e+00, -2.555997183915612592e+00, -2.124741624232038717e+00, -1.613285087701782361e+00,
      -1.050894081921180057e+00, -4.683296357081808137e-01, 1.039018920762839193e-01,  6.372298287674598161e-01,
      -1.341219362881158617e+00, -8.826052419777955516e-01, -3.473451667619283922e-01, 2.414243266840857827e-01,
      8.562013187364799771e-01,  1.466482765005763245e+00,  2.040373220738064042e+00,  2.543940200665009588e+00,
      2.953643468399339334e+00,  3.243886743134320572e+00,  3.394250171538570005e+00,  3.390834049607778233e+00,
      3.227267230967072908e+00,  2.905311748951458828e+00,  2.435019356021916881e+00,  1.834422765117191378e+00,
      1.128772541037670996e+00,  3.589120567094338243e-01,  -4.513928950862666767e-01, -1.264275854007859801e+00,
      -2.040845953957361569e+00, -2.743197886950301090e+00, -3.336393243051417379e+00, -3.790303968834916049e+00,
      -4.081216870314683831e+00, -4.193111843652504334e+00, -4.118544903769736365e+00, -3.859088939631526216e+00,
      -3.433802469184980222e+00, -2.851944620415314713e+00, -2.139378414617998381e+00, -1.328419646079702199e+00,
      -4.562511081348215591e-01, 4.369141757645528812e-01,  1.309825821176145544e+00,  2.122242247444659835e+00,
      2.836924644266589635e+00,  3.421475893320972794e+00,  3.849929467449864529e+00,  4.112904528649669267e+00,
      4.191207322046734518e+00,  4.083121391920456311e+00,  3.795944343954987144e+00,  3.345552715233082619e+00,
      2.755524462814314290e+00,  2.055865930106084427e+00,  1.281412021181196348e+00,  4.699867199898276060e-01,
      -3.395751243391672003e-01, -1.118989684367739157e+00, -1.825015858142812863e+00, -2.426349900902626366e+00,
      -2.897712907985633635e+00, -3.221031023351540679e+00, -3.386200129303894801e+00, -3.391396617504886635e+00,
      -3.242923215852055563e+00, -2.954606995681602122e+00, -2.546793754698695178e+00, -2.045007141041944365e+00,
      -1.472718972621298805e+00, -8.638001597023031719e-01, -2.500937818033618920e-01, 3.379382597875353889e-01,
      -1.061487577118566561e+00, -5.371625699491856532e-01, 5.097859948621857179e-02,  6.775853941825510374e-01,
      1.302407540139333575e+00,  1.907976040579533628e+00,  2.462824555025283857e+00,  2.936251027161979277e+00,
      3.299464497974518906e+00,  3.533171993928367005e+00,  3.618222048200857355e+00,  3.542921835909766504e+00,
      3.303330766901243720e+00,  2.915207089893366632e+00,  2.380068068200154041e+00,  1.717424731449994413e+00,
      9.539988129977814424e-01,  1.333637811601277867e-01,  -7.213392009222857615e-01, -1.570159330694270050e+00,
      -2.373164599248509976e+00, -3.091648688191982242e+00, -3.691407524726153078e+00, -4.143249670348978952e+00,
      -4.423240008300159509e+00, -4.515400998317040226e+00, -4.418017483350750396e+00, -4.130202349815627727e+00,
      -3.660626527068677749e+00, -3.028906793076438397e+00, -2.262639328572564335e+00, -1.399899916306618364e+00,
      -4.737944543156984434e-01, 4.737944543157069921e-01,  1.399899916306622583e+00,  2.262639328572558117e+00,
      3.028906793076440618e+00,  3.660626527068682190e+00,  4.130202349815627727e+00,  4.418017483350752173e+00,
      4.515400998317042891e+00,  4.423240008300160397e+00,  4.143249670348978952e+00,  3.691407524726154854e+00,
      3.091648688191982242e+00,  2.373164599248512197e+00,  1.570159330694274269e+00,  7.213392009222814316e-01,
      -1.333637811601320333e-01, -9.539988129977814424e-01, -1.717424731449996633e+00, -2.380068068200154041e+00,
      -2.915207089893368853e+00, -3.303330766901239279e+00, -3.542921835909765615e+00, -3.618222048200857355e+00,
      -3.533171993928365673e+00, -3.299464497974520238e+00, -2.936251027161974836e+00, -2.462824555025283857e+00,
      -1.907976040579535848e+00, -1.302407540139331577e+00, -6.775853941825532578e-01, -5.097859948621857179e-02,
      -7.114936829708495836e-01, -1.379102829465287672e-01, 4.843713388167624756e-01,  1.115049520927855253e+00,
      1.740692187068238139e+00,  2.330201041494530667e+00,  2.845469589312975778e+00,  3.268427875694002172e+00,
      3.572331834603324374e+00,  3.737192458124688255e+00,  3.749435648157773748e+00,  3.599375002739075846e+00,
      3.294706442139387903e+00,  2.834519786975144839e+00,  2.232667745518693181e+00,  1.523127112154915697e+00,
      7.202568606638372062e-01,  -1.419502037695177799e-01, -1.015581748156518360e+00, -1.876038093914821792e+00,
      -2.681801950488389430e+00, -3.389494380358811565e+00, -3.975093042144000055e+00, -4.406455547408394402e+00,
      -4.661764513198615845e+00, -4.729110139503870158e+00, -4.599774444384014060e+00, -4.282043708815530536e+00,
      -3.782579261620171884e+00, -3.121081956741387220e+00, -2.332483706122348455e+00, -1.439913158542848626e+00,
      -4.849152329680061491e-01, 4.849152329680104789e-01,  1.439913158542844407e+00,  2.332483706122357336e+00,
      3.121081956741385000e+00,  3.782579261620178546e+00,  4.282043708815536753e+00,  4.599774444384011396e+00,
      4.729110139503871046e+00,  4.661764513198617621e+00,  4.406455547408393514e+00,  3.975093042144003608e+00,
      3.389494380358804904e+00,  2.681801950488396091e+00,  1.876038093914813132e+00,  1.015581748156522801e+00,
      1.419502037695177799e-01,  -7.202568606638415361e-01, -1.523127112154915697e+00, -2.232667745518690960e+00,
      -2.834519786975140399e+00, -3.294706442139392344e+00, -3.599375002739074514e+00, -3.749435648157771972e+00,
      -3.737192458124686922e+00, -3.572331834603324374e+00, -3.268427875694003060e+00, -2.845469589312976666e+00,
      -2.330201041494530667e+00, -1.740692187068231700e+00, -1.115049520927855253e+00, -4.843713388167624756e-01,
      -2.771034698004570807e-01, 3.278992266036562242e-01,  9.638561084178509653e-01,  1.590395283532537452e+00,
      2.193380715117372315e+00,  2.732911448899550955e+00,  3.193977353112047446e+00,  3.544043091821261982e+00,
      3.769111854354786573e+00,  3.850754786655909978e+00,  3.779386096908943227e+00,  3.548804487878554070e+00,
      3.168350337046399456e+00,  2.640884708729922536e+00,  1.993952446498630238e+00,  1.237878743177773799e+00,
      4.150018254069109958e-01,  -4.570293222655053000e-01, -1.328330705747484686e+00, -2.177051967579779479e+00,
      -2.962989114803842305e+00, -3.645781613485978845e+00, -4.199485251882759229e+00, -4.593857235324866117e+00,
      -4.814989130032860665e+00, -4.848576127317847373e+00, -4.690535944033923599e+00, -4.347775173684141237e+00,
      -3.827276757348569980e+00, -3.149965675972792312e+00, -2.345719374249584099e+00, -1.444298180777963125e+00,
      -4.903228535018744338e-01, 4.903228535018744338e-01,  1.444298180777972007e+00,  2.345719374249588540e+00,
      3.149965675972796753e+00,  3.827276757348561098e+00,  4.347775173684151895e+00,  4.690535944033921822e+00,
      4.848576127317849149e+00,  4.814989130032861553e+00,  4.593857235324866117e+00,  4.199485251882758341e+00,
      3.645781613485981065e+00,  2.962989114803837865e+00,  2.177051967579775038e+00,  1.328330705747489127e+00,
      4.570293222655053000e-01,  -4.150018254069154366e-01, -1.237878743177769358e+00, -1.993952446498632458e+00,
      -2.640884708729924757e+00, -3.168350337046390575e+00, -3.548804487878560732e+00, -3.779386096908940118e+00,
      -3.850754786655909978e+00, -3.769111854354785240e+00, -3.544043091821260649e+00, -3.193977353112048334e+00,
      -2.732911448899550955e+00, -2.193380715117372315e+00, -1.590395283532535231e+00, -9.638561084178487448e-01,
      2.024021766250654886e-01,  8.141734992970954821e-01,  1.424170872668138577e+00,  2.022008399986619409e+00,
      2.568363900265215438e+00,  3.051104519296965023e+00,  3.435081233820335367e+00,  3.708850398177416352e+00,
      3.849285661993011143e+00,  3.847701765775221716e+00,  3.697666851380436270e+00,  3.395772047856999709e+00,
      2.954292035405053696e+00,  2.376707045572244326e+00,  1.692836849184913062e+00,  9.254379011141148359e-01,
      9.283062905207831172e-02,  -7.635641642972592624e-01, -1.623106665875771126e+00, -2.436123563358239874e+00,
      -3.183328169162753074e+00, -3.820413433825709415e+00, -4.330505776903590842e+00, -4.682225652365088031e+00,
      -4.864592986792697360e+00, -4.864673672392239467e+00, -4.680833627762423177e+00, -4.319109235572622829e+00,
      -3.791460534661990955e+00, -3.112227813649470320e+00, -2.317101712775890388e+00, -1.424930078047670357e+00,
      -4.836995077504246954e-01, 4.836995077504337437e-01,  1.424930078047674797e+00,  2.317101712775885947e+00,
      3.112227813649479202e+00,  3.791460534661995840e+00,  4.319109235572622829e+00,  4.680833627762423177e+00,
      4.864673672392239467e+00,  4.864592986792697360e+00,  4.682225652365085367e+00,  4.330505776903591730e+00,
      3.820413433825709415e+00,  3.183328169162755739e+00,  2.436123563358228328e+00,  1.623106665875771126e+00,
      7.635641642972592624e-01,  -9.283062905207831172e-02, -9.254379011141148359e-01, -1.692836849184913062e+00,
      -2.376707045572244326e+00, -2.954292035405047034e+00, -3.395772047857001930e+00, -3.697666851380436270e+00,
      -3.847701765775221272e+00, -3.849285661993009811e+00, -3.708850398177414576e+00, -3.435081233820336699e+00,
      -3.051104519296962803e+00, -2.568363900265210997e+00, -2.022008399986621630e+00, -1.424170872668138577e+00,
      6.605067378489495322e-01,  1.259291168561857077e+00,  1.839190999474684673e+00,  2.392853563815602325e+00,
      2.881371508884989385e+00,  3.284162147995321579e+00,  3.592573889856939040e+00,  3.780012536289011571e+00,
      3.839539838641600866e+00,  3.758208348606792537e+00,  3.536408001681977886e+00,  3.173611396499208759e+00,
      2.681709432899799239e+00,  2.078629325770993130e+00,  1.372126994833993141e+00,  5.987115712158778136e-01,
      -2.290511017467316557e-01, -1.068280861872652032e+00, -1.884485092349391211e+00, -2.661610071439171943e+00,
      -3.356104306738467713e+00, -3.938910276008287248e+00, -4.396951730874283371e+00, -4.701241079245408017e+00,
      -4.838808287552481602e+00, -4.805779829304563044e+00, -4.600189668902620710e+00, -4.225013635489649921e+00,
      -3.692710538330051140e+00, -3.028518370162588180e+00, -2.246204611923688255e+00, -1.379371447969170106e+00,
      -4.680439652537387030e-01, 4.680439652537479178e-01,  1.379371447969170106e+00,  2.246204611923706462e+00,
      3.028518370162578854e+00,  3.692710538330046699e+00,  4.225013635489658803e+00,  4.600189668902624263e+00,
      4.805779829304561268e+00,  4.838808287552480714e+00,  4.701241079245407128e+00,  4.396951730874281594e+00,
      3.938910276008289468e+00,  3.356104306738465493e+00,  2.661610071439167058e+00,  1.884485092349391211e+00,
      1.068280861872652032e+00,  2.290511017467362631e-01,  -5.987115712158778136e-01, -1.372126994834002245e+00,
      -2.078629325770988245e+00, -2.681709432899797019e+00, -3.173611396499215420e+00, -3.536408001681970781e+00,
      -3.758208348606794758e+00, -3.839539838641600422e+00, -3.780012536289010239e+00, -3.592573889856939928e+00,
      -3.284162147995320247e+00, -2.881371508884984500e+00, -2.392853563815604545e+00, -1.839190999474686894e+00,
      1.099742471735307481e+00,  1.665601980077083821e+00,  2.198787495795696767e+00,  2.693450549931069471e+00,
      3.113302169974171640e+00,  3.440100969103784490e+00,  3.658500194168339270e+00,  3.765715036234766000e+00,
      3.743078910475429399e+00,  3.587369291852109843e+00,  3.299851380107230003e+00,  2.887907978728421643e+00,
      2.361118811572981979e+00,  1.735883993601275765e+00,  1.025873600239942895e+00,  2.606375742019900188e-01,
      -5.328965530053541011e-01, -1.339384197662834453e+00, -2.114307537064144871e+00, -2.831068445531586431e+00,
      -3.462410709835448852e+00, -3.995364296207474908e+00, -4.397073787620039198e+00, -4.651070694840991671e+00,
      -4.752872146744155479e+00, -4.692533946110505561e+00, -4.467789325488516816e+00, -4.085456995938431035e+00,
      -3.563760911500926554e+00, -2.914491632597960180e+00, -2.160697897191358852e+00, -1.323993923198789835e+00,
      -4.467453779711883510e-01, 4.467453779711977324e-01,  1.323993923198799161e+00,  2.160697897191358852e+00,
      2.914491632597962401e+00,  3.563760911500926554e+00,  4.085456995938435476e+00,  4.467789325488519481e+00,
      4.692533946110507337e+00,  4.752872146744153703e+00,  4.651070694840989894e+00,  4.397073787620040086e+00,
      3.995364296207472687e+00,  3.462410709835446632e+00,  2.831068445531586431e+00,  2.114307537064140430e+00,
      1.339384197662829790e+00,  5.328965530053541011e-01,  -2.606375742019853003e-01, -1.025873600239942895e+00,
      -1.735883993601275765e+00, -2.361118811572979759e+00, -2.887907978728423863e+00, -3.299851380107232224e+00,
      -3.587369291852107622e+00, -3.743078910475428511e+00, -3.765715036234766888e+00, -3.658500194168338382e+00,
      -3.440100969103784490e+00, -3.113302169974171640e+00, -2.693450549931065030e+00, -2.198787495795699432e+00,
      1.505193806293808167e+00,  2.024204462574603625e+00,  2.498302251064618229e+00,  2.908943375348887805e+00,
      3.254375314428450405e+00,  3.503271259727063658e+00,  3.643303624654641748e+00,  3.665779823811112781e+00,
      3.566068526619294854e+00,  3.349534192260151144e+00,  3.012231829827727747e+00,  2.562591829633729112e+00,
      2.013388830954600639e+00,  1.381353499245159711e+00,  6.803561692049466814e-01,  -6.202150451162694728e-02,
      -8.205006539566508383e-01, -1.568722592125154192e+00, -2.280225567125432118e+00, -2.942609454119787138e+00,
      -3.518921490827651599e+00, -3.987481371649471029e+00, -4.330254244889022885e+00, -4.543374714151752869e+00,
      -4.610668908511220465e+00, -4.524702499854477367e+00, -4.286960365934854522e+00, -3.904479299503564871e+00,
      -3.390253341928263886e+00, -2.766811022176318247e+00, -2.048090139012233024e+00, -1.258845782607607600e+00,
      -4.264675248138233843e-01, 4.264675248138329322e-01,  1.258845782607607600e+00,  2.048090139012233024e+00,
      2.766811022176330237e+00,  3.390253341928266106e+00,  3.904479299503569312e+00,  4.286960365934858963e+00,
      4.524702499854474702e+00,  4.610668908511218689e+00,  4.543374714151751981e+00,  4.330254244889020221e+00,
      3.987481371649464812e+00,  3.518921490827666254e+00,  2.942609454119772927e+00,  2.280225567125441888e+00,
      1.568722592125154192e+00,  8.205006539566364054e-01,  6.202150451162694728e-02,  -6.803561692049466814e-01,
      -1.381353499245159711e+00, -2.013388830954600639e+00, -2.562591829633729112e+00, -3.012231829827727747e+00,
      -3.349534192260146259e+00, -3.566068526619292633e+00, -3.665779823811114557e+00, -3.643303624654642192e+00,
      -3.503271259727061437e+00, -3.254375314428452626e+00, -2.908943375348880700e+00, -2.498302251064615565e+00,
      1.835003291318242802e+00,  2.295537275354685658e+00,  2.702675434319650183e+00,  3.040552089083366827e+00,
      3.295162279449598941e+00,  3.467968301088316885e+00,  3.534974301888240689e+00,  3.490172454951459446e+00,
      3.334079256367743493e+00,  3.065765298731110811e+00,  2.690818300107997274e+00,  2.218725463354223049e+00,
      1.662625030892673328e+00,  1.032575211005031468e+00,  3.522940574289843974e-01,  -3.563964012403376680e-01,
      -1.068829579946018349e+00, -1.763124199994398511e+00, -2.415671031882382636e+00, -3.003898988011620919e+00,
      -3.507108506276563897e+00, -3.920425449237697979e+00, -4.214641460472062562e+00, -4.378208351886458516e+00,
      -4.409588412179642702e+00, -4.301421162494841965e+00, -4.055921664496842105e+00, -3.680107151603508875e+00,
      -3.187966843938493167e+00, -2.591069196419314213e+00, -1.908912825256781520e+00, -1.171246689396915786e+00,
      -3.947791954139448500e-01, 3.947791954139448500e-01,  1.171246689396935325e+00,  1.908912825256776635e+00,
      2.591069196419326648e+00,  3.187966843938493167e+00,  3.680107151603508875e+00,  4.055921664496842993e+00,
      4.301421162494846406e+00,  4.409588412179640926e+00,  4.378208351886459404e+00,  4.214641460472058121e+00,
      3.920425449237695759e+00,  3.507108506276563897e+00,  3.003898988011625804e+00,  2.415671031882377751e+00,
      1.763124199994398511e+00,  1.068829579946018349e+00,  3.563964012403376680e-01,  -3.522940574289892268e-01,
      -1.032575211005036353e+00, -1.662625030892675770e+00, -2.218725463354225713e+00, -2.690818300107992833e+00,
      -3.065765298731110811e+00, -3.334079256367740829e+00, -3.490172454951459446e+00, -3.534974301888238912e+00,
      -3.467968301088316441e+00, -3.295162279449597165e+00, -3.040552089083363718e+00, -2.702675434319650183e+00,
      2.115569753652723151e+00,  2.508297977154028580e+00,  2.841319059700373906e+00,  3.101631182530637965e+00,
      3.278172047199655825e+00,  3.362269852100345791e+00,  3.348023644958015410e+00,  3.232596809549412864e+00,
      3.016410242216812421e+00,  2.703225175067778707e+00,  2.305123990017395208e+00,  1.824413191299517178e+00,
      1.274079672966802690e+00,  6.697990012480489197e-01,  2.945652433251328556e-02,  -6.274223354371449846e-01,
      -1.280313373921262432e+00, -1.908384129310881416e+00, -2.491208981233588471e+00, -3.009482786725431236e+00,
      -3.445706627475587425e+00, -3.784819880840852413e+00, -4.014754481147451592e+00, -4.126889856495164821e+00,
      -4.116390518313046698e+00, -3.982412518106282828e+00, -3.743630359203476310e+00, -3.389672605394588700e+00,
      -2.930713267476816597e+00, -2.380479560059079791e+00, -1.755802165203605769e+00, -1.076052174965131281e+00,
      -3.624849847815463377e-01, 3.624849847815512782e-01,  1.076052174965136388e+00,  1.755802165203605769e+00,
      2.380479560059094446e+00,  2.930713267476816597e+00,  3.389672605394588700e+00,  3.743630359203481195e+00,
      3.982412518106282828e+00,  4.099718797800973036e+00,  4.110858820852761220e+00,  3.999980194084144625e+00,
      3.771870109922978109e+00,  3.435079025137181663e+00,  3.001585765764364755e+00,  2.486346019736672908e+00,
      1.906742108161267879e+00,  1.281955395070870862e+00,  6.322852969340581053e-01,  -2.155950337144188142e-02,
      -6.591713989096591453e-01, -1.261129902048925722e+00, -1.809638904236211987e+00, -2.289092954374990274e+00,
      -2.686553454555703713e+00, -2.992119441308128014e+00, -3.209239490497211023e+00, -3.326497416434534227e+00,
      -3.343401954995699477e+00, -3.262687564218334835e+00, -3.090125173669815428e+00, -2.834233694427928274e+00,
    };

    {
        auto [psi, x] = integrate_wavelet<double>(cwt_wavelet_t::MORLET, 10);
        auto r        = cwt<double>(data, scales, psi, x);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); i++) {
            auto diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-9);
        }
    }

    {
        auto [psi, x] = integrate_wavelet<float>(cwt_wavelet_t::MORLET, 10);
        auto r        = cwt<float>(data, scales, psi, x);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); i++) {
            auto diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-4);
        }
    }
}

#endif

}  // namespace tests

}  // namespace dsp
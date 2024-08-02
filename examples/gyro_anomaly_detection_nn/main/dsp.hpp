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

#ifdef EPS
    #undef EPS
#endif
static constexpr double EPS = 1.0e-20;
#ifdef PI
    #undef PI
#endif
static constexpr double PI = 3.14159265358979323846264338327950288419716939937510582097494459231;
#ifdef SQRT_2
    #undef SQRT_2
#endif
static constexpr double SQRT_2 = 1.41421356237309504880168872420969807856967187537694807317667973799;
#ifdef SQRT_2PI
    #undef SQRT_2PI
#endif
static constexpr double SQRT_2PI = 2.50662827463100050241576528481104525300698674060993831662992357634;

}  // namespace constants

using namespace constants;

namespace math {

// MARK: Sinc
template <typename T, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) sinc(T x, T eps = static_cast<T>(EPS)) {
    if (std::abs(x) < eps) {
        x = static_cast<T>(eps);
    }
    x *= static_cast<T>(PI);
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
        const T start = -PI;
        const T end   = PI;
        const T step  = std::abs(end - start);
        const T size  = fac.size() - 1;
        ENSURE_TRUE(size != 0);
        std::generate(fac.begin(), fac.end(), [start, step, size, n = 0]() mutable {
            return start + (step * (static_cast<T>(n++) / size));
        });
    }

    const std::array<T, 2> a = {alpha, static_cast<T>(static_cast<T>(1.0) - alpha)};
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
        const auto alpha = static_cast<T>(0.5) * static_cast<T>(numtaps - 1);
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
        if (left == static_cast<T>(0.0)) {
            scale_frequency = 0.0;
        } else if (right == static_cast<T>(1.0)) {
            scale_frequency = 1.0;
        } else {
            scale_frequency = static_cast<T>(0.5) * (left + right);
        }

        const auto size = h.size();
        ENSURE_TRUE(size == m.size());
        T sum = 0.0;
        for (size_t i = 0; i < size; ++i) {
            sum += h[i] * std::cos(static_cast<T>(PI) * m[i] * scale_frequency);
        }
        if (std::abs(sum) < static_cast<T>(EPS)) [[unlikely]] {
            sum = EPS;
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
                sum += static_cast<T>(numerators[j] * x[i - j]);
            }
            y_i = sum;
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
            const size_t start = std::min((n * i) / segments, n);
            const size_t end   = std::min((n * (i + 1)) / segments, n);
            T            sum   = 0.0;
            for (size_t j = start; j < end; ++j) {
                sum += static_cast<T>(x[j]);
            }
            const auto dist = end - start;
            if (dist != 0) [[likely]] {
                y[i] = sum / static_cast<T>(dist);
            } else [[unlikely]] {
                y[i] = sum / static_cast<T>(EPS);
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
    if (std::abs(diff) < static_cast<T>(EPS)) [[unlikely]] {
        diff = EPS;
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

// MARK: Norm CDF, PDF, PPF
template <typename T = double, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) norm_cdf(T x, T std = 1.0, T mean = 0.0) {
    return std::erfc((mean - x) / (std * static_cast<T>(SQRT_2))) / static_cast<T>(2.0);
}

template <typename T = double, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) norm_pdf(T x, T std = 1.0, T mean = 0.0) {
    return (static_cast<T>(1.0) / (std * static_cast<T>(SQRT_2PI))) *
           static_cast<T>(std::exp(-static_cast<T>(0.5) * static_cast<T>(std::pow((x - mean) / std, 2))));
}

template <typename T = double, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) norm_ppf(T q, T tol, size_t max_iter, T std = 1.0, T mean = 0.0) {
    if (q < static_cast<T>(0.0) || q > static_cast<T>(1.0)) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The q must be in the range [0, 1].");
#else
        return std::numeric_limits<T>::quiet_NaN();
#endif
    }

    // Newton raphson method
    T x = mean;
    for (size_t i = 0; i < max_iter; ++i) {
        const T f_x = norm_cdf<T>(x, std, mean) - q;
        if (std::abs(f_x) < tol) [[unlikely]] {
            return x;
        }
        const T f_prime_x = norm_pdf<T>(x, std, mean);
        if (std::abs(f_prime_x) < static_cast<T>(EPS)) {
#ifdef ENABLE_THROW
            throw std::runtime_error("The probability density of x is too small.");
#else
            return std::numeric_limits<T>::quiet_NaN();
#endif
        }
        x -= f_x / f_prime_x;
    }
#ifdef ENABLE_THROW
    throw std::runtime_error("Failed to converge in limited iterations.");
#else
    return std::numeric_limits<T>::quiet_NaN();
#endif
}
template <typename T = double, std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
constexpr inline decltype(auto) norm_ppf(T q, T std = 1.0, T mean = 0.0) {
    if constexpr (std::is_same_v<T, float>) {
        return norm_ppf(q, static_cast<T>(1.0e-6), static_cast<size_t>(32), std, mean);
    } else if constexpr (std::is_same_v<T, double>) {
        return norm_ppf(q, static_cast<T>(1.0e-16), static_cast<size_t>(64), std, mean);
    }
#ifdef ENABLE_THROW
    throw std::invalid_argument("PPF presets are not defined for the type T.");
#else
    return std::numeric_limits<T>::quiet_NaN();
#endif
}

// MARK: Markov Transition Field
template <typename T = double> struct mtf_ctx_t {
    minmax_scale_ctx_t<T> minmax_ctx;
    std::vector<T>        bins;
    std::vector<size_t>   digitize;
    std::vector<T>        transition_matrix;

    std::vector<T>      result;
    std::vector<size_t> shape;
};

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
        auto space = static_cast<P>(n_bins) - static_cast<P>(1.0);
        if (std::abs(space) < static_cast<P>(EPS)) [[unlikely]] {
            space = EPS;
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
    }
    {
        std::fill(transition_matrix.begin(), transition_matrix.end(), static_cast<T>(0.0));
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
            if (std::abs(sum) < static_cast<T>(EPS)) [[unlikely]] {
                sum = EPS;
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
        if (s < 1) [[unlikely]] {
#ifdef ENABLE_THROW
            throw std::invalid_argument("The shape must be greater than or equal to 1.");
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
        // mse /= static_cast<T>(n_target);
        // if (mse < static_cast<T>(EPS)) [[unlikely]] {
        //     mse = EPS;
        // }
        // y = static_cast<T>(10.0) * std::log10(static_cast<T>(max * max) / mse);
        y = static_cast<T>(20) * std::log10(static_cast<T>(1.0) / std::sqrt(mse));
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

template <typename T = double, typename Container = std::vector<T>> struct cwt_ctx_t {
    cwt_ctx_t() = default;

    cwt_ctx_t(const Container& scales) {
        this->scales = scales;
        std::sort(this->scales.begin(), this->scales.end());
        std::transform(
          this->scales.begin(), this->scales.end(), std::back_inserter(negtive_sqrt_scales), [](const auto& s) {
              return -std::sqrt(s);
          });
    }

    ~cwt_ctx_t() = default;

    Container      scales;
    std::vector<T> negtive_sqrt_scales;

    std::vector<size_t> psi_arange;
    std::vector<size_t> psi_indices;
    std::vector<T>      coefficients;

    std::vector<T>      result;
    std::vector<size_t> shape;
};

template <typename T = double,
          typename P = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          typename Container_4,
          std::enable_if_t<(std::is_floating_point_v<T> || is_complex_v<T>)&&std::is_floating_point_v<P> &&
                             is_cwt_container_fine_v<Container_1, P> && is_cwt_container_fine_v<Container_2, T> &&
                             is_cwt_container_fine_v<Container_3, T> && is_cwt_container_fine_v<Container_4, T>,
                           bool> = true>
void cwt(cwt_ctx_t<T, Container_1>& ctx,
         const Container_2&         signal,
         const Container_3&         wavelet_psi,
         const Container_4&         wavelet_x) {
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
    const auto n_scales = ctx.scales.size();

    if (std::abs(x_step) <= static_cast<T>(EPS)) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The step of wavelet_x must be greater than 0.");
#else
        return;
#endif
    }
    if (std::abs(x_range) <= static_cast<T>(EPS)) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::invalid_argument("The range of wavelet_x must be greater than 0.");
#else
        return;
#endif
    }
    if (n_scales == 0) [[unlikely]] {
#ifdef ENABLE_THROW
        throw std::runtime_error("The size of ctx.scales must be greater than 0.");
#else
        return;
#endif
    }

    const auto max_scale         = ctx.scales[n_scales - 1];
    const auto n_psi_indices_max = static_cast<size_t>(std::ceil(max_scale * x_range)) + 1;
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
        ENSURE_TRUE(ctx.scales.size() >= n_scales);
        ENSURE_TRUE(ctx.negtive_sqrt_scales.size() >= n_scales);
        size_t result_pos = 0;
        for (size_t i = 0; i < n_scales; ++i) {
            const auto scale           = static_cast<P>(ctx.scales[i]);
            size_t     len_psi_indices = 0;

            {
                const auto psi_arange_end =
                  std::min(static_cast<size_t>(std::ceil(scale * static_cast<P>(x_range))) + 1, psi_arange.size());
                const auto scale_mul_step = std::max(scale * static_cast<P>(x_step), static_cast<P>(EPS));
                for (size_t j = 0; j < psi_arange_end; ++j) {
                    const auto idx = static_cast<size_t>(std::floor(static_cast<P>(psi_arange[j]) / scale_mul_step));
                    if (idx >= n_psi) [[unlikely]] {
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
                std::fill(it, it + n_signal, std::numeric_limits<T>::quiet_NaN());
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
                    coefficients[j + k] += static_cast<T>(signal_j) * static_cast<T>(psi_rk);
                }
            }

            const auto len_diff = len_conv - 1;
            ENSURE_TRUE(len_diff > 1);
            ENSURE_TRUE(coefficients.size() >= len_diff);

            const T negtive_sqrt_scale = ctx.negtive_sqrt_scales[i];
            for (size_t j = 1; j < len_conv; ++j) {
                auto&       coefficient_prev    = coefficients[j - 1];
                const auto& coefficient_current = coefficients[j];
                coefficient_prev                = coefficient_current - coefficient_prev;
                coefficient_prev *= negtive_sqrt_scale;
            }

            ENSURE_TRUE(len_diff >= n_signal);
            const auto d = static_cast<P>(len_diff - n_signal) / static_cast<P>(2.0);
            if (d < static_cast<P>(EPS)) [[unlikely]] {
#ifdef ENABLE_THROW
                throw std::runtime_error("Selected scale is too small.");
#else
                auto it = result.begin() + result_pos;
                std::fill(it, it + n_signal, std::numeric_limits<T>::quiet_NaN());
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
                result[result_pos++] = std::numeric_limits<T>::quiet_NaN();
            }
        }
    }
}

template <typename T = double,
          typename P = double,
          typename Container_1,
          typename Container_2,
          typename Container_3,
          typename Container_4,
          std::enable_if_t<(std::is_floating_point_v<T> || is_complex_v<T>)&&std::is_floating_point_v<P> &&
                             is_cwt_container_fine_v<Container_1, T> && is_cwt_container_fine_v<Container_2, T> &&
                             is_cwt_container_fine_v<Container_3, T> && is_cwt_container_fine_v<Container_4, T>,
                           bool> = true>
decltype(auto) cwt(const Container_1& signal,
                   const Container_2& scales,
                   const Container_3& wavelet_psi,
                   const Container_4& wavelet_x) {
    cwt_ctx_t<T, Container_2> ctx(scales);
    cwt<T, P>(ctx, signal, wavelet_psi, wavelet_x);
    const auto r = std::move(ctx.result);
    return r;
}

// MARK: Tests
namespace tests {

#ifdef BUILD_TESTS

void test_eps() {
    assert(static_cast<double>(EPS) > 0.0);
    assert(static_cast<double>(-EPS) < -0.0);
    assert(static_cast<float>(EPS) > 0.0f);
    assert(static_cast<float>(-EPS) < -0.0f);

    {
        double a    = 1.0;
        double b    = 2.0;
        double c    = 3.0;
        double diff = std::abs((a + b) - c);
        assert(diff < EPS);
    }

    {
        float  a    = 1.0;
        float  b    = 2.0;
        float  c    = 3.0;
        double diff = std::abs((a + b) - c);
        assert(diff < EPS);
    }
}

void test_pi() {
    {
        double math_pi = 2.0 * std::acos(0.0);
        double diff    = std::abs(math_pi - PI);
        assert(diff < EPS);
    }

    {
        float  math_pi = 2.0f * std::acos(0.0f);
        double diff    = std::abs(math_pi - static_cast<float>(PI));
        assert(diff < EPS);
    }
}

void test_sqrt_2() {
    {
        double math_sqrt_2 = std::sqrt(2.0);
        double diff        = std::abs(math_sqrt_2 - SQRT_2);
        assert(diff < EPS);
    }

    {
        float  math_sqrt_2 = std::sqrt(2.0f);
        double diff        = std::abs(math_sqrt_2 - static_cast<float>(SQRT_2));
        assert(diff < EPS);
    }
}

void test_sqrt_2pi() {
    {
        double math_sqrt_2pi = std::sqrt(2.0 * PI);
        double diff          = std::abs(math_sqrt_2pi - SQRT_2PI);
        assert(diff < 1e-15);
    }

    {
        float  math_sqrt_2pi = std::sqrt(2.0f * static_cast<float>(PI));
        double diff          = std::abs(math_sqrt_2pi - static_cast<float>(SQRT_2PI));
        assert(diff < EPS);
    }
}

void test_sinc() {
    std::unordered_map<double, double> sinc_values = {
      {-1.000000000000000000e+01, -3.898171832519376111e-17},
      {-9.797979797979797567e+00, -1.926197637739193377e-02},
      {-9.595959595959595134e+00, -3.167529216345295329e-02},
      {-9.393939393939394478e+00, -3.202097548582460290e-02},
      {-9.191919191919192045e+00, -1.963689594706573976e-02},
      {-8.989898989898989612e+00,  1.123406938383345608e-03},
      {-8.787878787878787179e+00,  2.239062705539032402e-02},
      {-8.585858585858586522e+00,  3.573323328381772246e-02},
      {-8.383838383838384090e+00,  3.546686916735174860e-02},
      {-8.181818181818181657e+00,  2.103338319751807753e-02},
      {-7.979797979797979224e+00, -2.529946334270837503e-03},
      {-7.777777777777777679e+00, -2.630644082738655895e-02},
      {-7.575757575757576134e+00, -4.083251432108082890e-02},
      {-7.373737373737373701e+00, -3.981623910599781102e-02},
      {-7.171717171717171269e+00, -2.279908536992215212e-02},
      {-6.969696969696969724e+00,  4.341261672751295549e-03},
      {-6.767676767676768179e+00,  3.136071239083107476e-02},
      {-6.565656565656565746e+00,  4.745336466131256509e-02},
      {-6.363636363636363313e+00,  4.549990608592487107e-02},
      {-6.161616161616161769e+00,  2.511698613996069321e-02},
      {-5.959595959595959336e+00, -6.761470032864262854e-03},
      {-5.757575757575757791e+00, -3.815129506783636326e-02},
      {-5.555555555555555358e+00, -5.642532787936154620e-02},
      {-5.353535353535353813e+00, -5.327389425526406208e-02},
      {-5.151515151515151381e+00, -2.831361797209243705e-02},
      {-4.949494949494949836e+00,  1.016132087866636856e-02},
      {-4.747474747474747403e+00,  4.778489884461752796e-02},
      {-4.545454545454545858e+00,  6.931538911162694883e-02},
      {-4.343434343434343425e+00,  6.459757362729441621e-02},
      {-4.141414141414141881e+00,  3.303411947658101144e-02},
      {-3.939393939393939448e+00, -1.529182990563534086e-02},
      {-3.737373737373737903e+00, -6.256473652502492211e-02},
      {-3.535353535353535470e+00, -8.948146354933080027e-02},
      {-3.333333333333333037e+00, -8.269933431326874362e-02},
      {-3.131313131313131493e+00, -4.075611340708284319e-02},
      {-2.929292929292929060e+00,  2.393991393455531871e-02},
      {-2.727272727272727515e+00,  8.820627236525578618e-02},
      {-2.525252525252525082e+00,  1.256542571889122661e-01},
      {-2.323232323232323537e+00,  1.164222803677198020e-01},
      {-2.121212121212121104e+00,  5.577180743829834170e-02},
      {-1.919191919191918672e+00, -4.165445175934162636e-02},
      {-1.717171717171718015e+00, -1.438732598726785439e-01},
      {-1.515151515151515582e+00, -2.098465703717123654e-01},
      {-1.313131313131313149e+00, -2.018192796247390008e-01},
      {-1.111111111111110716e+00, -9.798155360510130141e-02},
      {-9.090909090909100598e-01,  9.864608391270920928e-02},
      {-7.070707070707076269e-01,  3.582369603998353802e-01},
      {-5.050505050505051940e-01,  6.301742431604163697e-01},
      {-3.030303030303027612e-01,  8.556490093311446277e-01},
      {-1.010101010101003283e-01,  9.833009727996325777e-01},
      { 1.010101010101003283e-01,  9.833009727996325777e-01},
      { 3.030303030303027612e-01,  8.556490093311446277e-01},
      { 5.050505050505051940e-01,  6.301742431604163697e-01},
      { 7.070707070707076269e-01,  3.582369603998353802e-01},
      { 9.090909090909082835e-01,  9.864608391271117993e-02},
      { 1.111111111111110716e+00, -9.798155360510130141e-02},
      { 1.313131313131313149e+00, -2.018192796247390008e-01},
      { 1.515151515151515582e+00, -2.098465703717123654e-01},
      { 1.717171717171716239e+00, -1.438732598726793210e-01},
      { 1.919191919191918672e+00, -4.165445175934162636e-02},
      { 2.121212121212121104e+00,  5.577180743829834170e-02},
      { 2.323232323232323537e+00,  1.164222803677198020e-01},
      { 2.525252525252524194e+00,  1.256542571889123217e-01},
      { 2.727272727272726627e+00,  8.820627236525595272e-02},
      { 2.929292929292929060e+00,  2.393991393455531871e-02},
      { 3.131313131313131493e+00, -4.075611340708284319e-02},
      { 3.333333333333333925e+00, -8.269933431326888240e-02},
      { 3.535353535353534582e+00, -8.948146354933081414e-02},
      { 3.737373737373737015e+00, -6.256473652502501925e-02},
      { 3.939393939393939448e+00, -1.529182990563534086e-02},
      { 4.141414141414141881e+00,  3.303411947658101144e-02},
      { 4.343434343434342537e+00,  6.459757362729436070e-02},
      { 4.545454545454544970e+00,  6.931538911162697658e-02},
      { 4.747474747474747403e+00,  4.778489884461752796e-02},
      { 4.949494949494949836e+00,  1.016132087866636856e-02},
      { 5.151515151515150492e+00, -2.831361797209224623e-02},
      { 5.353535353535352925e+00, -5.327389425526397881e-02},
      { 5.555555555555555358e+00, -5.642532787936154620e-02},
      { 5.757575757575757791e+00, -3.815129506783636326e-02},
      { 5.959595959595958448e+00, -6.761470032864452806e-03},
      { 6.161616161616162657e+00,  2.511698613996069321e-02},
      { 6.363636363636363313e+00,  4.549990608592487107e-02},
      { 6.565656565656563970e+00,  4.745336466131265529e-02},
      { 6.767676767676768179e+00,  3.136071239083107476e-02},
      { 6.969696969696968836e+00,  4.341261672751457745e-03},
      { 7.171717171717173045e+00, -2.279908536992241580e-02},
      { 7.373737373737373701e+00, -3.981623910599781102e-02},
      { 7.575757575757574358e+00, -4.083251432108087053e-02},
      { 7.777777777777778567e+00, -2.630644082738655895e-02},
      { 7.979797979797979224e+00, -2.529946334270837503e-03},
      { 8.181818181818179880e+00,  2.103338319751785201e-02},
      { 8.383838383838384090e+00,  3.546686916735174860e-02},
      { 8.585858585858584746e+00,  3.573323328381780573e-02},
      { 8.787878787878788955e+00,  2.239062705539011586e-02},
      { 8.989898989898989612e+00,  1.123406938383345608e-03},
      { 9.191919191919190268e+00, -1.963689594706563915e-02},
      { 9.393939393939394478e+00, -3.202097548582460290e-02},
      { 9.595959595959595134e+00, -3.167529216345295329e-02},
      { 9.797979797979799343e+00, -1.926197637739174642e-02},
      { 1.000000000000000000e+01, -3.898171832519376111e-17},
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
      8.000000000000007105e-02, 8.092612882293326315e-02, 8.370078609783421220e-02, 8.831279925915502815e-02,
      9.474359735767617918e-02, 1.029672858391652013e-01, 1.129507508126066928e-01, 1.246537923881543830e-01,
      1.380292865478989772e-01, 1.530233748976567720e-01, 1.695756815357131186e-01, 1.876195561652702071e-01,
      2.070823424716677907e-01, 2.278856706837192880e-01, 2.499457731411198758e-01, 2.731738215972489559e-01,
      2.974762848991688857e-01, 3.227553056045660562e-01, 3.489090940191323376e-01, 3.758323380677390246e-01,
      4.034166273489936394e-01, 4.315508896656635729e-01, 4.601218382732120693e-01, 4.890144280455349657e-01,
      5.181123187210786574e-01, 5.472983433640117301e-01, 5.764549801541227758e-01, 6.054648256057112432e-01,
      6.342110673099877749e-01, 6.625779542974161718e-01, 6.904512631260140143e-01, 7.177187578188393147e-01,
      7.442706417986562073e-01, 7.700000000000002398e-01, 7.948032293784108582e-01, 8.185804560833267463e-01,
      8.412359376148311751e-01, 8.626784483449079222e-01, 8.828216468508471859e-01, 9.015844235816823371e-01,
      9.188912274577231143e-01, 9.346723700880894548e-01, 9.488643063812647327e-01, 9.614098904187520223e-01,
      9.722586055615178857e-01, 9.813667678626688540e-01, 9.886977019672872347e-01, 9.942218887911413727e-01,
      9.979170843836189242e-01, 9.997684094962651091e-01, 9.997684094962651091e-01, 9.979170843836189242e-01,
      9.942218887911413727e-01, 9.886977019672872347e-01, 9.813667678626687429e-01, 9.722586055615178857e-01,
      9.614098904187520223e-01, 9.488643063812647327e-01, 9.346723700880894548e-01, 9.188912274577231143e-01,
      9.015844235816822261e-01, 8.828216468508471859e-01, 8.626784483449077001e-01, 8.412359376148311751e-01,
      8.185804560833266352e-01, 7.948032293784108582e-01, 7.699999999999997957e-01, 7.442706417986559853e-01,
      7.177187578188392036e-01, 6.904512631260140143e-01, 6.625779542974159497e-01, 6.342110673099876639e-01,
      6.054648256057111322e-01, 5.764549801541227758e-01, 5.472983433640115081e-01, 5.181123187210784353e-01,
      4.890144280455349657e-01, 4.601218382732121803e-01, 4.315508896656633508e-01, 4.034166273489935284e-01,
      3.758323380677390246e-01, 3.489090940191320600e-01, 3.227553056045658897e-01, 2.974762848991688857e-01,
      2.731738215972489559e-01, 2.499457731411195427e-01, 2.278856706837191770e-01, 2.070823424716677907e-01,
      1.876195561652702071e-01, 1.695756815357130076e-01, 1.530233748976566610e-01, 1.380292865478989772e-01,
      1.246537923881543830e-01, 1.129507508126065818e-01, 1.029672858391651458e-01, 9.474359735767617918e-02,
      8.831279925915491713e-02, 8.370078609783421220e-02, 8.092612882293320764e-02, 8.000000000000007105e-02,
    };

    {
        auto w = hamming_window<double>(expected.size());
        assert(w.size() == expected.size());
        for (size_t i = 0; i < w.size(); ++i) {
            double diff = std::abs(w[i] - expected[i]);
            assert(diff < 1e-14);
        }
    }

    {
        auto w = hamming_window<float>(expected.size());
        assert(w.size() == expected.size());
        for (size_t i = 0; i < w.size(); ++i) {
            double diff = std::abs(static_cast<double>(w[i]) - expected[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_firwin() {
    std::vector<double> expected = {
      3.635039006488662429e-04,  3.752937352658930513e-04,  -3.963329817315765658e-04, -4.271643351860715033e-04,
      4.683416281457542093e-04,  5.204314598503628850e-04,  -5.840152028244973348e-04, -6.596914465607415200e-04,
      7.480789491461126194e-04,  8.498201810045427802e-04,  -9.655855613368285063e-04, -1.096078508101363180e-03,
      1.242041447525274803e-03,  1.404262960506157338e-03,  -1.583586282616530330e-03, -1.780919424070652891e-03,
      1.997247239034851980e-03,  2.233645854172165972e-03,  -2.491299969867830349e-03, -2.771523681738332582e-03,
      3.075785645162931042e-03,  3.405739636068801004e-03,  -3.763261867185891188e-03, -4.150496828949181913e-03,
      4.569913978947260161e-03,  5.024378362376675025e-03,  -5.517239295110206408e-03, -6.052442709867453773e-03,
      6.634674849821037453e-03,  7.269547992754454499e-03,  -7.963843271817355046e-03, -8.725832174177281311e-03,
      9.565708164925903687e-03,  1.049617513123603935e-02,  -1.153326344298771788e-02, -1.269748348678473131e-02,
      1.401549163157910470e-02,  1.552255555846477134e-02,  -1.726630549377525942e-02, -1.931262858266904678e-02,
      2.175528552620915532e-02,  2.473231484060181562e-02,  -2.845555981587593458e-02, -3.326745027276156730e-02,
      3.975972700061706361e-02,  4.905046411172205717e-02,  -6.353598509203629841e-02, -8.944737483480084961e-02,
      1.496330354877559443e-01,  4.497318992892901934e-01,  4.497318992892901934e-01,  1.496330354877559443e-01,
      -8.944737483480084961e-02, -6.353598509203629841e-02, 4.905046411172205023e-02,  3.975972700061706361e-02,
      -3.326745027276156730e-02, -2.845555981587593458e-02, 2.473231484060181562e-02,  2.175528552620915532e-02,
      -1.931262858266904331e-02, -1.726630549377525942e-02, 1.552255555846476787e-02,  1.401549163157910470e-02,
      -1.269748348678472957e-02, -1.153326344298771788e-02, 1.049617513123603241e-02,  9.565708164925901952e-03,
      -8.725832174177279577e-03, -7.963843271817355046e-03, 7.269547992754451897e-03,  6.634674849821035718e-03,
      -6.052442709867452905e-03, -5.517239295110206408e-03, 5.024378362376673290e-03,  4.569913978947257559e-03,
      -4.150496828949181913e-03, -3.763261867185892055e-03, 3.405739636068799269e-03,  3.075785645162930175e-03,
      -2.771523681738332582e-03, -2.491299969867828180e-03, 2.233645854172164671e-03,  1.997247239034851980e-03,
      -1.780919424070652891e-03, -1.583586282616528161e-03, 1.404262960506156687e-03,  1.242041447525274803e-03,
      -1.096078508101363180e-03, -9.655855613368278557e-04, 8.498201810045421297e-04,  7.480789491461126194e-04,
      -6.596914465607415200e-04, -5.840152028244967927e-04, 5.204314598503625598e-04,  4.683416281457542093e-04,
      -4.271643351860709612e-04, -3.963329817315765658e-04, 3.752937352658928345e-04,  3.635039006488662429e-04,
    };

    {
        auto h = firwin<double>(100, 0.5);
        assert(h.size() == expected.size());
        for (size_t i = 0; i < h.size(); ++i) {
            double diff = std::abs(h[i] - expected[i]);
            assert(diff < 1e-17);
        }
    }

    {
        auto h = firwin<float>(100, 0.5);
        assert(h.size() == expected.size());
        for (size_t i = 0; i < h.size(); ++i) {
            double diff = std::abs(static_cast<double>(h[i]) - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_lfilter() {
    std::vector<double> data = {
      7.758785505079287548e-01,  -8.048745963192359687e-01, -2.335946529031862062e-01, 1.021825377695771309e+00,
      -6.610059209943869307e-01, -1.078421670202891924e+00, 7.944201917993521933e-01,  6.157182595413788206e-01,
      -1.037705309816154386e+00, -3.024026208955163941e-01, 1.075148747530737303e+00,  2.379412765486228487e-01,
      -9.065305269066586957e-01, -1.496834947691895457e-01, 1.038664422166346135e+00,  1.696657258595887507e-01,
      -9.328259686601119149e-01, -5.374681078670161538e-01, 5.675279328044953164e-01,  6.655218535994834594e-01,
      -5.515813659490748089e-01, -1.018675994086865488e+00, -1.419007143243637969e-01, 9.915514191362425622e-01,
      9.394113959115492074e-01,  -5.228622132001865541e-01, -1.086928378399568240e+00, -4.556530553455786503e-01,
      5.771261747907608930e-01,  1.027341634303461815e+00,  4.644749777194165929e-01,  -5.288080428613484152e-01,
      -8.180536406834181262e-01, -8.474601479869406928e-01, -2.979935968395131529e-01, 5.168690725219144966e-01,
      1.158698454281837709e+00,  1.165296126487883388e+00,  3.160129313530735673e-01,  -3.883183951711627335e-01,
      -7.299825753799842332e-01, -8.121674216296503879e-01, -9.765482115887189618e-01, -5.903073602700101841e-01,
      -4.752230755038068521e-02, 5.037787991931201859e-01,  8.199919223166741711e-01,  6.572089015871844797e-01,
      7.732331519882515991e-01,  1.249562038138374742e+00,  1.038148043083451766e+00,  5.516300967867800997e-01,
      3.132112735916603441e-01,  3.277964589600254053e-01,  2.143963066905003600e-01,  -5.783684550196797891e-02,
      -3.037957039029700046e-01, -3.858355194962742596e-01, -7.025360225383064927e-02, -3.000913802111795370e-01,
      -2.015813027336103291e-01, 6.465865746238344883e-03,  3.758566491616285865e-01,  6.110701807703095012e-01,
      5.168260694248962839e-01,  6.488783614146091949e-01,  1.039570884151026275e+00,  1.097944029873278771e+00,
      8.308540862647779690e-01,  7.698553268151763218e-01,  7.737022832153850338e-01,  2.136674697686018476e-01,
      -3.264744815314880033e-01, -8.366832026203162576e-01, -9.748706521661794078e-01, -8.662982843253909104e-01,
      -3.884121445821469565e-01, -1.435079277596562619e-01, 6.929932296814802495e-01,  1.172601769525126292e+00,
      1.018338753431271115e+00,  4.604765987083092593e-03,  -7.145389817245433317e-01, -1.060789028336126316e+00,
      -6.604795925880470042e-01, 7.342572958148116391e-02,  5.867688607707188808e-01,  7.295166528452791121e-01,
      1.794588155507974847e-01,  -7.029921181247038575e-01, -9.163029186026743211e-01, 3.179603251693904342e-01,
      1.148936957733460051e+00,  6.569545384507228247e-01,  -6.119726933900101473e-01, -1.079737179574907557e+00,
      1.288137278087668480e-01,  1.099925476993814932e+00,  2.169884909874185419e-01,  -1.182512654213021763e+00,
    };
    std::vector<double> expected = {
      2.820348795394204792e-04,  -1.392695962416402866e-06, -6.944832205586810988e-04, 2.713416042145869764e-04,
      9.429776428862596434e-04,  -9.184463311125031469e-04, -1.271867999126356612e-03, 1.546941053230812147e-03,
      1.469697239933995475e-03,  -2.317237950872780457e-03, -1.659564602992697098e-03, 3.313438447082675715e-03,
      1.990057735412806977e-03,  -4.522790764026658403e-03, -2.327939097065839097e-03, 6.165667799789844271e-03,
      2.776515056467869676e-03,  -8.364158895814797351e-03, -3.540750512936368354e-03, 1.103212584443297520e-02,
      4.414478185840504769e-03,  -1.447995156878884547e-02, -5.745737452976558066e-03, 1.844176992164072923e-02,
      7.492622684678495321e-03,  -2.282615839019682194e-02, -9.350952275557744206e-03, 2.777222518067025067e-02,
      1.099262024729930065e-02,  -3.374604805574553390e-02, -1.244683238581148661e-02, 4.144001643924181966e-02,
      1.441869964742972104e-02,  -5.111845598380481553e-02, -1.789425287837770676e-02, 6.192800698905434609e-02,
      2.245132795274849824e-02,  -7.382805038426670796e-02, -2.721764624756081372e-02, 8.901518812375450174e-02,
      3.421483030764929656e-02,  -1.080116658681754549e-01, -4.591718511952332105e-02, 1.302291412302520046e-01,
      6.409409006545999099e-02,  -1.585124541608891602e-01, -1.003303145886922654e-01, 1.913455963684796424e-01,
      2.134270373855348679e-01,  -5.065821006348153466e-02, -8.463983440833372507e-02, 1.400767164351787164e-01,
      6.854889502463334949e-02,  -3.159414121086323579e-01, -3.451478231784534945e-01, 6.788188381335083510e-02,
      1.760351565262222306e-01,  -1.958964856357286455e-01, -2.518571240662725863e-01, 2.430039812228102503e-01,
      3.934053827030695172e-01,  -1.494939853012700326e-01, -3.811832060275330836e-01, 2.313968170978885630e-01,
      5.837542887823288229e-01,  -1.481260914326865130e-01, -8.292007174873333719e-01, -2.128682565127707482e-01,
      7.389350466928579220e-01,  3.305561838697984034e-01,  -9.071590330556715820e-01, -9.205770629671589100e-01,
      5.036894064882490030e-01,  1.292180029091859916e+00,  2.873587470269327038e-01,  -1.065951880990499756e+00,
      -9.867548798116466680e-01, 2.183899761700220932e-01,  9.889743255196111749e-01,  6.965899154470766019e-01,
      -5.547091108322001890e-02, -6.428916394912139731e-01, -8.988647918569082629e-01, -6.886368220618857006e-01,
      7.669361246634008589e-02,  9.730626254694006994e-01,  1.263712915611414411e+00,  7.364120556044463362e-01,
      -6.592387271546586158e-02, -5.772049955059450621e-01, -7.903954890286879476e-01, -9.155423733956311594e-01,
      -8.438637515310060122e-01, -3.596092600072034462e-01, 3.161498940610707464e-01,  6.876810840799311153e-01,
      6.872268810975514786e-01,  7.410957607016566140e-01,  1.027577464245421401e+00,  1.157497500960512848e+00,
    };

    auto w = firwin<double>(100, 0.5);

    {
        auto r = lfilter<double>(w, 1.0, data);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-15);
        }
    }

    {
        auto r = lfilter<float>(w, 1.0, data);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(static_cast<double>(r[i]) - expected[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_paa() {
    std::vector<double> data = {
      -2.707123230673205150e-01, 1.048480526097400611e-01,  2.505278157235719672e-01,  -9.251999652780766858e-01,
      5.671436602859060150e-01,  -1.040180216082938092e+00, -1.536759514579374430e-01, 7.898518103468190610e-01,
      -1.226215846441854218e+00, -9.480069877134584821e-01, -5.696539419300646889e-01, -9.771502146977724124e-01,
      -7.706317111835507827e-01, -3.371129145348601697e-02, -1.032859245166130036e+00, 1.142427376683152973e+00,
      -6.097780076874727007e-01, 1.469416386876670311e+00,  1.492678836738333237e+00,  7.071252278892183885e-01,
      -1.858490256651611272e+00, -1.370623771128104851e+00, -3.301063879206029283e-01, -1.515289945872204669e+00,
      1.200060185821550496e+00,  -1.822619143420840748e+00, 2.693845413635515551e-01,  -4.464243756471875657e-01,
      1.114313600092154521e+00,  -1.380802598177499840e+00, 1.015424517555432615e+00,  2.240812832877029648e-01,
      -6.445511142707021079e-01, 6.615316603676937302e-01,  1.292964845138367957e+00,  -8.953120453084657537e-01,
      -5.683106531193177213e-01, -2.111618357922506295e+00, -8.183077187246110551e-01, -9.623836123304461143e-01,
      1.245016729991359483e-01,  1.085086947130721735e-01,  -4.393012630155798681e-01, -7.135603732210121786e-01,
      9.341809592297067866e-01,  5.865553231659231120e-02,  1.609714279427906014e+00,  8.599067438306581268e-01,
      -9.852030368738226018e-01, -9.583683207781137359e-01, 4.491086508702783475e-01,  -9.424625600410473369e-01,
      1.589094473370271654e-01,  3.880756169024742186e-01,  4.373377634040143680e-01,  4.182295511742706307e-01,
      -7.321849929469287366e-01, -1.428282966236801954e+00, -2.009217354005291334e+00, -2.334395828525029637e-01,
      1.803952369039028447e+00,  -1.948670660022757906e+00, 1.367851011658045124e+00,  -1.858739421184189000e+00,
      -1.233950727694971494e+00, -5.075697635372413696e-01, 1.407171498157143708e+00,  -9.410970527102838767e-01,
      8.735047328229837982e-01,  1.135100193751177322e+00,  1.165984400987112757e+00,  4.921449233362669429e-02,
      5.109467426852568606e-01,  6.312993903486825431e-01,  8.877020658357981375e-01,  5.765163666799102971e-02,
      -3.295444380146471675e-01, -2.832078687004639939e+00, -1.182553661837196568e+00, -5.484602008780896376e-02,
      1.248351830009002628e+00,  2.510342915668025476e+00,  -1.713364592110025175e-01, -4.580551375004912051e-01,
      -1.338781518930379333e+00, 1.320062921969912040e+00,  -1.409329277109643064e+00, -1.098297219607405228e+00,
      -4.867510608120249049e-01, -1.000970560034476264e+00, 1.773876785213478025e+00,  5.557830507037224699e-01,
      -5.487119717073534186e-01, 1.080015754404080930e+00,  -7.524335847542489297e-01, 1.158794968036509943e+00,
      7.508713298353077992e-01,  -1.262712693042902012e+00, -7.907005345100035498e-01, -1.707888483145026948e-01,
      -1.619385357673486459e+00, 6.428946640852444272e-01,  -6.341265693929329927e-01, -5.614828715356590116e-01,
      -1.025565474672023747e+00, -2.545527105477953245e-01, 3.961760954826345493e-01,  5.166931011354936043e-01,
      7.514899428189479869e-01,  1.148879007428065036e+00,  2.386198739520692946e-01,  -7.511773050888372882e-01,
      3.106640084718299000e-01,  -1.081209224458365981e+00, 3.649759963418085923e-02,  -2.944488687648951775e-01,
      -2.708711787807264182e-01, -1.288584452929955604e-01, -1.018221443280190819e+00, -1.917968747414895869e+00,
      -3.829450293378013348e-01, 1.636355548159416973e+00,  -1.503574276141866228e+00, -1.140920221757884406e-02,
      7.602182672823447618e-01,  4.946180419291597019e-01,  8.958526633919711157e-02,  -3.346244790761944143e-01,
      2.360550950966867401e+00,  6.102602242727093174e-01,  1.107425007192212263e+00,  9.291534659481502922e-01,
      -8.982336798590710991e-01, 2.969164067788503392e-01,  1.297420113403151776e+00,  2.830474471869959263e-01,
      -1.918192600599175346e-01, -7.884036595078141030e-01, 4.317659077567626569e-01,  5.909669056964579614e-02,
      1.470858121465076218e+00,  -1.554218047610205877e+00, 2.726268613063043134e-01,  -5.656834045454091076e-01,
      7.047222019298797768e-01,  4.595684015346917506e-01,  8.754836314218116256e-01,  1.045569650441375842e+00,
      -2.492584485086628623e-02, -1.172228053410863557e+00, 4.150201978628382804e-01,  -1.455578925160157244e-01,
      6.211956446548982935e-01,  -6.724937073875727478e-01, 6.701335325890445116e-01,  -4.955442592943617441e-01,
      6.697002010538350980e-01,  -9.398253727980400429e-02, -3.796215232648539328e-02, 1.285564718411116858e-02,
      -9.231839799987423545e-01, -6.717594481874026302e-01, 5.426823179995654556e-01,  3.287650821213281560e-01,
      -2.476366890995689574e-01, -3.775027569377259473e-01, 2.602190943667161971e-01,  -1.483839193663842426e+00,
      -1.103835015051469082e+00, 1.072433966880660394e+00,  2.365344901971001335e+00,  7.509059237278949972e-01,
      -8.686455053166215745e-02, 1.215015555255819502e+00,  -7.631423070165320732e-01, 3.302367979421337862e-01,
      1.482018289804353017e+00,  1.267784022433528779e-01,  6.100638491175328637e-01,  4.607221842540662471e-01,
      -1.234605091663139476e+00, 1.470796653331652992e+00,  -3.080968100231850082e-01, 3.983002259174399651e-01,
      7.318976031388524373e-01,  -9.664301947397304637e-01, 6.412625703265225630e-01,  6.305591313252802144e-02,
      -7.499448654601594821e-01, 3.503062755001197237e-01,  -1.205790850772226364e+00, -1.190786371240603270e+00,
      -2.457277721395691727e-01, -1.933938717923781914e+00, 2.964322341547116935e+00,  9.248504038482635581e-01,
      9.731281131769021764e-01,  -7.287713877833996712e-01, 2.970185596152891683e-01,  5.519811945237956818e-01,
    };
    std::vector<double> expected_7 = {
      -2.824237745879321637e-01,
      -7.920420096839880797e-02,
      -1.028246489595118357e-01,
      -9.666806068015047770e-02,
      9.172045505502028628e-02,
      7.931677237130800584e-02,
      1.699504116244705820e-01,
    };
    std::vector<double> expected_10 = {
      -1.016878267503324995e-01,
      -4.473539673433823971e-01,
      -1.447445768882981987e-01,
      -4.993359489034446957e-02,
      4.546153451528033018e-02,
      -2.757976951947169186e-01,
      3.122701875792610604e-01,
      1.227569096111141150e-01,
      1.943711212598739357e-01,
      4.014138961563442493e-02,
    };

    {
        auto r = paa<double>(data, 7);
        assert(r.size() == expected_7.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected_7[i]);
            assert(diff < 1e-16);
        }
    }

    {
        auto r = paa<float>(data, 10);
        assert(r.size() == expected_10.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(static_cast<double>(r[i]) - expected_10[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_minmax_scale() {
    std::vector<double> data = {
      7.796343665038752221e-17,  9.286723986772055239e-02,  -1.803945870164925802e-01, 2.215397745670692753e-01,
      -1.644843685505689435e-01, -9.954496206758275956e-02, 1.779830276444839487e+00,  1.205446246857638792e+00,
      -4.050471169471980826e-01, 9.818729853215622805e-02,  7.140894438702208302e-02,  -1.446596846239995016e-01,
      1.418172055126633302e-01,  -8.899151382224200491e-02, 1.731216731610123530e-02,  4.428425307130670618e-02,
      -7.639302571278244747e-02, 7.328530097545010724e-02,  -4.245359536810088680e-02, -7.796343665038752221e-17,
    };
    std::vector<double> expected = {
      1.853866574720514082e-01, 2.278912118001747977e-01, 1.028215727848833111e-01, 2.867835483168631461e-01,
      1.101035459125483396e-01, 1.398257658775630685e-01, 1.000000000000000000e+00, 7.371092623666789523e-01,
      0.000000000000000000e+00, 2.303261578893813222e-01, 2.180699305028364898e-01, 1.191771369463186653e-01,
      2.502951992243604429e-01, 1.446559903456539342e-01, 1.933102907928319070e-01, 2.056551875073020508e-01,
      1.504222123531507860e-01, 2.189287231262133926e-01, 1.659560040649092072e-01, 1.853866574720513527e-01,
    };

    {
        auto copy = data;
        minmax_scale<double>(copy, 0.0, 1.0);
        assert(copy.size() == expected.size());
        for (size_t i = 0; i < copy.size(); ++i) {
            double diff = std::abs(copy[i] - expected[i]);
            assert(diff < EPS);
        }
    }

    {
        minmax_scale_ctx_t<float> ctx;
        minmax_scale<float>(ctx, data, 0.0f, 1.0f);
        auto r = ctx.result;
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(static_cast<double>(r[i]) - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_norm_cdf() {
    std::vector<double> data = {
      0.000000000000000000e+00, 3.225806451612903136e-02, 6.451612903225806273e-02, 9.677419354838709409e-02,
      1.290322580645161255e-01, 1.612903225806451568e-01, 1.935483870967741882e-01, 2.258064516129032195e-01,
      2.580645161290322509e-01, 2.903225806451612545e-01, 3.225806451612903136e-01, 3.548387096774193727e-01,
      3.870967741935483764e-01, 4.193548387096773800e-01, 4.516129032258064391e-01, 4.838709677419354982e-01,
      5.161290322580645018e-01, 5.483870967741935054e-01, 5.806451612903225090e-01, 6.129032258064516236e-01,
      6.451612903225806273e-01, 6.774193548387096309e-01, 7.096774193548387455e-01, 7.419354838709677491e-01,
      7.741935483870967527e-01, 8.064516129032257563e-01, 8.387096774193547599e-01, 8.709677419354838745e-01,
      9.032258064516128782e-01, 9.354838709677418818e-01, 9.677419354838709964e-01, 1.000000000000000000e+00,
    };
    std::vector<double> expected = {
      5.000000000000000000e-01, 5.128668742728588192e-01, 5.257203676221837707e-01, 5.385471408564991291e-01,
      5.513339380315980254e-01, 5.640676275336392598e-01, 5.767352425179979036e-01, 5.893240204964120821e-01,
      6.018214418711302915e-01, 6.142152672223537557e-01, 6.264935731642267136e-01, 6.386447865948615688e-01,
      6.506577171772994772e-01, 6.625215879008157227e-01, 6.742260635854371298e-01, 6.857612772068568674e-01,
      6.971178539339611957e-01, 7.082869327867966636e-01, 7.192601858388718572e-01, 7.300298349040595935e-01,
      7.405886656649046662e-01, 7.509300392157201021e-01, 7.610479010103134989e-01, 7.709367872204114791e-01,
      7.805918285267099410e-01, 7.900087513798303984e-01, 7.991838767832303292e-01, 8.081141166641422346e-01,
      8.167969679118535220e-01, 8.252305041749536940e-01, 8.334133655205246960e-01, 8.413447460685429258e-01,
    };

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_cdf<double>(data[i]);
            double diff = std::abs(r - expected[i]);
            assert(diff < 1e-15);
        }
    }

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_cdf<float>(data[i]);
            double diff = std::abs(static_cast<double>(r) - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_norm_pdf() {
    std::vector<double> data = {
      0.000000000000000000e+00, 3.225806451612903136e-02, 6.451612903225806273e-02, 9.677419354838709409e-02,
      1.290322580645161255e-01, 1.612903225806451568e-01, 1.935483870967741882e-01, 2.258064516129032195e-01,
      2.580645161290322509e-01, 2.903225806451612545e-01, 3.225806451612903136e-01, 3.548387096774193727e-01,
      3.870967741935483764e-01, 4.193548387096773800e-01, 4.516129032258064391e-01, 4.838709677419354982e-01,
      5.161290322580645018e-01, 5.483870967741935054e-01, 5.806451612903225090e-01, 6.129032258064516236e-01,
      6.451612903225806273e-01, 6.774193548387096309e-01, 7.096774193548387455e-01, 7.419354838709677491e-01,
      7.741935483870967527e-01, 8.064516129032257563e-01, 8.387096774193547599e-01, 8.709677419354838745e-01,
      9.032258064516128782e-01, 9.354838709677418818e-01, 9.677419354838709964e-01, 1.000000000000000000e+00,
    };
    std::vector<double> expected = {
      3.989422804014327029e-01, 3.987347681666320587e-01, 3.981128788701271959e-01, 3.970785513704122027e-01,
      3.956350059054691282e-01, 3.937867273889578690e-01, 3.915394421949670023e-01, 3.889000885995027579e-01,
      3.858767810922429375e-01, 3.824787688154548393e-01, 3.787163884279626802e-01, 3.746010117303019693e-01,
      3.701449884223635967e-01, 3.653615843966153687e-01, 3.602649159981285987e-01, 3.548698807069033401e-01,
      3.491920847182030108e-01, 3.432477679126343939e-01, 3.370537267194528286e-01, 3.306272353839858624e-01,
      3.239859661531530044e-01, 3.171479088918513489e-01, 3.101312906375712219e-01, 3.029544955911239179e-01,
      2.956359860279734186e-01, 2.881942245975774863e-01, 2.806475984575829696e-01, 2.730143456659755441e-01,
      2.653124842276272655e-01, 2.575597441624530304e-01, 2.497735029309033628e-01, 2.419707245191433653e-01,
    };

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_pdf<double>(data[i]);
            double diff = std::abs(r - expected[i]);
            assert(diff < 1e-15);
        }
    }

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_pdf<float>(data[i]);
            double diff = std::abs(static_cast<double>(r) - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_norm_ppf() {
    std::vector<double> data = {
      1.562500000000000000e-02, 3.125000000000000000e-02, 4.687500000000000000e-02, 6.250000000000000000e-02,
      7.812500000000000000e-02, 9.375000000000000000e-02, 1.093750000000000000e-01, 1.250000000000000000e-01,
      1.406250000000000000e-01, 1.562500000000000000e-01, 1.718750000000000000e-01, 1.875000000000000000e-01,
      2.031250000000000000e-01, 2.187500000000000000e-01, 2.343750000000000000e-01, 2.500000000000000000e-01,
      2.656250000000000000e-01, 2.812500000000000000e-01, 2.968750000000000000e-01, 3.125000000000000000e-01,
      3.281250000000000000e-01, 3.437500000000000000e-01, 3.593750000000000000e-01, 3.750000000000000000e-01,
      3.906250000000000000e-01, 4.062500000000000000e-01, 4.218750000000000000e-01, 4.375000000000000000e-01,
      4.531250000000000000e-01, 4.687500000000000000e-01, 4.843750000000000000e-01, 5.000000000000000000e-01,
      5.156250000000000000e-01, 5.312500000000000000e-01, 5.468750000000000000e-01, 5.625000000000000000e-01,
      5.781250000000000000e-01, 5.937500000000000000e-01, 6.093750000000000000e-01, 6.250000000000000000e-01,
      6.406250000000000000e-01, 6.562500000000000000e-01, 6.718750000000000000e-01, 6.875000000000000000e-01,
      7.031250000000000000e-01, 7.187500000000000000e-01, 7.343750000000000000e-01, 7.500000000000000000e-01,
      7.656250000000000000e-01, 7.812500000000000000e-01, 7.968750000000000000e-01, 8.125000000000000000e-01,
      8.281250000000000000e-01, 8.437500000000000000e-01, 8.593750000000000000e-01, 8.750000000000000000e-01,
      8.906250000000000000e-01, 9.062500000000000000e-01, 9.218750000000000000e-01, 9.375000000000000000e-01,
      9.531250000000000000e-01, 9.687500000000000000e-01, 9.843750000000000000e-01,
    };
    std::vector<double> expected = {
      -2.153874694061456374e+00, -1.862731867421651533e+00, -1.675939722773443830e+00, -1.534120544352546300e+00,
      -1.417797137996267054e+00, -1.318010897303536932e+00, -1.229858759216589048e+00, -1.150349380376007868e+00,
      -1.077515567040280287e+00, -1.009990169249582070e+00, -9.467817563010456627e-01, -8.871465590188759576e-01,
      -8.305108782053992611e-01, -7.764217611479277137e-01, -7.245143834923654103e-01, -6.744897501960817054e-01,
      -6.260990123464211798e-01, -5.791321622555559712e-01, -5.334097062412805901e-01, -4.887764111146695178e-01,
      -4.450965249855163841e-01, -4.022500653217253586e-01, -3.601298917895694451e-01, -3.186393639643751441e-01,
      -2.776904398215767622e-01, -2.372021093287876858e-01, -1.970990842943123045e-01, -1.573106846101706979e-01,
      -1.177698745790952961e-01, -7.841241273311219673e-02, -3.917608550309762544e-02, 0.000000000000000000e+00,
      3.917608550309762544e-02,  7.841241273311219673e-02,  1.177698745790952961e-01,  1.573106846101706979e-01,
      1.970990842943123045e-01,  2.372021093287876858e-01,  2.776904398215767622e-01,  3.186393639643751441e-01,
      3.601298917895694451e-01,  4.022500653217253586e-01,  4.450965249855163841e-01,  4.887764111146695178e-01,
      5.334097062412805901e-01,  5.791321622555559712e-01,  6.260990123464211798e-01,  6.744897501960817054e-01,
      7.245143834923654103e-01,  7.764217611479277137e-01,  8.305108782053992611e-01,  8.871465590188759576e-01,
      9.467817563010456627e-01,  1.009990169249582070e+00,  1.077515567040280287e+00,  1.150349380376007868e+00,
      1.229858759216589048e+00,  1.318010897303536932e+00,  1.417797137996267054e+00,  1.534120544352546300e+00,
      1.675939722773443830e+00,  1.862731867421651533e+00,  2.153874694061456374e+00,
    };

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_ppf<double>(data[i]);
            double diff = std::abs(r - expected[i]);
            assert(diff < 1e-14);
        }
    }

    {
        assert(data.size() == expected.size());
        for (size_t i = 0; i < data.size(); ++i) {
            auto   r    = norm_ppf<float>(data[i]);
            double diff = std::abs(static_cast<double>(r) - expected[i]);
            assert(diff < 1e-5);
        }
    }
}

void test_mtf() {
    std::vector<double> data = {
      0.000000000000000000e+00,  6.142127126896678169e-01,  9.694002659393303745e-01,  9.157733266550575069e-01,
      4.759473930370736738e-01,  -1.645945902807337824e-01, -7.357239106731312539e-01, -9.965844930066698470e-01,
      -8.371664782625287682e-01, -3.246994692046837327e-01, 3.246994692046832887e-01,  8.371664782625285461e-01,
      9.965844930066699581e-01,  7.357239106731322531e-01,  1.645945902807346983e-01,  -4.759473930370728967e-01,
      -9.157733266550571738e-01, -9.694002659393305965e-01, -6.142127126896682610e-01, -4.898587196589412829e-16,
    };
    std::vector<double> expected = {
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
      5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      5.000000000000000000e-01, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      0.000000000000000000e+00, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      5.000000000000000000e-01, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      0.000000000000000000e+00, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
      5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      5.000000000000000000e-01, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      0.000000000000000000e+00, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      5.000000000000000000e-01, 5.000000000000000000e-01, 0.000000000000000000e+00, 5.000000000000000000e-01,
      0.000000000000000000e+00, 0.000000000000000000e+00, 5.000000000000000000e-01, 5.000000000000000000e-01,
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 4.000000000000000222e-01, 5.999999999999999778e-01, 5.999999999999999778e-01,
      5.999999999999999778e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01,
      5.999999999999999778e-01, 5.999999999999999778e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
      2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 4.000000000000000222e-01,
      4.000000000000000222e-01, 2.000000000000000111e-01, 4.000000000000000222e-01, 0.000000000000000000e+00,
      0.000000000000000000e+00, 0.000000000000000000e+00, 4.000000000000000222e-01, 2.000000000000000111e-01,
      4.000000000000000222e-01, 4.000000000000000222e-01, 2.000000000000000111e-01, 2.000000000000000111e-01,
    };

    {
        auto r = mtf<double>(data, 4);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < EPS);
        }
    }

    {
        auto r = mtf<float>(data, 4);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); ++i) {
            double diff = std::abs(static_cast<double>(r[i]) - expected[i]);
            assert(diff < 1e-7);
        }
    }
}

void test_resize() {
    std::vector<double> data = {

      -1.000000000000000000e+01, -9.797979797979797567e+00, -9.595959595959595134e+00, -9.393939393939394478e+00,
      -9.191919191919192045e+00, -8.989898989898989612e+00, -8.787878787878787179e+00, -8.585858585858586522e+00,
      -8.383838383838384090e+00, -8.181818181818181657e+00, -7.979797979797979224e+00, -7.777777777777777679e+00,
      -7.575757575757576134e+00, -7.373737373737373701e+00, -7.171717171717171269e+00, -6.969696969696969724e+00,
      -6.767676767676768179e+00, -6.565656565656565746e+00, -6.363636363636363313e+00, -6.161616161616161769e+00,
      -5.959595959595959336e+00, -5.757575757575757791e+00, -5.555555555555555358e+00, -5.353535353535353813e+00,
      -5.151515151515151381e+00, -4.949494949494949836e+00, -4.747474747474747403e+00, -4.545454545454545858e+00,
      -4.343434343434343425e+00, -4.141414141414141881e+00, -3.939393939393939448e+00, -3.737373737373737903e+00,
      -3.535353535353535470e+00, -3.333333333333333037e+00, -3.131313131313131493e+00, -2.929292929292929060e+00,
      -2.727272727272727515e+00, -2.525252525252525082e+00, -2.323232323232323537e+00, -2.121212121212121104e+00,
      -1.919191919191918672e+00, -1.717171717171718015e+00, -1.515151515151515582e+00, -1.313131313131313149e+00,
      -1.111111111111110716e+00, -9.090909090909100598e-01, -7.070707070707076269e-01, -5.050505050505051940e-01,
      -3.030303030303027612e-01, -1.010101010101003283e-01, 1.010101010101003283e-01,  3.030303030303027612e-01,
      5.050505050505051940e-01,  7.070707070707076269e-01,  9.090909090909082835e-01,  1.111111111111110716e+00,
      1.313131313131313149e+00,  1.515151515151515582e+00,  1.717171717171716239e+00,  1.919191919191918672e+00,
      2.121212121212121104e+00,  2.323232323232323537e+00,  2.525252525252524194e+00,  2.727272727272726627e+00,
      2.929292929292929060e+00,  3.131313131313131493e+00,  3.333333333333333925e+00,  3.535353535353534582e+00,
      3.737373737373737015e+00,  3.939393939393939448e+00,  4.141414141414141881e+00,  4.343434343434342537e+00,
      4.545454545454544970e+00,  4.747474747474747403e+00,  4.949494949494949836e+00,  5.151515151515150492e+00,
      5.353535353535352925e+00,  5.555555555555555358e+00,  5.757575757575757791e+00,  5.959595959595958448e+00,
      6.161616161616162657e+00,  6.363636363636363313e+00,  6.565656565656563970e+00,  6.767676767676768179e+00,
      6.969696969696968836e+00,  7.171717171717173045e+00,  7.373737373737373701e+00,  7.575757575757574358e+00,
      7.777777777777778567e+00,  7.979797979797979224e+00,  8.181818181818179880e+00,  8.383838383838384090e+00,
      8.585858585858584746e+00,  8.787878787878788955e+00,  8.989898989898989612e+00,  9.191919191919190268e+00,
      9.393939393939394478e+00,  9.595959595959595134e+00,  9.797979797979799343e+00,  1.000000000000000000e+01,
    };
    std::vector<int16_t> data_shape = {10, 10};
    std::vector<double>  expected   = {
         -7.491582491582491343e+00,
         -6.986531986531986149e+00,
         -6.481481481481480955e+00,
         -5.976430976430976649e+00,
         -7.575757575757580131e-01,
         -2.525252525252523750e-01,
         2.525252525252521529e-01,
         7.575757575757573470e-01,
         5.976430976430977537e+00,
         6.481481481481482732e+00,
         6.986531986531987926e+00,
         7.491582491582493120e+00,
    };
    std::vector<int16_t> expected_shape = {3, 4};

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
            double diff = std::abs(static_cast<double>(r[i]) - expected[i]);
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
        auto   r    = psnr<double>(original, compressed);
        double diff = std::abs(r - expected);
        assert(diff < EPS);
    }

    {
        auto   r    = psnr<float>(original, compressed);
        double diff = std::abs(static_cast<double>(r) - expected);
        assert(diff < 1e-6);
    }
}

void test_integrate_wavelet() {
    std::vector<double> expected_psi = {
      -5.299585269807807314e-16, -1.059810177808842127e-15, -1.280250887992044579e-15, -5.428809827700196712e-16,
      2.330234996573825912e-15,  9.251017955057641048e-15,  2.299680093286884090e-14,  4.708854538158249716e-14,
      8.509750893481118414e-14,  1.388835213815836833e-13,  2.050801191432388192e-13,  2.690226453346947958e-13,
      2.954166316385676802e-13,  2.156164722626999692e-13,  -8.720693956802289556e-14, -7.910288358917377850e-13,
      -2.137624468399588327e-12, -4.409992075990127377e-12, -7.861396559256033958e-12, -1.256429843066230856e-11,
      -1.814169863811438856e-11, -2.334641231165085407e-11, -2.547465607375205323e-11, -1.965163056678819988e-11,
      1.876843276857092147e-12,  5.015781204009137781e-11,  1.391425741067994828e-10,  2.837202409613411524e-10,
      4.951264395760269101e-10,  7.725045634664899011e-10,  1.089483552502495088e-09,  1.375227069152863441e-09,
      1.490810504505079042e-09,  1.204318883228841214e-09,  1.719758352402246139e-10,  -2.062083568782882267e-09,
      -6.028559014201671830e-09, -1.223347786688213298e-08, -2.096825074298846566e-08, -3.200365555958080799e-08,
      -4.415521012761031919e-08, -5.473627373389202903e-08, -5.896467589444757976e-08, -4.946115308212549665e-08,
      -1.607077704966431159e-08, 5.365909645951067063e-08,  1.729217703418355556e-07,  3.525535878919496807e-07,
      5.959967458183003601e-07,  8.921483966798099125e-07,  1.206370023060411025e-06,  1.470645303361859528e-06,
      1.574854695581840942e-06,  1.362306537286575134e-06,  6.338485188646725130e-07,  -8.342267918619145542e-07,
      -3.253110110218763907e-06, -6.760973543504742273e-06, -1.133770678626189384e-05, -1.669875760577256218e-05,
      -2.217954974995390557e-05, -2.663158803536885491e-05, -2.836190748703862813e-05, -2.515696671828773418e-05,
      -1.443750087145424908e-05, 6.411390450471171412e-06,  3.950436930670965708e-05,  8.571206765031058735e-05,
      1.437518415749337235e-04,  2.092147985528628525e-04,  2.736990677335376064e-04,  3.242858666784447798e-04,
      3.436442752362048043e-04,  3.110634713267177126e-04,  2.046671532933091824e-04,  4.946654231401560635e-06,
      -3.004521470401397645e-04, -7.110325647720922709e-04, -1.207519612108992179e-03, -1.746721360205802159e-03,
      -2.258482825273337539e-03, -2.646179674937754363e-03, -2.792117763057256480e-03, -2.568826817115647896e-03,
      -1.856524570860210903e-03, -5.659971470577141459e-04, 1.335121739765615396e-03,  3.796023943595913577e-03,
      6.660876304238467355e-03,  9.656705108845186139e-03,  1.239629076633310192e-02,  1.440041616393652329e-02,
      1.514181523925779878e-02,  1.411015784388946863e-02,  1.089364197830548081e-02,  5.268718478344164985e-03,
      -2.714199451694800092e-03, -1.266373241647520374e-02, -2.381460089751299969e-02, -3.504235180668296284e-02,
      -4.493484690186115571e-02, -5.192257209513118782e-02, -5.446088968672301717e-02, -5.124766183243524564e-02,
      -4.145072871527503977e-02, -2.491319824529852900e-02, -2.301954629701886273e-03, 2.483263401292950048e-02,
      5.410968164463076518e-02,  8.249401314315282785e-02,  1.065892172919034042e-01,  1.230225552188961569e-01,
      1.288795422153519998e-01,  1.221331013084644007e-01,  1.020061949871194917e-01,  6.920956548964937727e-02,
      2.600818878163070960e-02,  -2.390967425377755681e-02, -7.576077914545065184e-02, -1.241640536380923948e-01,
      -1.637516680270434954e-01, -1.898189359281931832e-01, -1.989276853273680146e-01, -1.893795306706036741e-01,
      -1.614897112431692427e-01, -1.176172345820605952e-01, -6.193956709501882013e-02, 4.670667106284658046e-06,
      6.194890842923139046e-02,  1.176265759162731656e-01,  1.614990525773820074e-01,  1.893888720048166885e-01,
      1.989370266615813065e-01,  1.898282772624064751e-01,  1.637610093612567874e-01,  1.241733949723056868e-01,
      7.577012047966394381e-02,  2.391901558799084532e-02,  -2.599884744741748355e-02, -6.920022415543627958e-02,
      -1.019968536529062275e-01, -1.221237599742511365e-01, -1.288702008811387356e-01, -1.230132138846828926e-01,
      -1.065798759576901400e-01, -8.248467180893956363e-02, -5.410034031041754260e-02, -2.482329267871627443e-02,
      2.311295963915159166e-03,  2.492253957951180363e-02,  4.146007004948831787e-02,  5.125700316664852374e-02,
      5.447023102093628832e-02,  5.193191342934446592e-02,  4.494418823607448932e-02,  3.505169314089633115e-02,
      2.382394223172636799e-02,  1.267307375068857378e-02,  2.723540785908169694e-03,  -5.259377144130795816e-03,
      -1.088430064409211077e-02, -1.410081650967609859e-02, -1.513247390504445129e-02, -1.439107482972319316e-02,
      -1.238694943211978219e-02, -9.647363774631864675e-03, -6.651534970025145890e-03, -3.786682609382592112e-03,
      -1.325780405552293498e-03, 5.753384812710359356e-04,  1.865865905073542125e-03,  2.578168151328987575e-03,
      2.801459097270591388e-03,  2.655521009151089271e-03,  2.267824159486672448e-03,  1.756062694419137284e-03,
      1.216860946322327088e-03,  7.203738989854271794e-04,  3.097934812534734262e-04,  4.394679981930761889e-06,
      -1.953258190799780144e-04, -3.017221371133865988e-04, -3.343029410228736905e-04, -3.149445324651136660e-04,
      -2.643577335202064926e-04, -1.998734643395317116e-04, -1.344105073616025012e-04, -7.637073343697918894e-05,
      -3.016303509337845518e-05, 2.929943762860032181e-06,  2.377883508478545268e-05,  3.449830093161893946e-05,
      3.770324170036983003e-05,  3.597292224870005681e-05,  3.152088396328512780e-05,  2.604009181910378441e-05,
      2.067904099959311099e-05,  1.610230775683595857e-05,  1.259444432354998190e-05,  1.017556100519313149e-05,
      8.707485694466544950e-06,  7.979027676044640529e-06,  7.766479517749365192e-06,  7.870688909969341312e-06,
      8.134964190270792145e-06,  8.449185816651392622e-06,  8.745337467512903445e-06,  8.988780625439254918e-06,
      9.168412442989369414e-06,  9.287675116871694285e-06,  9.357404990380870135e-06,  9.390795366413330340e-06,
      9.400298889225653932e-06,  9.396070487065097773e-06,  9.385489423458814488e-06,  9.373337868890784996e-06,
      9.362302464074192214e-06,  9.353567691198086609e-06,  9.347362772345404493e-06,  9.343396296899986368e-06,
      9.341162237495965475e-06,  9.340129894447976209e-06,  9.339843402826700371e-06,  9.339958986262052114e-06,
      9.340244729778704387e-06,  9.340561708767740914e-06,  9.340839086891631888e-06,  9.341050493090246389e-06,
      9.341195070757099791e-06,  9.341284055519166933e-06,  9.341332336487930172e-06,  9.341353864961773274e-06,
      9.341359687987281301e-06,  9.341357559743519522e-06,  9.341352355029845317e-06,  9.341346777629638124e-06,
      9.341342074727765395e-06,  9.341338623323282680e-06,  9.341336350955675691e-06,  9.341335004360043381e-06,
      9.341334300538146445e-06,  9.341333997714735093e-06,  9.341333917914576980e-06,  9.341333944308562379e-06,
      9.341334008251088038e-06,  9.341334074447685260e-06,  9.341334128233698040e-06,  9.341334166242661692e-06,
      9.341334190334406459e-06,  9.341334204080189264e-06,  9.341334211000971909e-06,  9.341334213874087338e-06,
      9.341334214611456623e-06,  9.341334214391016299e-06,  9.341334213861165003e-06,  9.341334213331206981e-06,
    };
    std::vector<double> expected_x = {
      -8.000000000000000000e+00, -7.937254901960784537e+00, -7.874509803921569073e+00, -7.811764705882352722e+00,
      -7.749019607843137258e+00, -7.686274509803921795e+00, -7.623529411764705443e+00, -7.560784313725489980e+00,
      -7.498039215686274517e+00, -7.435294117647059053e+00, -7.372549019607843590e+00, -7.309803921568627239e+00,
      -7.247058823529411775e+00, -7.184313725490196312e+00, -7.121568627450979960e+00, -7.058823529411764497e+00,
      -6.996078431372549034e+00, -6.933333333333333570e+00, -6.870588235294118107e+00, -6.807843137254901755e+00,
      -6.745098039215686292e+00, -6.682352941176470829e+00, -6.619607843137254477e+00, -6.556862745098039014e+00,
      -6.494117647058823550e+00, -6.431372549019608087e+00, -6.368627450980392624e+00, -6.305882352941176272e+00,
      -6.243137254901960809e+00, -6.180392156862745345e+00, -6.117647058823528994e+00, -6.054901960784313530e+00,
      -5.992156862745098067e+00, -5.929411764705882604e+00, -5.866666666666667140e+00, -5.803921568627450789e+00,
      -5.741176470588235325e+00, -5.678431372549019862e+00, -5.615686274509803511e+00, -5.552941176470588047e+00,
      -5.490196078431372584e+00, -5.427450980392157120e+00, -5.364705882352941657e+00, -5.301960784313725306e+00,
      -5.239215686274509842e+00, -5.176470588235294379e+00, -5.113725490196078027e+00, -5.050980392156862564e+00,
      -4.988235294117647101e+00, -4.925490196078431637e+00, -4.862745098039216174e+00, -4.799999999999999822e+00,
      -4.737254901960784359e+00, -4.674509803921568896e+00, -4.611764705882352544e+00, -4.549019607843137081e+00,
      -4.486274509803921617e+00, -4.423529411764706154e+00, -4.360784313725490691e+00, -4.298039215686274339e+00,
      -4.235294117647058876e+00, -4.172549019607843412e+00, -4.109803921568627061e+00, -4.047058823529411598e+00,
      -3.984313725490196134e+00, -3.921568627450980671e+00, -3.858823529411765207e+00, -3.796078431372548856e+00,
      -3.733333333333333393e+00, -3.670588235294117929e+00, -3.607843137254901578e+00, -3.545098039215686114e+00,
      -3.482352941176470651e+00, -3.419607843137255188e+00, -3.356862745098039724e+00, -3.294117647058823373e+00,
      -3.231372549019607909e+00, -3.168627450980392446e+00, -3.105882352941176094e+00, -3.043137254901960631e+00,
      -2.980392156862745168e+00, -2.917647058823529704e+00, -2.854901960784314241e+00, -2.792156862745097889e+00,
      -2.729411764705882426e+00, -2.666666666666666963e+00, -2.603921568627450611e+00, -2.541176470588235148e+00,
      -2.478431372549019684e+00, -2.415686274509804221e+00, -2.352941176470588758e+00, -2.290196078431372406e+00,
      -2.227450980392156943e+00, -2.164705882352941479e+00, -2.101960784313725128e+00, -2.039215686274509665e+00,
      -1.976470588235294201e+00, -1.913725490196078738e+00, -1.850980392156863275e+00, -1.788235294117646923e+00,
      -1.725490196078431460e+00, -1.662745098039215996e+00, -1.599999999999999645e+00, -1.537254901960784181e+00,
      -1.474509803921568718e+00, -1.411764705882353255e+00, -1.349019607843137791e+00, -1.286274509803921440e+00,
      -1.223529411764705976e+00, -1.160784313725490513e+00, -1.098039215686274161e+00, -1.035294117647058698e+00,
      -9.725490196078432348e-01, -9.098039215686277714e-01, -8.470588235294123081e-01, -7.843137254901959565e-01,
      -7.215686274509804932e-01, -6.588235294117650298e-01, -5.960784313725486783e-01, -5.333333333333332149e-01,
      -4.705882352941177516e-01, -4.078431372549022882e-01, -3.450980392156868248e-01, -2.823529411764704733e-01,
      -2.196078431372550099e-01, -1.568627450980395466e-01, -9.411764705882319504e-02, -3.137254901960773168e-02,
      3.137254901960773168e-02,  9.411764705882319504e-02,  1.568627450980386584e-01,  2.196078431372541218e-01,
      2.823529411764695851e-01,  3.450980392156868248e-01,  4.078431372549022882e-01,  4.705882352941177516e-01,
      5.333333333333332149e-01,  5.960784313725486783e-01,  6.588235294117641416e-01,  7.215686274509796050e-01,
      7.843137254901968447e-01,  8.470588235294123081e-01,  9.098039215686277714e-01,  9.725490196078432348e-01,
      1.035294117647058698e+00,  1.098039215686274161e+00,  1.160784313725489625e+00,  1.223529411764705088e+00,
      1.286274509803920552e+00,  1.349019607843137791e+00,  1.411764705882353255e+00,  1.474509803921568718e+00,
      1.537254901960784181e+00,  1.599999999999999645e+00,  1.662745098039215108e+00,  1.725490196078430571e+00,
      1.788235294117647811e+00,  1.850980392156863275e+00,  1.913725490196078738e+00,  1.976470588235294201e+00,
      2.039215686274509665e+00,  2.101960784313725128e+00,  2.164705882352940591e+00,  2.227450980392156055e+00,
      2.290196078431371518e+00,  2.352941176470588758e+00,  2.415686274509804221e+00,  2.478431372549019684e+00,
      2.541176470588235148e+00,  2.603921568627450611e+00,  2.666666666666666075e+00,  2.729411764705881538e+00,
      2.792156862745098778e+00,  2.854901960784314241e+00,  2.917647058823529704e+00,  2.980392156862745168e+00,
      3.043137254901960631e+00,  3.105882352941176094e+00,  3.168627450980391558e+00,  3.231372549019607021e+00,
      3.294117647058822484e+00,  3.356862745098039724e+00,  3.419607843137255188e+00,  3.482352941176470651e+00,
      3.545098039215686114e+00,  3.607843137254901578e+00,  3.670588235294117041e+00,  3.733333333333332504e+00,
      3.796078431372549744e+00,  3.858823529411765207e+00,  3.921568627450980671e+00,  3.984313725490196134e+00,
      4.047058823529411598e+00,  4.109803921568627061e+00,  4.172549019607842524e+00,  4.235294117647057988e+00,
      4.298039215686273451e+00,  4.360784313725490691e+00,  4.423529411764706154e+00,  4.486274509803921617e+00,
      4.549019607843137081e+00,  4.611764705882352544e+00,  4.674509803921568007e+00,  4.737254901960783471e+00,
      4.800000000000000711e+00,  4.862745098039216174e+00,  4.925490196078431637e+00,  4.988235294117647101e+00,
      5.050980392156862564e+00,  5.113725490196078027e+00,  5.176470588235293491e+00,  5.239215686274508954e+00,
      5.301960784313724417e+00,  5.364705882352941657e+00,  5.427450980392157120e+00,  5.490196078431372584e+00,
      5.552941176470588047e+00,  5.615686274509803511e+00,  5.678431372549018974e+00,  5.741176470588234437e+00,
      5.803921568627451677e+00,  5.866666666666667140e+00,  5.929411764705882604e+00,  5.992156862745098067e+00,
      6.054901960784313530e+00,  6.117647058823528994e+00,  6.180392156862744457e+00,  6.243137254901959921e+00,
      6.305882352941175384e+00,  6.368627450980392624e+00,  6.431372549019608087e+00,  6.494117647058823550e+00,
      6.556862745098039014e+00,  6.619607843137254477e+00,  6.682352941176469940e+00,  6.745098039215685404e+00,
      6.807843137254902643e+00,  6.870588235294118107e+00,  6.933333333333333570e+00,  6.996078431372549034e+00,
      7.058823529411764497e+00,  7.121568627450979960e+00,  7.184313725490195424e+00,  7.247058823529410887e+00,
      7.309803921568626350e+00,  7.372549019607843590e+00,  7.435294117647059053e+00,  7.498039215686274517e+00,
      7.560784313725489980e+00,  7.623529411764705443e+00,  7.686274509803920907e+00,  7.749019607843136370e+00,
      7.811764705882353610e+00,  7.874509803921569073e+00,  7.937254901960784537e+00,  8.000000000000000000e+00,
    };

    {
        auto [psi, x] = integrate_wavelet<double>(cwt_wavelet_t::MORLET, 8);
        assert(psi.size() == expected_psi.size());
        assert(x.size() == expected_x.size());
        for (size_t i = 0; i < psi.size(); ++i) {
            double diff = std::abs(psi[i] - expected_psi[i]);
            assert(diff < 1e-15);
        }
        for (size_t i = 0; i < x.size(); ++i) {
            double diff = std::abs(static_cast<double>(x[i]) - expected_x[i]);
            assert(diff < 1e-14);
        }
    }

    {
        auto [psi, x] = integrate_wavelet<float>(cwt_wavelet_t::MORLET, 8);
        assert(psi.size() == expected_psi.size());
        assert(x.size() == expected_x.size());
        for (size_t i = 0; i < psi.size(); ++i) {
            double diff = std::abs(static_cast<double>(psi[i]) - expected_psi[i]);
            assert(diff < 1e-6);
        }
        for (size_t i = 0; i < x.size(); ++i) {
            double diff = std::abs(static_cast<double>(x[i]) - expected_x[i]);
            assert(diff < 1e-6);
        }
    }
}

void test_cwt() {
    std::vector<double> data = {
      1.000000339986812925e+00,  1.736480053287412784e-01,  -9.396943473846959272e-01, -5.000051575124934145e-01,
      7.660330944286311006e-01,  7.660238650690985862e-01,  -5.000311720976511332e-01, -9.397299706428607902e-01,
      1.736216172453500517e-01,  1.000022553850015594e+00,  1.737874172170271458e-01,  -9.393394040905544973e-01,
      -4.993231972005436892e-01, 7.671186881754112896e-01,  7.674654027119431898e-01,  -4.985355578732070247e-01,
      -9.388821583872918319e-01, 1.726167126691110221e-01,  9.954920904148363281e-01,  1.638803316049711467e-01,
      -9.560106073714963459e-01, -5.226739374561988338e-01, 7.398790480092543032e-01,  7.429440496676487093e-01,
      -5.094744318355105550e-01, -9.219826906908977771e-01, 2.323228245864276120e-01,  1.109100545120023540e+00,
      3.327164837018257160e-01,  -7.462045914338762964e-01, -3.054190863942370382e-01, 9.124645601016637686e-01,
      8.067836687551558361e-01,  -6.175787686637238538e-01, -1.247066359488790077e+00, -3.186419911897016011e-01,
      3.730515595260813777e-01,  -4.938615196479253511e-01, -1.523915148254922247e+00, -8.723248929904801052e-01,
      7.085076210492048387e-01,  1.072474145584599192e+00,  1.486337036468881045e-01,  -4.213585328184499268e-02,
      1.172782059771360741e+00,  1.930900213486871131e+00,  8.816184377428425423e-01,  -5.606835383877424306e-01,
      -4.865626418918085894e-01, 4.498068920611914545e-01,  2.142867755166433241e-01,  -1.161327805832036653e+00,
      -1.583515307981602671e+00, -3.510973404470357218e-01, 6.538743316410082773e-01,  1.966015613031465992e-02,
      -9.268411268743367293e-01, -3.700517051526182599e-01, 9.553466753987214233e-01,  9.629882632107671370e-01,
      -3.322505090073503142e-01, -8.201442597445423033e-01, 2.418852009040726170e-01,  1.024863385846330743e+00,
    };
    std::vector<int> scales = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    };
    std::vector<double> expected = {
      -1.076156379656637851e-01, -2.605430752535708439e-01, 1.571196802293376993e-01,  3.008582844156415526e-01,
      -5.553283623356639054e-02, -3.198573314184935401e-01, -5.554810139455701457e-02, 3.005675569206666675e-01,
      1.599404993046590895e-01,  -2.450071410878400524e-01, -2.450072726026717163e-01, 1.599496696088877068e-01,
      3.005928236707823276e-01,  -5.553287248432439704e-02, -3.199005990154500934e-01, -5.567451717219062957e-02,
      3.003260090051703135e-01,  1.595745536093102768e-01,  -2.454569435960985146e-01, -2.454231680392203185e-01,
      1.597772462712993835e-01,  3.009495025778660127e-01,  -5.434744969759383593e-02, -3.176866140479779332e-01,
      -5.248265984033706477e-02, 3.040483167182718161e-01,  1.629057665756528106e-01,  -2.438498957715058424e-01,
      -2.470325236120803347e-01, 1.537381796369042819e-01,  2.900891347086071992e-01,  -6.910887408599641901e-02,
      -3.338791856881936360e-01, -6.627863632731689769e-02, 2.971349148602809298e-01,  1.668932524731738809e-01,
      -2.267085850972798411e-01, -2.173423459933979118e-01, 1.920183170941296980e-01,  3.300181999050571524e-01,
      -3.617526637794180044e-02, -3.163419285197205388e-01, -7.015192746705710414e-02, 2.699854595548604741e-01,
      1.196166516092393994e-01,  -2.862738194941565339e-01, -2.781681144867066413e-01, 1.418050252118814580e-01,
      3.005030701250036018e-01,  -3.897519328089948942e-02, -2.918523562725188203e-01, -2.337292139540650404e-02,
      3.296224509088608312e-01,  1.803875270671104758e-01,  -2.357730182685037756e-01, -2.465692360227703384e-01,
      1.503485399063912153e-01,  2.868734794527096343e-01,  -6.946128347904231837e-02, -3.310961665410804278e-01,
      -6.261518570832441677e-02, 2.980533370643457802e-01,  1.577518316438456525e-01,  -2.571113766910629317e-01,
      -6.823143821518336261e-02, 3.535274163597732500e-01,  -1.903735179033879399e-01, -1.131602402311856492e-01,
      3.058608330169115125e-02,  1.378570745361978045e-01,  2.561145404534874437e-02,  -1.334634320727948742e-01,
      -7.093629262273327962e-02, 1.087097341916494070e-01,  1.086933354875476154e-01,  -7.096127575065415416e-02,
      -1.333406010903072925e-01, 2.464889173881680898e-02,  1.418964818054416488e-01,  2.462648997306957424e-02,
      -1.333474267269414981e-01, -7.093833312682250314e-02, 1.087148642828156681e-01,  1.087051302968629340e-01,
      -7.094449898572656454e-02, -1.333210499904519675e-01, 2.466645083831970828e-02,  1.419053099405282181e-01,
      2.461931994673578067e-02,  -1.333758471976119186e-01, -7.098811605879884468e-02, 1.086513824118921362e-01,
      1.086441154557130501e-01,  -7.098079872771345666e-02, -1.333102416318660555e-01, 2.473851021142142032e-02,
      1.420362245711967453e-01,  2.478540973103205217e-02,  -1.332177109217724253e-01, -7.089092504134456019e-02,
      1.086403992102379784e-01,  1.085006317908646795e-01,  -7.124556742378825025e-02, -1.336466960245822899e-01,
      2.440903261856977244e-02,  1.418017554196316643e-01,  2.471885834564469669e-02,  -1.330803613434022037e-01,
      -7.056295418819964749e-02, 1.090973756433906011e-01,  1.089905874468634628e-01,  -7.083008987852526528e-02,
      -1.333964751118613679e-01, 2.442058240524896603e-02,  1.415891733636368832e-01,  2.433866099436311770e-02,
      -1.335584810831271785e-01, -7.104904578670510895e-02, 1.087153914767614304e-01,  1.088063015649613080e-01,
      -7.079353255675036038e-02, -1.333091382633651567e-01, 2.571721274799774332e-02,  1.378892419735902319e-01,
      3.068681296894113994e-02,  -1.134254704907689398e-01, -1.906287020626867279e-01, 3.550027957824627434e-01,
      3.132437948104131453e-01,  1.127169820665900213e+00,  -3.448696127945110601e-01, -1.375894634155801821e+00,
      1.253549241850782092e-01,  1.450919705767415691e+00,  2.761892112039513458e-01,  -1.352336612504464686e+00,
      -7.208802417330612000e-01, 1.099289889202480719e+00,  1.098926546188231601e+00,  -7.169626341133648051e-01,
      -1.347553682127654806e+00, 2.488832038142758940e-01,  1.433958056576689843e+00,  2.490860248079360406e-01,
      -1.347532502825535827e+00, -7.171906280640752973e-01, 1.098341633725597788e+00,  1.098577514640833863e+00,
      -7.167514678788799998e-01, -1.347249207589195086e+00, 2.493526706814023219e-01,  1.434571063233999411e+00,
      2.497076623783223703e-01,  -1.347107856045638652e+00, -7.172047548015741958e-01, 1.097677144658633086e+00,
      1.097164643877527324e+00,  -7.188243434365493290e-01, -1.349674637491107054e+00, 2.470717236705259334e-01,
      1.433025892737331164e+00,  2.494421508256276510e-01,  -1.345746927182238872e+00, -7.141810184719613597e-01,
      1.102047188879602180e+00,  1.102252444932566577e+00,  -7.138478423862985034e-01, -1.345686981402133942e+00,
      2.493046346115352363e-01,  1.432957781416869691e+00,  2.469109436000337388e-01,  -1.350530703615347106e+00,
      -7.206629198685005067e-01, 1.094734799617215870e+00,  1.095200429637067208e+00,  -7.195205072055125273e-01,
      -1.349069978879927634e+00, 2.487418631056716456e-01,  1.435325690195917714e+00,  2.518221896233034940e-01,
      -1.343901130397258337e+00, -7.134160578679805687e-01, 1.102017031638097722e+00,  1.101615696753093898e+00,
      -7.198118711887909749e-01, -1.353513683502497456e+00, 2.750057493042513124e-01,  1.449381967866932630e+00,
      1.221957445326948971e-01,  -1.377711828832558716e+00, -3.427658708371437157e-01, 1.127258810548665613e+00,
      1.150778505010539243e+00,  7.211756536716331656e-01,  -1.092898878728561174e+00, -1.463257015014054252e+00,
      5.401318049695984502e-01,  1.863272253416391688e+00,  2.276952638125384620e-01,  -1.840264512420301646e+00,
      -9.425027669825837284e-01, 1.507027736511684424e+00,  1.490090069545874529e+00,  -9.798774443859313843e-01,
      -1.834012736840517022e+00, 3.396055095051805361e-01,  1.951814441952952395e+00,  3.387566763717443363e-01,
      -1.834187527607261892e+00, -9.760792790637838889e-01, 1.494881841063582018e+00,  1.495068585147699469e+00,
      -9.755345707796859722e-01, -1.833293340559175366e+00, 3.399360595107828731e-01,  1.952814060547697128e+00,
      3.396532827282109368e-01,  -1.834228094887012883e+00, -9.775314175290227281e-01, 1.491947941173756176e+00,
      1.491145775082470593e+00,  -9.793264029479497035e-01, -1.835422557954736522e+00, 3.409211097906539312e-01,
      1.957662820969813922e+00,  3.478619240601671381e-01,  -1.824495859935988396e+00, -9.691528799586429654e-01,
      1.495870872202105462e+00,  1.488400482827532345e+00,  -9.891051376274871965e-01, -1.850390296231056997e+00,
      3.244265878242276191e-01,  1.944121922474809860e+00,  3.413110827362990651e-01,  -1.821768428650604843e+00,
      -9.573063267811765531e-01, 1.514236744169903837e+00,  1.508939628478096040e+00,  -9.713506511734445459e-01,
      -1.839748184367982820e+00, 3.253813786933881236e-01,  1.935413839501383571e+00,  3.254004329032448295e-01,
      -1.842193003312496469e+00, -9.806415617375058291e-01, 1.495742268125974439e+00,  1.516512596604547403e+00,
      -9.326054716632418140e-01, -1.833633218291254252e+00, 2.287851746253908303e-01,  1.860517421838525642e+00,
      5.384794152093784758e-01,  -1.461850641746465174e+00, -1.093408912300081637e+00, 7.138905117957662361e-01,
      6.523695436673904169e-01,  -9.148361284015988498e-03, -6.999927466053038616e-01, -5.222580204586706465e-01,
      2.445049380987487919e-01,  5.966371652545662263e-01,  1.413067507939507195e-01,  -3.954348487890615993e-01,
      -2.628116688762455744e-01, 2.373866121120297712e-01,  2.837216761458572489e-01,  -1.496525558093976804e-01,
      -3.181721058696513893e-01, 5.669460308311169427e-02,  3.421016160400449091e-01,  5.851763223967258037e-02,
      -3.258222646060847660e-01, -1.740016839373331181e-01, 2.637621784505587152e-01,  2.643879713358527339e-01,
      -1.720785010552191574e-01, -3.220870732645387258e-01, 6.499268102844796280e-02,  3.514031187636908471e-01,
      6.361499964833523812e-02,  -3.261724627857623404e-01, -1.804119178480161900e-01, 2.515318226345136243e-01,
      2.485554977196628157e-01,  -1.865888386560058676e-01, -3.286708280876642352e-01, 7.223183508219922777e-02,
      3.748766397536335560e-01,  1.001945972854710182e-01,  -2.854232580553300314e-01, -1.484430099621606158e-01,
      2.616119614541374538e-01,  2.281299243955776657e-01,  -2.378360911604434913e-01, -4.013270221303558705e-01,
      -4.431002238201910311e-03, 3.150304291354710795e-01,  7.538626338084754297e-02,  -2.651305657879480804e-01,
      -8.444075102359345497e-02, 3.564522115888340004e-01,  3.326931426605788666e-01,  -1.476405212223987906e-01,
      -3.471227757927574653e-01, -3.821339644465658451e-04, 2.663008277812257130e-01,  -1.236544590525535947e-02,
      -3.617691595726456599e-01, -1.594445771835852343e-01, 3.041515013677485446e-01,  2.752608374669945901e-01,
      -2.234320061976452221e-01, -3.666309077530378047e-01, 1.555138609716829878e-01,  5.991562838618165188e-01,
      2.395727031850540811e-01,  -5.334292854175035625e-01, -7.173988299781207667e-01, -2.874053431491973773e-02,
      3.640880561230374624e-01,  5.257561939274062179e-02,  -2.955390817007730875e-01, -3.535598152079009182e-01,
      -1.321558389546443335e-01, 1.340505194492614405e-01,  1.788811468879522859e-01,  8.231854734570667742e-02,
      -5.089474571260232776e-03, -6.068109232214073841e-03, -2.208926490283508182e-02, -6.151292464756356188e-02,
      -5.188793950918246106e-02, 2.996648812238376230e-02,  7.292812131024753453e-02,  1.007656881884263535e-02,
      -6.596904534114987617e-02, -3.876368984097969955e-02, 4.261676593712745292e-02,  4.855285165751441895e-02,
      -2.102988677346148347e-02, -3.890526610104851762e-02, 3.457624326425645656e-02,  8.174652235946550105e-02,
      1.770296325643517035e-02,  -7.143510922277179764e-02, -6.870708931692730281e-02, -1.202485683385157156e-02,
      -1.951182752009961163e-02, -8.130810530379688206e-02, -6.704870979863195024e-02, 5.827333250822971239e-02,
      1.640891292521426126e-01,  1.472099417870301186e-01,  7.528731070437484629e-02,  5.321863170768557222e-02,
      4.290743587625941297e-02,  -6.003738344930531490e-02, -2.207538740607294658e-01, -2.795214495669220978e-01,
      -1.766889587435533648e-01, -3.111813330716103193e-02, 4.821170232765562697e-02,  1.063628131847433322e-01,
      2.145432650448417800e-01,  2.993911877456273407e-01,  2.295394539175575344e-01,  3.054503441691908822e-02,
      -1.277523492146537265e-01, -1.584432079861004816e-01, -1.452240847830046644e-01, -1.704066352520756111e-01,
      -1.781024919222879699e-01, -8.694321572293842171e-02, 4.409695613090788835e-02,  1.165909628658356040e-01,
      1.306115775893016140e-01,  1.941234299231316962e-01,  2.433476223019869489e-01,  1.438260190501788049e-01,
      -1.693579587570422651e-01, -4.189892663710857446e-01, -3.670953692804365676e-01, -5.243796510396939344e-03,
      2.697453724377494311e-01,  5.606967605037058849e-02,  -1.336330319830032476e-01, -1.968709854334711185e-01,
      -2.087920324902741431e-01, -1.209335148965447887e-01, 3.167940565814987564e-02,  1.488251506204237296e-01,
      1.465543649533269932e-01,  5.446596924130853168e-02,  -2.254669760169095843e-02, -3.432948183237215234e-02,
      -3.045866739640899767e-02, -3.792233926513460807e-02, -3.624290708745740719e-02, -7.726838708116441981e-03,
      1.827209317733121535e-02,  9.662966541991018302e-03,  -1.042588052209014564e-02, 5.729733248186564057e-03,
      5.151216322237899686e-02,  7.542401569179536802e-02,  5.243387856359182919e-02,  1.267260839111160851e-02,
      -1.665267358469470563e-02, -5.470532515396978740e-02, -1.213714262624657619e-01, -1.793200973226497519e-01,
      -1.657505308855534154e-01, -7.097015142802499954e-02, 5.461386109345674200e-02,  1.688213057847406107e-01,
      2.673031213752043445e-01,  3.338040757747570697e-01,  3.099731124739781296e-01,  1.550764105573461393e-01,
      -8.595387468510176654e-02, -3.120381452113342724e-01, -4.525917271704635314e-01, -4.904956065716953861e-01,
      -4.139476117936018440e-01, -2.041856960054673209e-01, 1.054227595474576118e-01,  4.014397443732543591e-01,
      5.638716504290620657e-01,  5.556407664236857613e-01,  4.151944682384513530e-01,  1.888811317152777058e-01,
      -9.325710837486608784e-02, -3.657078022012389962e-01, -5.263901657940285306e-01, -5.159366309488829794e-01,
      -3.683073590772836803e-01, -1.569366535146479724e-01, 7.612977186372285598e-02,  3.185882954066596162e-01,
      4.829671508855328810e-01,  4.590809383249135900e-01,  2.402592143600549845e-01,  -4.959598342666301368e-02,
      -2.674915128776357842e-01, -3.451914171650982133e-01, -3.146645721834679388e-01, -1.019830448167796944e-01,
      2.185015503227412115e-01,  1.105401825914447966e-01,  -3.645804114634373927e-02, -1.570039526128303997e-01,
      -2.023978599110545784e-01, -1.623464906102394245e-01, -6.545899785271218352e-02, 3.926936206876608521e-02,
      1.079564450580926677e-01,  1.203903895828972725e-01,  8.379061046861495388e-02,  2.316160694266171383e-02,
      -3.378990976580330957e-02, -6.842860779162730778e-02, -7.504109145654207225e-02, -5.766574730166833945e-02,
      -2.425193181962282535e-02, 1.750324458295147295e-02,  6.080081102869283605e-02,  9.794915538075871231e-02,
      1.187917204229755330e-01,  1.116630889985279346e-01,  6.747082005648256942e-02,  -1.450600501662783516e-02,
      -1.218421199123784215e-01, -2.280846691007404614e-01, -2.975174624722484173e-01, -2.953093996538160293e-01,
      -2.005043417186586907e-01, -1.754898506514538173e-02, 2.184259710114040132e-01,  4.460936290592523168e-01,
      5.927991506916320263e-01,  5.966356990504843028e-01,  4.292049463066227588e-01,  1.110356118504624012e-01,
      -2.870799137027047476e-01, -6.595112304735133435e-01, -8.949676681382167098e-01, -9.110710914499896562e-01,
      -6.830757268582222652e-01, -2.564448861041591643e-01, 2.621880408486327863e-01,  7.339937626324779529e-01,
      1.028251362274525293e+00,  1.061292538978783728e+00,  8.217165779782973356e-01,  3.740726530008248174e-01,
      -1.597887324620674843e-01, -6.356566411261002081e-01, -9.277198330901089696e-01, -9.636319382545472934e-01,
      -7.443779524024758398e-01, -3.434629592486291849e-01, 1.149619503912955865e-01,  4.938610531529100678e-01,
      6.865132343536899384e-01,  6.512764584705139637e-01,  4.247204034709558540e-01,  1.062680159964822790e-01,
      -1.815204680229429990e-01, -3.415916380800598784e-01, -3.390205139272841062e-01, -2.091074574063999936e-01,
      1.545215409971751608e-01,  9.812498051948156941e-02,  4.167779402703281066e-02,  -5.807180919864113333e-02,
      -1.701474975430069814e-01, -2.352505690407705741e-01, -1.201242514156009022e-01, 3.686499118737240899e-02,
      6.964282482937619867e-02,  1.682271974604750836e-02,  2.533130619533068006e-02,  9.129652260616810677e-02,
      7.062003708897043874e-02,  -6.171739761984700035e-02, -1.411087328260186846e-01, -6.422356041703554397e-02,
      6.905407262330469365e-02,  1.095500999286599803e-01,  8.584088621666889629e-02,  1.259838815983557181e-01,
      2.106154429632017544e-01,  1.752456226813108309e-01,  -3.229614216981113728e-02, -2.570330759224564998e-01,
      -3.581727831852760668e-01, -3.841403752369945135e-01, -4.251374983113783146e-01, -3.998338236580256666e-01,
      -1.563300612899848896e-01, 2.672114035255429010e-01,  6.436290145429529597e-01,  8.247584207295350911e-01,
      8.560190198647534832e-01,  7.820559464177336828e-01,  5.024840511314054847e-01,  -5.554898059737789495e-02,
      -7.222294633196590929e-01, -1.198086833117398164e+00, -1.346353877467284299e+00, -1.234867521797036183e+00,
      -9.119319777023042750e-01, -3.247384457075377373e-01, 4.787057101089932099e-01,  1.211292991091145410e+00,
      1.570548045103978163e+00,  1.508261696931222140e+00,  1.169158020896638561e+00,  6.389525848011010645e-01,
      -7.697625147307118887e-02, -8.413374629980580055e-01, -1.350443936193856498e+00, -1.405115760220999599e+00,
      -1.092747434740040102e+00, -6.448937162994430317e-01, -1.593762044139684919e-01, 3.668706036705465801e-01,
      8.140108496750596556e-01,  9.573118865322826077e-01,  7.437002629365834228e-01,  3.868241848042429654e-01,
      1.125042800628359152e-01,  -1.165870956388290192e-01, -2.803511096314761364e-01, -3.629150740911444295e-01,
      1.933486562518807739e-01,  1.076426712595783958e-01,  5.047841060002760533e-02,  -3.013160590350243487e-02,
      -1.166270859828469342e-01, -1.470231926047324555e-01, -1.168429765426066552e-01, -1.186931086208969088e-01,
      -6.812186274945519759e-02, -1.489070404138536849e-02, 2.383164778178215384e-02,  1.387594893358104660e-02,
      1.595918415137407426e-02,  1.475871847142585028e-02,  5.206363357864175062e-02,  8.463267165763772859e-02,
      1.269979573658552297e-01,  1.768418789044756967e-01,  2.166723362144298837e-01,  1.928227121246958164e-01,
      7.373273957566094106e-02,  -9.100122345185270023e-02, -2.685445812189546921e-01, -4.430219197822350963e-01,
      -5.825907070499073415e-01, -6.089242391644661412e-01, -4.643510465300749179e-01, -1.567919562176030568e-01,
      2.392785001773080067e-01,  6.465360271175842888e-01,  9.944262886575462312e-01,  1.192167799213530444e+00,
      1.135440257101176798e+00,  7.809627031268677744e-01,  1.920143316393541633e-01,  -4.951525455844807411e-01,
      -1.141122955512682502e+00, -1.616497805623771100e+00, -1.792029734191694645e+00, -1.570461588154390187e+00,
      -9.691264246785707970e-01, -1.261184475651200565e-01, 7.540490136214349493e-01,  1.493421167179531439e+00,
      1.945792402071490290e+00,  1.999506416397538455e+00,  1.602371568569924687e+00,  8.654587838732198390e-01,
      -1.247400493433087690e-02, -8.371511917615156451e-01, -1.440605562491721781e+00, -1.723056253958058504e+00,
      -1.604451793975099738e+00, -1.173636244206996970e+00, -5.299931613627775207e-01, 1.139518040537967625e-01,
      6.430504052856730324e-01,  9.619414442721506697e-01,  1.067738651002308581e+00,  8.884017261368395424e-01,
      5.846004974735256043e-01,  2.505605712832118526e-01,  -7.267055625585411294e-02, -3.196013881602461915e-01,
      1.877756745197220467e-01,  1.773498263523564655e-01,  7.981390462123486018e-02,  -5.588962475280836312e-02,
      -9.171829747138637856e-02, -9.218874200454439671e-02, -1.490767091650735399e-01, -2.202163109979105027e-01,
      -1.590887205996218690e-01, -8.585708015613892352e-02, -4.676577808961319277e-02, -1.340275293147117838e-02,
      6.718951459939699611e-02,  1.756054550337380893e-01,  2.197110402800965812e-01,  2.533753564706837103e-01,
      2.804509470618867240e-01,  2.658515891773254869e-01,  1.699413070810313064e-01,  9.807904495693667637e-03,
      -1.918865455864875169e-01, -3.969382920812995486e-01, -5.758512082214654049e-01, -6.908500730837361381e-01,
      -6.857060639279918268e-01, -5.193811494364500847e-01, -1.930283873096097502e-01, 2.371766815606875800e-01,
      6.911276023359046450e-01,  1.087055762182129959e+00,  1.335471746908890589e+00,  1.347638759454575563e+00,
      1.066688774587909450e+00,  5.176773432378547479e-01,  -1.972534213772513811e-01, -9.430749618329744566e-01,
      -1.585070611914226824e+00, -1.981286135412752225e+00, -2.011450033924125691e+00, -1.631945203287598289e+00,
      -9.084431679538237381e-01, 1.282577536801942156e-02,  9.597190668882201736e-01,  1.754029544855383760e+00,
      2.230044038226858927e+00,  2.275539468229606843e+00,  1.880232343253755989e+00,  1.150973975308672292e+00,
      2.300210360237523455e-01,  -6.952702251670459521e-01, -1.440829021202086890e+00, -1.866272080506176545e+00,
      -1.949683808257608453e+00, -1.641031203195145949e+00, -1.047399904224441514e+00, -3.568724812299680638e-01,
      2.770692529416379779e-01,  8.336316227699677173e-01,  1.165808417270559616e+00,  1.208073944954302981e+00,
      1.021436940236768676e+00,  7.254039571157427924e-01,  3.939841214883700315e-01,  3.423510521739598408e-03,
      1.979195562565257416e-01,  1.032875325408822015e-01,  7.388021258899205024e-02,  1.364670078299134136e-02,
      -7.863463019867435178e-02, -1.884531318644405151e-01, -2.202831533155493737e-01, -1.860729836987937147e-01,
      -2.029050262230952528e-01, -1.490048539423707574e-01, -2.833820635971010665e-02, 9.175180730792789618e-02,
      1.965433600681169846e-01,  2.755449113737773459e-01,  3.630533814099292100e-01,  3.698641325118732226e-01,
      2.858133179835334881e-01,  1.608399517947124435e-01,  -2.870693803023338653e-02, -2.592171368239996054e-01,
      -4.977879803753233467e-01, -6.797226920112315884e-01, -7.609601510412746794e-01, -7.282424640194391507e-01,
      -5.387360709737706310e-01, -2.014929239664536020e-01, 2.346027491889908223e-01,  7.014340980213025212e-01,
      1.114543840168730426e+00,  1.390054706777353433e+00,  1.442413393992351578e+00,  1.229725387785199464e+00,
      7.623017392947875059e-01,  1.009878983305406569e-01,  -6.525661278980623425e-01, -1.369823868264403721e+00,
      -1.908453646823702776e+00, -2.141326600352158938e+00, -2.011622777745675883e+00, -1.524674751459233812e+00,
      -7.496178124873309478e-01, 1.954429556211285668e-01,  1.144178670530257902e+00,  1.899237394402517154e+00,
      2.344589308571881414e+00,  2.403475914952520220e+00,  2.050772728378840171e+00,  1.356516152565677258e+00,
      4.483601528818672022e-01,  -4.742202657667306531e-01, -1.295285837794622141e+00, -1.881322027563492760e+00,
      -2.082692093657950494e+00, -1.940419540235171603e+00, -1.511008413597929145e+00, -8.956003789833757445e-01,
      -1.687578221313258409e-01, 5.359218065178924517e-01,  9.919970528666315346e-01,  1.260262585582463846e+00,
      1.343782241322490245e+00,  1.183353957092918352e+00,  8.382339580124620326e-01,  4.029866037595676298e-01,
      1.135536725882935472e-01,  5.869987746684678098e-02,  6.163084541150179452e-02,  1.457199816867432544e-02,
      -1.231539552306697149e-01, -2.359848323791751967e-01, -2.205030920483776746e-01, -9.313017843559566467e-02,
      -8.827519165220858166e-02, -1.267613114472213587e-01, -9.985223900131810770e-03, 2.375381680969095577e-01,
      3.792385911043010815e-01,  3.228578206321173205e-01,  2.387692702279881618e-01,  2.510476591105527122e-01,
      2.180206709417117084e-01,  -3.013711947522528661e-02, -3.869499138443790454e-01, -6.122981087081406093e-01,
      -6.319654726059141803e-01, -6.479396689383293273e-01, -7.185904880161348984e-01, -6.210617102189838201e-01,
      -2.049070814464947210e-01, 3.392518291328842639e-01,  7.357846848640706705e-01,  9.876796659454714078e-01,
      1.259047856168387991e+00,  1.489080505390903086e+00,  1.396944230536462550e+00,  8.875035340004571172e-01,
      2.163887415926450675e-01,  -3.800154728812719096e-01, -9.531611119833783841e-01, -1.601516861774638700e+00,
      -2.124042072523784075e+00, -2.201127858925345127e+00, -1.805754396537006201e+00, -1.202809565364706401e+00,
      -5.472391237384681695e-01, 2.570512756230960427e-01,  1.221293094656357159e+00,  2.043278712963822574e+00,
      2.398915490668351413e+00,  2.310477520542332464e+00,  2.012941191828379939e+00,  1.527085048070050410e+00,
      7.533218913102441849e-01,  -2.533427864789731254e-01, -1.166944210792282055e+00, -1.719680948420976518e+00,
      -1.958159941635327694e+00, -2.033609401020752383e+00, -1.892345858060423103e+00, -1.383842786154750870e+00,
      -5.995256488034405029e-01, 1.375259602986234342e-01,  6.364289318617047941e-01,  1.049597973983896537e+00,
      1.350674458664880451e+00,  1.437532750381322222e+00,  1.233692367346445495e+00,  8.456034531491152251e-01,
      1.467260742316298938e-01,  1.123622900941055325e-01,  -5.657402306607337700e-02, -1.057557238112471903e-01,
      -6.295177083605282831e-02, -4.464776814485743595e-02, -9.866228491316128557e-02, -1.508522603967922537e-01,
      -6.124591639689858785e-02, 1.668056972825438800e-01,  2.198902799992258628e-01,  1.735346770311333997e-01,
      1.543669368623728222e-01,  2.479903525544368270e-01,  2.508892409513964195e-01,  4.415746317429558587e-02,
      -2.566419500318690150e-01, -3.860285406362509164e-01, -4.385077795744283247e-01, -5.455650372160563144e-01,
      -7.194165864493482498e-01, -6.919814987446996080e-01, -3.932271443011954304e-01, -1.916367786634825404e-02,
      2.543158665452901479e-01,  5.362962306259021483e-01,  9.301494974354773682e-01,  1.316753443511357702e+00,
      1.414038687718725873e+00,  1.202569425983842866e+00,  8.689806557591766012e-01,  5.161383179290985890e-01,
      4.411968685043276064e-03,  -7.173881381080952080e-01, -1.425111261817489927e+00, -1.831311742363123773e+00,
      -1.937474037265871640e+00, -1.889854852341761537e+00, -1.695171737706322013e+00, -1.159375062802855183e+00,
      -3.000537809259007926e-01, 6.151866450149036103e-01,  1.309609932719181247e+00,  1.813242814634357103e+00,
      2.218736710663839151e+00,  2.409349752238156395e+00,  2.131919718945650466e+00,  1.471173600338621990e+00,
      6.747505405841823789e-01,  -2.135423953263143951e-02, -7.394436871413290202e-01, -1.470481297711863755e+00,
      -2.008134008841505835e+00, -2.108552724408489176e+00, -1.866974383838370111e+00, -1.463971343776215184e+00,
      -1.062583856409258232e+00, -4.528804576639091084e-01, 2.638738743497704986e-01,  8.600976087351883681e-01,
      1.201019500614485702e+00,  1.306325870313787219e+00,  1.298340598227454956e+00,  1.239492502421928988e+00,
      3.028411255436088464e-03,  1.197869650068672000e-02,  1.094005712205633196e-01,  4.161638887259306446e-02,
      -8.574762736806021413e-02, -6.156022851758608666e-02, 7.186336795651582232e-02,  1.465110572446766957e-01,
      9.272330430872105511e-02,  5.512180259669289401e-02,  8.948498433501747562e-02,  2.124272042617097456e-01,
      1.802655469612115535e-01,  -4.060206081937024064e-02, -2.296276229577691730e-01, -2.472724972173437341e-01,
      -2.800315819014573826e-01, -4.801815148332166938e-01, -6.520023102575503993e-01, -6.267062609140928719e-01,
      -4.120031574709806521e-01, -1.829870682782789026e-01, -4.825200993772560398e-02, 1.849531501132461453e-01,
      6.151043230767001546e-01,  1.020821009498220189e+00,  1.177580129516455409e+00,  1.133752644735109616e+00,
      1.077107855558536054e+00,  9.762625382507381655e-01,  6.409098583248109549e-01,  3.216072586947701234e-02,
      -6.020293971300634261e-01, -1.053596774720969131e+00, -1.385727194206231694e+00, -1.721525846201141130e+00,
      -1.934550898500656757e+00, -1.796221817022149336e+00, -1.290663183450162288e+00, -6.728599988824979228e-01,
      -1.011788165144424517e-01, 5.193417073111463722e-01,  1.264039636031914382e+00,  1.906733766342876057e+00,
      2.161609805423058361e+00,  2.086203981389263884e+00,  1.848918451869991930e+00,  1.499150700336880915e+00,
      9.535066525350043287e-01,  1.280647677557977537e-01,  -7.070739787578573798e-01, -1.258380759315280484e+00,
      -1.578292684975801219e+00, -1.829282097597322077e+00, -1.959455522455186616e+00, -1.748901866766265245e+00,
      -1.269585892234865288e+00, -6.486031512762205908e-01, -1.114546356682864919e-01, 3.292747349790575706e-01,
      7.699011018435779619e-01,  1.205323951948908512e+00,  1.405573432282957169e+00,  1.270626147412671880e+00,
      7.703878338784575508e-02,  8.328670371872076705e-02,  8.964832657000576521e-02,  9.755243638104110770e-02,
      1.072988178521259717e-01,  1.175894779174822546e-01,  1.253619819613369657e-01,  1.260220122476720661e-01,
      1.141072762629739001e-01,  8.433283180603712292e-02,  3.288183465571936637e-02,  -4.126856281792745662e-02,
      -1.352430003706726058e-01, -2.416479549938996829e-01, -3.486560233049110780e-01, -4.409238659271715877e-01,
      -5.013439484087415243e-01, -5.134798602301046122e-01, -4.643876237814310581e-01, -3.474079255984316994e-01,
      -1.644518708048106070e-01, 7.268551100963493994e-02,  3.423560696067957654e-01,  6.146352965771482157e-01,
      8.544191770021098975e-01,  1.025762651931504177e+00,  1.096943695474724034e+00,  1.045556718592434819e+00,
      8.628527938316823409e-01,  5.565765645046996113e-01,  1.517072965686061536e-01,  -3.112197559386404855e-01,
      -7.801899525359208276e-01, -1.197789971414207733e+00, -1.508558813850797486e+00, -1.666625434326106880e+00,
      -1.642536441199826225e+00, -1.428228612425967547e+00, -1.039318058075737738e+00, -5.142350309407417441e-01,
      8.981942908081297361e-02,  7.036487296058009377e-01,  1.254702060668571040e+00,  1.676415010224636237e+00,
      1.916950671050619004e+00,  1.946104059862946656e+00,  1.759381635529760235e+00,  1.378667621338910187e+00,
      8.493788212324417941e-01,  2.345142039704910530e-01,  -3.935529485338875588e-01, -9.623812023386317804e-01,
      -1.408570928148620283e+00, -1.685441345497397325e+00, -1.768275474812050829e+00, -1.656509831104123087e+00,
      -1.372698677366644082e+00, -9.585414256224105145e-01, -4.686572715043277881e-01, 3.693261579177598009e-02,
      5.005327104119423609e-01,  8.736685172964291368e-01,  1.122294666938070673e+00,  1.229756184691179310e+00,
      1.726309009731196442e-01,  1.430555717394552673e-01,  1.448244296758931238e-01,  1.554106942819315773e-01,
      2.260047553059642556e-01,  2.193449888207139042e-01,  1.140876910464539568e-01,  5.709358199018328001e-02,
      4.267809541586050498e-02,  -1.124069483956910941e-02, -1.455380431434608046e-01, -2.317759765138503736e-01,
      -3.460735755986550366e-01, -4.369015657894604332e-01, -4.773969661178912860e-01, -4.684770217894247968e-01,
      -4.742103851556133121e-01, -4.264949258024777756e-01, -2.677922255544332719e-01, -6.761245426763606847e-02,
      1.631187046948890662e-01,  3.869188564880423620e-01,  5.990302832795830756e-01,  8.028669469003396619e-01,
      9.390998624318590560e-01,  1.005696173684265116e+00,  9.504639605478419906e-01,  7.780535180260986783e-01,
      5.290217240375075791e-01,  2.087874246362421415e-01,  -1.711811561585681873e-01, -5.844746399555035277e-01,
      -9.532431321335074204e-01, -1.252717372690612452e+00, -1.427683749681106340e+00, -1.486062739646681008e+00,
      -1.402269785669054469e+00, -1.160141840480132025e+00, -7.654122092833203217e-01, -2.869160824785925157e-01,
      2.183530104822541673e-01,  7.323962983080670730e-01,  1.178286232220251417e+00,  1.538354930733374593e+00,
      1.742900496317809189e+00,  1.749576949754955368e+00,  1.573372473320533871e+00,  1.229480790337705631e+00,
      8.173994879517971768e-01,  3.290973087852758772e-01,  -2.408671277910405528e-01, -7.610963918030260178e-01,
      -1.167892777475626698e+00, -1.436197342149151845e+00, -1.581888085614167716e+00, -1.512701638107700308e+00,
      -1.362369346079326693e+00, -1.111767065979217595e+00, -7.144214570701877820e-01, -2.091468727346524337e-01,
      2.091888708179358847e-01,  5.171919648728799945e-01,  8.144730631522935083e-01,  1.023077902305088216e+00,
      1.881161805480971172e-01,  2.359710490397077809e-01,  2.150580415294692516e-01,  2.344150430302511523e-01,
      1.926594747174036337e-01,  1.481402186341032878e-01,  8.203683962513245997e-02,  -5.258755150704496999e-03,
      -7.769905418511015294e-02, -2.025594106227799618e-01, -3.332452450650434383e-01, -3.510645492577034177e-01,
      -4.239751250964340357e-01, -4.712830594653529270e-01, -5.213993464760555696e-01, -4.404949476452622026e-01,
      -3.150187285910792667e-01, -1.874702455461361716e-01, -5.185521692353211448e-02, 1.331771956144575664e-01,
      3.921407919052110613e-01,  6.254442977404938153e-01,  7.165948518077205343e-01,  8.057693601279675155e-01,
      8.779678650429114661e-01,  9.065333211798984880e-01,  7.545641325022122148e-01,  4.929139969421373890e-01,
      2.182451168723386070e-01,  -3.254693864949957660e-02, -3.581571483898192687e-01, -7.278449519031879866e-01,
      -1.061527565480239410e+00, -1.214970824206470512e+00, -1.275867659122747799e+00, -1.274381980390025593e+00,
      -1.203873745190214617e+00, -9.451211225442880393e-01, -5.337027719252307456e-01, -8.429182233452817141e-02,
      2.969997489731258655e-01,  6.993845280569482847e-01,  1.094476334285647612e+00,  1.442432161491944864e+00,
      1.553823011812375698e+00,  1.508591457666013991e+00,  1.384997334525116974e+00,  1.155277015782768757e+00,
      7.917914001067103413e-01,  3.239058760686507887e-01,  -1.705339741665028441e-01, -5.581305902874631464e-01,
      -9.223197970449693583e-01, -1.190412878226028370e+00, -1.414329278227212772e+00, -1.408740857848732819e+00,
      -1.285714629826386401e+00, -1.095604819771491156e+00, -7.908813975703943289e-01, -4.400411334473687819e-01,
      -7.720095693434277095e-02, 2.855952692611269383e-01,  5.598792759697388544e-01,  8.184909533996466147e-01,
      2.736273543021752186e-01,  1.680436987381665748e-01,  2.233395602269365177e-01,  2.512446833159482917e-01,
      1.663657868570496690e-01,  4.677076204011699351e-02,  -5.582959949554577861e-02, -1.163693045169445078e-01,
      -1.754915053392157864e-01, -2.868173920594484505e-01, -4.589785020228672185e-01, -4.704399330489481557e-01,
      -3.904897481210252153e-01, -4.363423967455598573e-01, -4.408960033207887652e-01, -3.166058753382128277e-01,
      -1.452583237883194323e-01, 3.331742695095382678e-02,  1.706360507272416394e-01,  3.192245312856296646e-01,
      5.129550139512375484e-01,  7.119145007865759212e-01,  7.709150876698102062e-01,  7.371315741633722718e-01,
      7.468186825997702449e-01,  6.950974938010141546e-01,  5.081937915463478062e-01,  2.555076320331673423e-01,
      -1.660055183057786155e-02, -2.718323875504024545e-01, -5.255601006474531767e-01, -8.013370425252260842e-01,
      -1.021898999220889337e+00, -1.111232694673638832e+00, -1.110408822178643451e+00, -1.065164875023532964e+00,
      -9.284082209065659397e-01, -6.710414167675055053e-01, -3.276198205776061911e-01, 4.568604461223657598e-02,
      3.586647989209277299e-01,  6.566323483686988816e-01,  1.011189076605315762e+00,  1.250796398086361894e+00,
      1.296704640196478353e+00,  1.268709707245692542e+00,  1.189440135253847286e+00,  1.018914442371484874e+00,
      7.091944518973579692e-01,  3.234813112441642380e-01,  -7.559391924824936104e-02, -3.771037195581692369e-01,
      -6.380602485179993622e-01, -9.960722010582592212e-01, -1.206861603838810248e+00, -1.182183404867649168e+00,
      -1.132777980764397396e+00, -1.046786330971882339e+00, -8.774210666052706387e-01, -5.983529279390236155e-01,
      -2.543623487536421646e-01, 7.944545744965257617e-02,  2.999919524952535665e-01,  4.702522674378065703e-01,
      2.897149595283884382e-01,  1.533069401671905141e-01,  9.600327719960310979e-02,  2.067554326643253892e-01,
      1.459231900630888479e-01,  -8.477993157657867651e-02, -2.305667029280785707e-01, -1.867680675863448436e-01,
      -1.870370726892342239e-01, -3.358214513429326753e-01, -4.881436916938916015e-01, -4.573535614570060637e-01,
      -3.141992331736871980e-01, -3.245885330649925238e-01, -3.194886082104057556e-01, -1.821179012439004263e-01,
      4.112269846750674085e-02,  2.020967994375577614e-01,  2.592079507606143141e-01,  3.815713138259901327e-01,
      5.863790042747188958e-01,  7.206486123266985233e-01,  6.890296547366029367e-01,  6.299441288831169050e-01,
      5.773587754529514449e-01,  5.041034000340586241e-01,  3.290399297308453930e-01,  4.920234650940384646e-02,
      -1.783499854726365297e-01, -3.740781429115180301e-01, -5.816017159789339530e-01, -8.007545082678807491e-01,
      -9.644905759825412872e-01, -9.931632007610342594e-01, -9.441474129077135347e-01, -8.648708557702422528e-01,
      -7.404851892495842858e-01, -5.184319926152773927e-01, -1.764781195363510080e-01, 1.222874961517503994e-01,
      3.682607212240895933e-01,  6.246464704138199320e-01,  8.974326584310664545e-01,  1.130300174993894124e+00,
      1.162911762943819260e+00,  1.072988216651206361e+00,  1.001236335979882597e+00,  9.246491922327110258e-01,
      6.753060515138049613e-01,  3.032983386183236152e-01,  -2.903095545257759758e-02, -2.567378820919261062e-01,
      -4.759796238237690158e-01, -8.349293543345818192e-01, -1.047037514265016345e+00, -1.026079659200134664e+00,
      -9.483577511135131699e-01, -9.544745466890556251e-01, -9.416131010177876126e-01, -6.829579215834233885e-01,
      -2.954012829118684014e-01, -4.925442598794700644e-02, 3.689625781468347521e-02,  2.859066092899030243e-01,
      2.509818586430216936e-01,  1.842599822864128545e-01,  5.651273913449596231e-02,  -1.136641092142118455e-03,
      5.314039190335323459e-03,  -5.133954982175892995e-02, -2.011193697726740004e-01, -3.174537527965924366e-01,
      -3.157261584582995440e-01, -2.829729554081275578e-01, -3.218251337568452386e-01, -3.802620122384016144e-01,
      -3.868563634301446230e-01, -1.853754564893856371e-01, 3.019633715274166151e-03,  2.206810844558627568e-02,
      1.950148959117479694e-02,  1.910780327080033314e-01,  4.440265778800091545e-01,  5.369158890532397255e-01,
      4.654642690150756490e-01,  4.572332400363430249e-01,  5.845159206629798598e-01,  6.358210686041558812e-01,
      4.591512900508898376e-01,  2.107476008490490327e-01,  9.932922252132257712e-02,  4.954995807852673184e-02,
      -1.585303333742181520e-01, -5.046588662981835727e-01, -7.354688041374819463e-01, -7.540548776077706794e-01,
      -7.525147973414769709e-01, -8.625856682354984262e-01, -9.366704622132667746e-01, -7.693273290009360332e-01,
      -4.485811357371958241e-01, -2.244080122899938523e-01, -1.191776030106372897e-01, 8.031114532015962149e-02,
      4.394963687827080356e-01,  7.381877338478749495e-01,  8.085047947489780729e-01,  7.916076862471849163e-01,
      8.894752024080258002e-01,  1.021839698763748139e+00,  9.548290408460443635e-01,  6.883247614053178420e-01,
      4.617411489203375186e-01,  3.688695796412180816e-01,  2.186529354784229606e-01,  -1.279051902042792344e-01,
      -5.019882046397190489e-01, -6.672869016827609467e-01, -7.636707182644285608e-01, -8.455659437032555514e-01,
      -9.542446742725352316e-01, -9.742078310896505888e-01, -8.236924725065022379e-01, -5.926992100281092135e-01,
      -4.141012032456360958e-01, -2.693968391445499266e-01, -4.939591508416577709e-02, 2.344653737002040106e-01,
      4.747643560646129524e-02,  3.477437005883472071e-02,  8.383377199071352648e-02,  2.510664306526687317e-02,
      -1.424049599175457037e-01, -2.425100086212893802e-01, -2.191072898867240937e-01, -1.621254648258393105e-01,
      -2.317547090250825781e-01, -3.484169006555289383e-01, -3.233139514783892055e-01, -1.678572618693576768e-01,
      -7.480186723515618030e-02, -1.023509999167394463e-01, -8.315169525566028230e-02, 9.178223123668298455e-02,
      3.334270582375974401e-01,  3.261153112364972850e-01,  2.314975640036733562e-01,  3.243769602133048102e-01,
      5.394556893333580794e-01,  5.706966121201653142e-01,  3.703695707245379110e-01,  2.236017036453251394e-01,
      2.757925567792656740e-01,  2.944309327214336136e-01,  5.930251702751001414e-02,  -2.994540899938952783e-01,
      -3.778807741670948683e-01, -3.092395987584689032e-01, -4.372995022152535438e-01, -7.470079700466597528e-01,
      -8.715798957235488809e-01, -6.839368804200198770e-01, -4.966010954918945175e-01, -5.408283491192638071e-01,
      -5.864049339774849434e-01, -3.304572844981776836e-01, 1.052639200425978944e-01,  2.687805404964021538e-01,
      2.629424369697350716e-01,  4.256023879186502379e-01,  7.833394270948978999e-01,  9.846800821697564432e-01,
      8.617712492654249212e-01,  6.939977261290004584e-01,  7.392589621328143901e-01,  8.083847517913232972e-01,
      5.783840437009357682e-01,  2.127551501857655736e-01,  -4.793296271213072501e-02, -1.220412997945457823e-01,
      -2.365938049537476828e-01, -5.264883330399160455e-01, -7.888880972046899975e-01, -8.004892269206252964e-01,
      -6.745383632631807291e-01, -6.681505960100158070e-01, -7.583360021929064221e-01, -6.830699694661372945e-01,
      -4.249440263026483255e-01, -1.741563878612986394e-01, -9.063693653494860059e-02, -4.356107008774551259e-02,
      7.321808039794695799e-02,  2.380901590695183936e-02,  -2.751142583131200095e-02, -1.332714855350916883e-01,
      -1.220768121956060243e-01, -1.010897484693604326e-01, -1.742404111534891775e-01, -2.778977990730032155e-01,
      -2.528027643808485325e-01, -1.569866962648459041e-01, -1.263980574741991880e-01, -1.639125361734175279e-01,
      -7.521477922673232286e-02, 2.830540153179601087e-02,  1.021948489754357786e-01,  1.509635700791882307e-01,
      2.782970795799216690e-01,  3.205458107648209154e-01,  3.267870750510250555e-01,  3.559675171941669714e-01,
      3.982850050050637525e-01,  3.857052236671436041e-01,  2.887148808624919738e-01,  1.702717015093893616e-01,
      9.885411377424634105e-02,  6.165864427309489942e-03,  -1.090077400931198093e-01, -2.392838469261402978e-01,
      -3.674935411065574087e-01, -4.547403510769028046e-01, -5.327566277689934626e-01, -5.544568064090868997e-01,
      -5.494122548752417012e-01, -5.399434152245331520e-01, -5.012541115533053482e-01, -4.016977491261139455e-01,
      -2.872999445625856185e-01, -1.252422896180991618e-01, 4.474198222056428864e-02,  2.031668417837795559e-01,
      3.393961750204066985e-01,  4.506434540818844425e-01,  6.013915373286873312e-01,  7.186998147929229974e-01,
      7.355562581824524049e-01,  6.790268700173530680e-01,  6.157986942013415899e-01,  5.562548243436666873e-01,
      4.437258919518201061e-01,  2.334736128146131795e-01,  9.466314539955893170e-02,  -7.200144873716969596e-02,
      -2.650692583487521126e-01, -4.341907718079499601e-01, -4.629900707917616876e-01, -5.433310589367671417e-01,
      -6.695570584036687478e-01, -7.040140540669374003e-01, -5.883143874410380336e-01, -4.825279064469844004e-01,
      -4.516040397823052888e-01, -3.940371670920473912e-01, -2.064065009288035268e-01, -6.488992968728242439e-02,
      -6.723420182669906896e-03, 1.832894567485181953e-02,  -7.856005883318716021e-02, -1.057681122832552223e-01,
      -9.126664412895567347e-02, -1.443881669649202948e-01, -1.576865280240225542e-01, -1.479678724789767197e-01,
      -1.274994770780365494e-01, -1.107232489558259425e-01, -9.841101505382197911e-02, -9.528384056423406731e-03,
      3.371672364678073019e-02,  4.793749894056247018e-02,  1.636084266298088175e-01,  2.071562964673385465e-01,
      2.110205428463668709e-01,  2.942100468118016288e-01,  3.025490468956293921e-01,  2.794320653584481673e-01,
      2.843249922510974748e-01,  2.361433444581998997e-01,  1.794633482104236455e-01,  1.005233736377470904e-01,
      8.107782630280937694e-03,  -6.745656597887597639e-02, -1.892710177319704423e-01, -2.794949566663201268e-01,
      -3.400986883734262722e-01, -4.376186429877271800e-01, -4.678532206752753497e-01, -4.776929896004197262e-01,
      -4.946744722450769904e-01, -4.309195492326619026e-01, -3.751497235098402716e-01, -2.993086221432724181e-01,
      -1.576471299366999368e-01, -5.571691281052375266e-02, 7.249951674350330055e-02,  2.271207958933705584e-01,
      3.283795408454601161e-01,  4.378203033463570204e-01,  5.232948653502461633e-01,  5.746071637164829049e-01,
      6.040901192152554611e-01,  5.669110083949433498e-01,  5.447760093545850335e-01,  4.783217079111895531e-01,
      3.265325075995235693e-01,  2.468403568114483360e-01,  1.222868796628895638e-01,  -7.507787219825361180e-02,
      -1.669353199274775656e-01, -2.808275410569823771e-01, -4.289033062631600890e-01, -4.851604569562498193e-01,
      -5.286966581012793043e-01, -5.568118850092828032e-01, -5.544536837582648703e-01, -5.097590083130195682e-01,
      -4.077767743112116205e-01, -3.590563433767017254e-01, -2.568323280183899548e-01, -7.788508658903724180e-02,
      -3.147893611747956677e-02, -1.493939023593335302e-02, -8.957617314333463088e-02, -8.532724167945135441e-02,
      -1.069942061709636127e-01, -6.934961057683493302e-02, -9.247737017713857088e-02, -8.893000492760837206e-02,
      -6.272674499776025037e-02, -1.711920029910489471e-02, 3.549111306146712774e-03,  5.417880934428492834e-02,
      6.819032336480047862e-02,  1.430846789212276982e-01,  1.496306681501724523e-01,  2.181052531363233959e-01,
      2.008594293907250972e-01,  2.510225025894266548e-01,  2.194803840061405886e-01,  1.887405184033458250e-01,
      2.088688428589401125e-01,  1.629203158968475784e-01,  6.446171621021457021e-02,  -4.796180651504833303e-02,
      -8.800510913151426973e-02, -1.212319563046126980e-01, -2.293783881073599429e-01, -3.254694893989787396e-01,
      -3.561472995581365097e-01, -3.667528746097366010e-01, -3.692183528963707828e-01, -4.012187958874691152e-01,
      -3.483829577362937413e-01, -3.372862309570864858e-01, -2.430309517111126083e-01, -1.823221664635843453e-01,
      -1.180950283910139897e-01, -4.029224368235023723e-02, 9.592598287373632626e-02,  2.356617310797001763e-01,
      2.904366848241204946e-01,  3.409681327383979887e-01,  4.511291562494007534e-01,  5.333548138259963656e-01,
      5.484131866558092927e-01,  4.820861419893971433e-01,  4.527982958003512470e-01,  4.117483617150968778e-01,
      2.806440323263878489e-01,  2.135702100322256491e-01,  6.157222100494019745e-02,  -2.231975351354664738e-02,
      -1.651443676159378149e-01, -2.340084505865142050e-01, -3.256219537067972514e-01, -3.724334694397651857e-01,
      -4.303643056660655386e-01, -4.557463134637371183e-01, -4.462847237649191778e-01, -3.988352393924288197e-01,
      -4.015690422998912101e-01, -3.344401174497665941e-01, -2.838747384989886990e-01, -1.454873151899249706e-01,
      -4.881034931394508114e-02, -4.639136921804250246e-02, 2.614479227252786836e-02,  -4.835579329511428132e-02,
      -8.142036908175281917e-02, -8.934639055148757558e-02, 4.495389028957229652e-03,  1.937715893356594873e-02,
      -2.025761077642181454e-02, -1.801581939901750196e-02, 5.117343475582251944e-02,  1.541522625168581517e-01,
      1.219024258914298692e-01,  1.205056874557008939e-01,  1.815896943021886867e-01,  1.875034858657960712e-01,
      2.088911846734999023e-01,  1.524734281138133696e-01,  1.769489064978926052e-01,  1.247974616724564778e-01,
      1.128496870247042844e-01,  2.678146930839691089e-02,  -1.640259198955761749e-02, -7.712046404598811578e-02,
      -1.293410853907396296e-01, -1.967276747535141590e-01, -2.613412005831962093e-01, -2.995398836237181950e-01,
      -3.049667856904977881e-01, -3.542379613117136450e-01, -3.236524240698330779e-01, -3.384593457863483423e-01,
      -2.673837264478722275e-01, -2.628894136563670880e-01, -1.737884990662244555e-01, -1.332742711185166684e-01,
      -1.857037192545481219e-02, 4.316050128359554677e-02,  1.256497159895860749e-01,  2.190137103786391481e-01,
      2.981507536006987036e-01,  3.448408163078662270e-01,  3.839967471487597539e-01,  3.918710210983397113e-01,
      4.316802427871671011e-01,  3.887578933874620835e-01,  3.797495017380336790e-01,  2.889201142941014155e-01,
      2.756094017889403891e-01,  1.817352134672862440e-01,  1.037543616201775792e-01,  -2.714731836587163175e-02,
      -8.894697266669827906e-02, -1.109766168558555255e-01, -2.572937754549015632e-01, -3.566798126063410179e-01,
      -3.758079108009848568e-01, -3.397975188815649528e-01, -3.460012688631161626e-01, -4.198891970979794497e-01,
      -3.813563018558220885e-01, -3.081603885179572955e-01, -1.841531151297598479e-01, -1.998062468078471987e-01,
      3.520232519816158395e-02,  -1.236331976474966260e-01, -4.413314408414584916e-02, 4.925063647417166762e-02,
      2.886596799855360718e-02,  -6.947034512318209926e-03, -4.228782735678940363e-02, 4.369477759537340050e-02,
      1.112502685141738418e-01,  1.001667238730192949e-01,  4.713828426606734429e-02,  1.032757246128547385e-01,
      2.167497892472332321e-01,  1.639119539454443886e-01,  1.195388240450297179e-01,  7.173704731601088236e-02,
      1.769646234627801351e-01,  2.281169414828164987e-01,  2.164948435048865902e-02,  -5.773780223404040984e-02,
      5.242202674552712272e-02,  5.142686227240435282e-02,  -5.954388986444959092e-02, -1.775850389382033212e-01,
      -1.962284649742421505e-01, -1.352335773788961581e-01, -1.493747765049948106e-01, -2.945996403491220050e-01,
      -3.970552780597992881e-01, -2.708547192461389086e-01, -2.073203011434522069e-01, -3.181227731891986821e-01,
      -3.140129928122266101e-01, -2.358988534790014413e-01, -4.611781580196704783e-02, -3.720715297066544414e-02,
      -9.952231672320929756e-02, 5.561044082952244089e-02,  2.402577471478319071e-01,  2.797127776370690877e-01,
      2.291536686794735656e-01,  2.435644087513559874e-01,  3.434889569828610822e-01,  4.242451832603476025e-01,
      3.842336482472631931e-01,  2.237256738238149467e-01,  2.459507794445477957e-01,  3.892373036010145504e-01,
      2.699150230803641359e-01,  9.410028756940437555e-02,  6.972802472595349133e-02,  4.202505302228336237e-02,
      2.625742229828981894e-02,  -1.498566681499978459e-01, -2.588159376254779986e-01, -2.463271191971313778e-01,
      -2.611299087312873857e-01, -3.381590108506488113e-01, -4.167773001284572065e-01, -3.578248443496963582e-01,
      -2.844650483427195731e-01, -2.150746399221543093e-01, -2.526019753388205058e-01, -2.722275873343588670e-01,
      3.218458606854036202e-02,  -8.951352999798896370e-02, 1.750054189622553269e-02,  1.024666591977734659e-01,
      6.334713275705762536e-02,  -8.487682845267272708e-03, 7.467677576962440666e-02,  1.380814437605556444e-01,
      1.154968293717406197e-01,  6.525138328771483420e-02,  8.451822311162648926e-02,  1.472811279624131608e-01,
      1.763438975075016235e-01,  8.684779311590065665e-02,  6.470017882107796658e-02,  1.514146907832754074e-01,
      1.229020190454302025e-01,  8.059643148047028471e-02,  3.287543742598308949e-02,  2.114917183718285562e-02,
      -5.520569490555778808e-02, -6.544058396385245280e-02, -7.299782782972116801e-02, -1.730931277942356628e-01,
      -2.264789974030037845e-01, -1.949332486117194285e-01, -2.008187787560014870e-01, -2.787734550601250194e-01,
      -2.900980461043506931e-01, -2.615542932709719048e-01, -2.102954480207508414e-01, -2.357391303832436225e-01,
      -2.079452777000126429e-01, -1.557327308772371299e-01, -5.396474206309315874e-02, -3.475940777064515097e-02,
      -4.632703348813570752e-04, 6.389308095540653654e-02,  1.831881627150858671e-01,  2.179179652102991582e-01,
      2.020063470830844310e-01,  2.567668791983893084e-01,  3.434972116810321441e-01,  3.229050023209693032e-01,
      2.919723475865938500e-01,  3.156223065330357458e-01,  2.652150710232154540e-01,  2.456473578695576310e-01,
      2.185192878415106177e-01,  1.773707190098244568e-01,  2.205193929061282504e-02,  -2.070411452615782133e-02,
      8.344753492403805123e-03,  -7.627015013834255452e-02, -1.875730914118262926e-01, -2.466178841680749523e-01,
      -2.255453193839996318e-01, -2.196030332419116882e-01, -2.858439599188513647e-01, -3.573594848304572635e-01,
      -2.598841445989664711e-01, -1.839935206870050577e-01, -2.234700742909233318e-01, -2.784454281941622367e-01,
      6.745271665576166309e-02,  -1.179619195913448049e-01, -5.368820936908742791e-02, 2.514882822603049517e-01,
      2.202429136992057612e-01,  -5.479308301707361273e-02, -9.458999145925801877e-02, 1.898722104786342291e-01,
      3.676511716021294007e-01,  1.153441379386542509e-01,  -1.212049748928953341e-01, 5.604307569624749491e-02,
      3.553990316379467829e-01,  2.327796851794284028e-01,  -9.190967321827155090e-02, -9.107047890801112100e-02,
      2.083160714819293213e-01,  2.775789143693890848e-01,  -1.096528459174175435e-01, -2.849446862276554526e-01,
      -2.353935163970034133e-02, 1.774639126699177061e-01,  -1.332750200958857656e-01, -4.481135896599743873e-01,
      -2.952803668520708191e-01, 7.541037369351690292e-04,  -9.084400395694865182e-02, -4.478386952524024966e-01,
      -4.832015065785014163e-01, -8.631483353867601405e-02, 7.363086510868833978e-02,  -2.290374422636484442e-01,
      -4.106894060561265269e-01, -1.508877852866644109e-01, 2.282685511545913259e-01,  1.403016730876339713e-01,
      -1.913979112366274438e-01, -1.010740486733938126e-01, 2.983696210801613513e-01,  4.180404988697420388e-01,
      1.347714708154428287e-01,  -2.062042517153607479e-02, 2.768375673851934393e-01,  5.569266068834646255e-01,
      3.138701490900753233e-01,  2.398661862723603343e-03,  1.214728779133942560e-01,  4.476920143829548060e-01,
      3.137022019550284035e-01,  -5.108653613810573008e-02, -1.166404717501257349e-01, 1.455806446218592953e-01,
      2.099099344896189767e-01,  -1.420163566625512186e-01, -3.637013598028356887e-01, -1.626057590592031865e-01,
      6.359949985389677651e-02,  -1.305006265652049191e-01, -4.215578170826986271e-01, -3.794207956290854833e-01,
      -9.322537867894475916e-02, -4.192309441730865394e-02, -3.175618585109141190e-01, -3.425346573356718438e-01,
      8.645565026673478551e-03,  1.512376474440250973e-01,  1.458918039304007153e-01,  2.726883843379072567e-02,
      8.299970907243818530e-02,  1.564608413936748599e-01,  1.592333130186406598e-01,  1.067459126011479076e-01,
      8.339621880207062632e-02,  1.501077289245447699e-01,  1.994768983339336443e-01,  1.450767914911553214e-01,
      3.099751724389636018e-02,  5.192467408992620242e-02,  1.415345293116217484e-01,  1.188371884138934576e-01,
      -2.641360591396722107e-02, -8.677789293168254592e-02, -8.798364527554451003e-03, 2.839843965419402289e-02,
      -1.584068576097380776e-01, -2.342469712274244664e-01, -1.425544937548147706e-01, -9.323281730515980970e-02,
      -1.805475220414123061e-01, -3.184891912456469520e-01, -2.978850118331576313e-01, -1.681613055544497470e-01,
      -1.090560745386705555e-01, -1.994741466137849861e-01, -2.659337483553795844e-01, -1.192657226468444298e-01,
      -3.378293840709086643e-03, -6.829444568499420942e-02, -1.600791627699673092e-01, -3.382756710068388073e-02,
      1.202917271004480121e-01,  1.247138511018138496e-01,  5.199308095974266436e-02,  7.530920049185378662e-02,
      2.397735004136512527e-01,  3.344341737505284740e-01,  2.739026097771503632e-01,  1.560677124957531858e-01,
      1.951729758107156043e-01,  3.377457264450920982e-01,  2.502103850347778269e-01,  1.174659774881786356e-01,
      1.176484618429071843e-01,  1.966751605866033925e-01,  1.482841170495079464e-01,  -1.425685649707205023e-02,
      -1.052625018576391902e-01, -5.322291849506893824e-02, -4.845684365737801952e-02, -1.320829206282782831e-01,
      -2.180858461881967914e-01, -2.016302383570082735e-01, -1.469466244754474016e-01, -1.418765338233266282e-01,
      -2.039267499901066261e-01, -2.444758186495680496e-01, -1.050980633539001924e-01, -7.307108491904883152e-02,
    };

    {
        auto [psi, x] = integrate_wavelet<double>(cwt_wavelet_t::MORLET, 10);
        auto r        = cwt<double>(data, scales, psi, x);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); i++) {
            double diff = std::abs(r[i] - expected[i]);
            assert(diff < 1e-14);
        }
    }

    {
        auto [psi, x] = integrate_wavelet<float>(cwt_wavelet_t::MORLET, 10);
        auto r        = cwt<float>(data, scales, psi, x);
        assert(r.size() == expected.size());
        for (size_t i = 0; i < r.size(); i++) {
            double diff = std::abs(static_cast<double>(r[i]) - expected[i]);
            assert(diff < 1e-5);
        }
    }
}

#endif

}  // namespace tests

}  // namespace dsp
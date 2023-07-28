
#include "app_imu.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <tuple>

#include "test_data.h"

#define LODA_COMPILE true
#define KMEANS_COMPILE false

// inspired by https://github.com/yzhao062/pyod
class Anomaly
{
protected:
  float contamination;
  float threshold_;
  float score_;
  std::vector<int> labels_;

public:
  Anomaly(float contamination) : contamination(contamination) {}

  virtual void fit(std::vector<std::vector<float>> &X) = 0;

  virtual std::vector<float> decision_function(std::vector<std::vector<float>> &X) = 0;

  std::vector<int> predict(std::vector<std::vector<float>> &X)
  {
    std::vector<float> scores = decision_function(X);
    std::vector<int> labels(X.size(), 0);
    for (int i = 0; i < X.size(); ++i)
    {
      if (scores[i] >= threshold_)
      {
        labels[i] = 1;
      }
    }
    return labels;
  }

  virtual ~Anomaly() {}
};

#if LODA_COMPILE
class LODA : public Anomaly
{
private:
  int n_bins;
  int n_random_cuts;
  std::vector<float> weights;
  std::vector<std::vector<float>> projections_;
  std::vector<float> decision_scores_;
  std::vector<std::vector<float>> histograms_;
  std::vector<std::vector<float>> limits_;

public:
  LODA(float contamination, int n_bins, int n_random_cuts)
      : Anomaly(contamination), n_bins(n_bins), n_random_cuts(n_random_cuts)
  {
    weights = std::vector<float>(n_random_cuts, 1.0 / n_random_cuts);
  }

  void fit(std::vector<std::vector<float>> &X) override
  {
    int n_samples = X.size();
    int n_features = X[0].size();
    decision_scores_ = std::vector<float>(n_samples, 0.0);

    float n_nonzero_components = std::sqrt(n_features);
    int n_zero_components = n_features - static_cast<int>(n_nonzero_components);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(0.0, 1.0);

    projections_ =
        std::vector<std::vector<float>>(n_random_cuts, std::vector<float>(n_features));

    // Generate random projections
    for (int i = 0; i < n_random_cuts; ++i)
      for (int j = 0; j < n_features; ++j)
        projections_[i][j] = dis(gen);

    // Randomly set some components to zero
    for (int i = 0; i < n_random_cuts; ++i)
    {
      for (int j = 0; j < n_zero_components; ++j)
      {
        int rand_idx = std::uniform_int_distribution<>(0, n_features - 1)(gen);
        projections_[i][rand_idx] = 0.0;
      }
    }

    // Calculate decision scores
    for (int i = 0; i < n_random_cuts; ++i)
    {
      std::vector<float> projected_data(n_samples, 0.0);

      // Calculate dot product of each sample with the projection vector
      for (int j = 0; j < n_samples; ++j)
      {
        float dot_product = 0.0;
        for (int k = 0; k < n_features; ++k)
        {
          dot_product += projections_[i][k] * X[j][k];
        }
        projected_data[j] = dot_product;
      }

      int curr_n_bins = n_bins;
      if (n_bins == -1)
      {
        // Birge-Rozenblac method for determining the optimal number of bins
        // Not implemented in C++ code
      }

      std::vector<float> histogram(curr_n_bins, 1e-12);
      std::vector<float> limits(curr_n_bins + 1, 0.0);
      float min_val = *std::min_element(projected_data.begin(), projected_data.end());
      float max_val = *std::max_element(projected_data.begin(), projected_data.end());
      limits[0] = min_val;
      limits[curr_n_bins] = max_val;

      for (int j = 1; j < curr_n_bins; ++j)
      {
        limits[j] = min_val + j * (max_val - min_val) / curr_n_bins;
      }

      std::vector<int> inds(n_samples);
      for (int j = 0; j < n_samples; ++j)
      {
        inds[j] = std::upper_bound(limits.begin(), limits.end() - 1, projected_data[j]) -
                  limits.begin() - 1;
        if (inds[j] < 0)
        {
          inds[j] = 0;
        }
        else if (inds[j] >= curr_n_bins)
        {
          inds[j] = curr_n_bins - 1;
        }
      }

      for (int j = 0; j < n_samples; ++j)
      {
        histogram[inds[j]] += 1.0;
      }

      for (int j = 0; j < curr_n_bins; ++j)
      {
        histogram[j] /= n_samples;
      }

      for (int j = 0; j < n_samples; ++j)
      {
        decision_scores_[j] += -weights[i] * std::log(histogram[inds[j]]);
      }
      histograms_.push_back(histogram);
      limits_.push_back(limits);
    }

    for (int i = 0; i < n_samples; ++i)
    {
      decision_scores_[i] /= n_random_cuts;
    }

    // Calculate threshold based on contamination
    int n_outliers = static_cast<int>(n_samples * contamination);
    std::vector<float> sorted_scores = decision_scores_;
    std::sort(sorted_scores.begin(), sorted_scores.end());
    threshold_ = sorted_scores[n_samples - n_outliers];

    // Calculate labels
    labels_ = std::vector<int>(n_samples, 0);
    for (int i = 0; i < n_samples; ++i)
    {
      if (decision_scores_[i] >= threshold_)
      {
        labels_[i] = 1;
      }
    }
  }

  std::vector<float> decision_function(std::vector<std::vector<float>> &X) override
  {
    int n_samples = X.size();
    std::vector<float> pred_scores(n_samples, 0.0);

    for (int i = 0; i < n_random_cuts; ++i)
    {
      std::vector<float> projected_data(n_samples, 0.0);
      for (int j = 0; j < n_samples; ++j)
      {
        float dot_product = 0.0;
        for (int k = 0; k < X[j].size(); ++k)
        {
          dot_product += projections_[i][k] * X[j][k];
        }
        projected_data[j] = dot_product;
      }

      int curr_n_bins = n_bins;
      if (n_bins == -1)
      {
        // Birge-Rozenblac method for determining the optimal number of bins
        // Not implemented in C++ code
      }

      std::vector<int> inds(n_samples);
      for (int j = 0; j < n_samples; ++j)
      {
        inds[j] =
            std::upper_bound(limits_[i].begin(), limits_[i].end() - 1, projected_data[j]) -
            limits_[i].begin() - 1;
        if (inds[j] < 0)
        {
          inds[j] = 0;
        }
        else if (inds[j] >= curr_n_bins)
        {
          inds[j] = curr_n_bins - 1;
        }
      }

      for (int j = 0; j < n_samples; ++j)
      {
        pred_scores[j] += -weights[i] * std::log(histograms_[i][inds[j]]);
      }
    }

    for (int i = 0; i < n_samples; ++i)
    {
      pred_scores[i] /= n_random_cuts;
    }

    return pred_scores;
  }
};
#endif
#if KMEANS_COMPILE
class K_Means : public Anomaly
{
private:
  int num_cuts;
  int num_bins;
  int cluster_num;
  int max_iteration;
  std::vector<std::vector<float>> means;
  std::vector<float> means_r;
  // std::tuple<uint16_t,float> test;
  // std::vector<std::vector<float>> *Data;

public:
  K_Means(float p, int cluster_num, int max_iteration) : Anomaly(p), cluster_num(cluster_num), max_iteration(max_iteration)
  // params init
  {
    means_r.resize(cluster_num);
    std::fill(means_r.begin(), means_r.end(), 0);
  };
  std::vector<float> decision_function(std::vector<std::vector<float>> &data)
  {

    std::vector<int> labels(data.size(), 0); // normal
    std::vector<std::tuple<uint16_t, float>> clusters = K_Means::calculate_clusters(data);
    for (int i = 0; i < data.size(); i++)
    {
      if (std::get<1>(clusters[i]) > (means_r[std::get<0>(clusters[i])]))
      {
        labels[i] = 1; // abnormal
      }
    }
    return labels;
  };

  std::vector<int> predict(const std::vector<std::vector<float>> &data)
  // predict infrence
  {
    std::vector<int> labels(data.size(), 0); // normal
    std::vector<std::tuple<uint16_t, float>> clusters = K_Means::calculate_clusters(data);
    for (int i = 0; i < data.size(); i++)
    {
      if (std::get<1>(clusters[i]) > (means_r[std::get<0>(clusters[i])]))
      {
        labels[i] = 1; // abnormal
      }
    }
    return labels;
  }

  void fit(std::vector<std::vector<float>> &data) override
  // train model params
  {
    means.resize(cluster_num, std::vector<float>(data[0].size()));
    // Data = &data;
    K_Means::random_plusplus(data); // init cluster center point

    std::vector<std::vector<float>> pre_means;
    std::vector<std::vector<float>> pre_pre_means;
    std::vector<std::tuple<uint16_t, float>> clusters;
    float min_distance = -1.0;
    uint8_t count = 0;
    do // start train model
    {

      vTaskDelay(16 / portTICK_PERIOD_MS);
      // calculater cluster center of with every data
      clusters = K_Means::calculate_clusters(data);
      vTaskDelay(16 / portTICK_PERIOD_MS);
      pre_pre_means = pre_means;
      pre_means = means;
      // recalculate means point
      K_Means::move_means(clusters, pre_means, data);
      vTaskDelay(16 / portTICK_PERIOD_MS);
      ++count;
    } while (means != pre_means && means != pre_pre_means && !(max_iteration && count == max_iteration) && !min_distance_below(pre_means, min_distance));
    calculate_means_r(clusters);
  };
  bool min_distance_below(std::vector<std::vector<float>> &pre_means, float &min_distance)
  {
    std::vector<float> distance(means.size(), 0);
    for (int i = 0; i < means.size(); i++)
    {
      distance.push_back(distance_sqrt(means[i], pre_means[i]));
    }
    for (auto &d : distance)
    {
      if (d > min_distance)
      {
        return false;
      };
    }
    return true;
  }
  void calculate_means_r(std::vector<std::tuple<uint16_t, float>> &clusters)
  // calculate max r of every means
  {
    for (auto &cl : clusters)
    {
      if (means_r[std::get<0>(cl)] < std::get<1>(cl))
      {
        means_r[std::get<0>(cl)] = std::get<1>(cl);
      }
    }
  }

  void move_means(const std::vector<std::tuple<uint16_t, float>> &clusters, const std::vector<std::vector<float>> &pre_means, std::vector<std::vector<float>> data)
  // recalculate means point
  {
    std::vector<uint16_t> count(clusters.size(), 0);
    // reset means to 0
    std::fill(means.begin(), means.end(), std::vector<float>(means[0].size(), 0.0));
    for (int i = 0; i < data.size(); i++)
    {
      auto &mean = means[std::get<0>(clusters[i])];

      for (int j = 0; j < mean.size(); j++)
      {
        mean[j] += data[i][j];
      }
    }
    for (int i = 0; i < means.size(); i++)
    {
      if (count[i] == 0)
      {
        means[i] = pre_means[i];
        continue;
      }
      for (int j = 0; j < means[0].size(); j++)
      {
        means[i][j] /= count[i];
      }
    }
  };

  std::vector<std::tuple<uint16_t, float>> calculate_clusters(const std::vector<std::vector<float>> &data)
  // calculater every data point with in where cluster index and distance
  {
    std::vector<std::tuple<uint16_t, float>> cluster;

    for (auto &point : data)
    {
      cluster.push_back(closest_mean(point));
    }

    return cluster;
  }

  std::tuple<uint16_t, float> closest_mean(const std::vector<float> &point)
  // calculater cluster index and distance
  {
    // caculer the closest distance of point with means ,
    // return the index of data and smallest distance
    float smallest_distance = K_Means::distance_sqrt(point, means[0]);
    uint16_t index = 0;
    float distance;
    for (uint16_t i = 1; i < means.size(); i++)
    {
      distance = K_Means::distance_sqrt(means[i], point);
      if (distance < smallest_distance)
      {
        smallest_distance = distance;
        index = i;
      }
    }
    // std::vector<float> res(2);

    return std::tuple<uint16_t, float>(index, smallest_distance);
  }

  std::vector<float> decision_function(const std::vector<std::vector<float>> &X)
  {
    std::vector<float> t;
    return t;
  };
  void random_plusplus(const std::vector<std::vector<float>> &data)
  // init cluster cent point
  {
    std::random_device rand_device;
    std::linear_congruential_engine<uint16_t, 16807, 0, UINT16_MAX> rand_engine(rand_device());

    {
      std::uniform_int_distribution<uint16_t> uniform_generator(0, data.size() - 1);
      means.push_back(data[uniform_generator(rand_engine)]);
    }
    std::mt19937 rng(std::random_device{}());
    for (int i = 1; i < cluster_num; ++i)
    {

      std::vector<float> distances = K_Means::closest_distance(data);
      std::discrete_distribution<uint16_t> generator(distances.begin(), distances.end());
      means.push_back(data[generator(rng)]);
    }
  }
  std::vector<float> closest_distance(const std::vector<std::vector<float>> &data)
  {
    std::vector<float> distances;
    distances.reserve(data.size());
    for (auto &d : data)
    {
      float closet = K_Means::distance_sqrt(d, means[0]);
      for (auto &m : means)
      {
        float distance = K_Means::distance_sqrt(d, m);
        if (distance < closet)
        {
          closet = distance;
        }
      }
      distances.push_back(closet);
    }
    return distances;
  }

  float distance_sqrt(const std::vector<float> &A, const std::vector<float> &B)
  {
    float distance_ = 0;
    for (int i = 0; i < A.size(); i++)
    {
      auto delta = A[i] - B[i];
      distance_ += delta * delta;
    }
    return sqrt(distance_);
  }
};

#endif

extern "C" void app_main()
{
  qma7981_init();
  qma7981_set_range(QMA_RANGE_8G);
  std::vector<std::vector<float>> X;
  std::cout << "Collecting data..." << std::endl;
  for (int i = 0; i < 100; ++i)
  {
    std::cout << "Iteration " << i << std::endl;
    std::vector<float> row = std::vector<float>(30 * 3);
    for (int j = 0; j < 30; ++j)
    {
      float x, y, z;
      // std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
      qma7981_get_acce(&x, &y, &z);
      row[j * 3] = x * 9.8;
      row[j * 3 + 1] = y * 9.8;
      row[j * 3 + 2] = z * 9.8;
      vTaskDelay(16 / portTICK_PERIOD_MS);
    }
    X.push_back(row);
  }
#if LODA_COMPILE
  float contamination = 0.05;
  int n_bins = 10;
  int n_random_cuts = 100;
  LODA detector(contamination, n_bins, n_random_cuts);
#elif KMEANS_COMPILE
  int cluster_num = 16;
  int max_iteration = 20;
  float contamination = 0.0;
  K_Means detector(contamination, cluster_num, max_iteration);
#elif
#endif

  // Fit the model
  std::cout << "Fitting model..." << std::endl;
  detector.fit(X);

  std::cout << "Predicting..." << std::endl;
  while (1)
  {
    std::vector<std::vector<float>> Y;
    for (int i = 0; i < 1; ++i)
    {
      std::vector<float> row = std::vector<float>(30 * 3);
      for (int j = 0; j < 30; ++j)
      {
        float x, y, z;
        qma7981_get_acce(&x, &y, &z);
        // std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
        row[j * 3] = x * 9.8;
        row[j * 3 + 1] = y * 9.8;
        row[j * 3 + 2] = z * 9.8;
        vTaskDelay(16 / portTICK_PERIOD_MS);
      }
      Y.push_back(row);
    }
    auto scores = detector.predict(Y);
    for (int i = 0; i < scores.size(); ++i)
    {
      std::cout << xTaskGetTickCount() << ": Score: " << scores[i] << std::endl;
    }
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
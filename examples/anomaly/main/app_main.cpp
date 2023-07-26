
#include "app_imu.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "test_data.h"

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

  float contamination = 0.05;
  int n_bins = 10;
  int n_random_cuts = 100;

  LODA detector(contamination, n_bins, n_random_cuts);

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
    auto scores = detector.decision_function(Y);
    for (int i = 0; i < scores.size(); ++i)
    {
      std::cout << xTaskGetTickCount() << ": Score: " << scores[i] << std::endl;
    }
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
#include<fstream>
#include<chrono>
#include<iostream>
#include<cstdlib>  
#include<limits>  
#include <algorithm>
#include"stdafx.h"
#include"common.h"
#include<opencv2/core/utils/logger.hpp>
#include<opencv2/core.hpp>
#include<opencv2/calib3d.hpp>
#include"sos.h"


wchar_t* projectPath;
using namespace std;
using namespace std::chrono;



float computeSAD(const Mat& left, const Mat& right, int x, int y, int d, int level) {
    float sad = 0.0;
    int count = 0;

    if (x - d < 0) {
        return FLT_MAX;
    }

    for (int i = -level / 2; i <= level / 2; i++) {
        for (int j = -level / 2; j <= level / 2; j++) {
            int left_x = x + j, left_y = y + i;
            int right_x = left_x - d, right_y = left_y;

            if (left_x >= 0 && left_x < left.cols && left_y >= 0 && left_y < left.rows &&
                right_x >= 0 && right_x < right.cols && right_y >= 0 && right_y < right.rows) {

                Vec3b left_pixel = left.at<Vec3b>(left_y, left_x);
                Vec3b right_pixel = right.at<Vec3b>(right_y, right_x);

                sad += abs(left_pixel[0] - right_pixel[0]) +
                    abs(left_pixel[1] - right_pixel[1]) +
                    abs(left_pixel[2] - right_pixel[2]);

                count++;
            }
        }
    }

    return (count > 0) ? sad / count : FLT_MAX;
}



void computeInitialHypotheses(const Mat left, const Mat right, vector<vector<int>>& hypotheses, int max_disparity) {

    int rows = left.rows;
    int cols = left.cols;
    hypotheses.resize(rows, vector<int>(cols, 0));

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {

            int random_disparity;
            float cost;
            float min_cost = FLT_MAX;
            int best_disparity = 0;

            for (int d = 0; d < 4; d++) {

                random_disparity = rand() % (max_disparity + 1);
                cost = computeSAD(left, right, x, y, random_disparity, 1);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_disparity = random_disparity;
                }
            }

            hypotheses[y][x] = best_disparity;

        }
    }

}



void computeHypotheses(Mat left, Mat right, vector<vector<int>>& hypotheses) {
    int rows = left.rows;
    int cols = left.cols;
    int level = 2;

    while (level <= 16) {
        for (int y = 0; y < rows; y += level) {
            for (int x = 0; x < cols; x += level) {
                float best_cost = FLT_MAX;
                int best_disparity = 0;

                vector<int> disparities = {
                    hypotheses[y][x],
                    hypotheses[y][x + level / 2],
                    hypotheses[y + level / 2][x],
                    hypotheses[y + level / 2][x + level / 2]
                };

                for (int i = 0; i < 4; i++) {
                    float cost = computeSAD(left, right, x, y, disparities[i], level);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_disparity = disparities[i];
                    }
                }

                for (int yy = y; yy < y + level && yy < rows; yy++) {
                    for (int xx = x; xx < x + level && xx < cols; xx++) {
                        hypotheses[yy][xx] = best_disparity;
                    }
                }
            }
        }

        level *= 2;
    }
}

void refineDisparity(Mat left, Mat right, vector<vector<float>>& refined_disp, vector<vector<int>>& hypotheses) {
    int rows = left.rows;
    int cols = left.cols;
    refined_disp.resize(rows, vector<float>(cols, 0.0f));

    for (int y = 0; y < rows; y += 16) {
        for (int x = 0; x < cols; x += 16) {

            if ((x - hypotheses[y][x] - 1) >= 0 && (x - hypotheses[y][x] + 1) <= cols) {

                float cost = computeSAD(left, right, x, y, hypotheses[y][x], 16);
                float cost_plus = computeSAD(left, right, x, y, hypotheses[y][x] + 1, 16);
                float cost_minus = computeSAD(left, right, x, y, hypotheses[y][x] - 1, 16);
                float refined_disparity = 0.f;

                if (2 * (cost_minus - 2 * cost + cost_plus) == 0)
                    refined_disparity = hypotheses[y][x];
                else
                    refined_disparity = hypotheses[y][x] + (cost_minus - cost_plus) / (2 * (cost_minus - 2 * cost + cost_plus));
                if (refined_disparity < 0) refined_disparity = 0;

                for (int yy = y; yy < y + 16; yy++) {
                    for (int xx = x; xx < x + 16; xx++) {
                        refined_disp[yy][xx] = refined_disparity;

                    }
                }

            }
        }
    }



 
}



void refineDisparityWithSlant(Mat left, Mat right, vector<vector<float>>& refined_disp, vector<vector<int>>& hypotheses) {
    int rows = left.rows;
    int cols = left.cols;

   

    for (int y = 0; y < rows; y += 16) {
        for (int x = 0; x < cols; x += 16) {

            int center_x = x + 16 / 2;
            int center_y = y + 16 / 2;

            if (center_x >= cols || center_y >= rows) continue;

            int d_center = hypotheses[center_y][center_x];


            float dx = 0.0f, dy = 0.0f;
            float tilt_angle = 30.0f * CV_PI / 180.0f;
            float tan_tilt = tan(tilt_angle);

            float cost = computeSAD(left, right, center_x, center_y, d_center, 16);
            float cost_dx = computeSAD(left, right, center_x, center_y, d_center + tan_tilt, 16);
            float cost_dy = computeSAD(left, right, center_x, center_y, d_center - tan_tilt, 16);

            dx = (cost - cost_dx) / tan_tilt;
            dy = (cost - cost_dy) / tan_tilt;

            for (int yy = y; yy < y + 16; yy++) {
                for (int xx = x; xx < x + 16; xx++) {

                    int kx = xx - center_x;
                    int ky = yy - center_y;

                    float disparity = d_center + kx * dx + ky * dy;

                    if (disparity < 0) disparity = 0;
                    if (disparity > 255) disparity = 255;

                    refined_disp[yy][xx] = disparity;
                }
            }
        }
    }

 
}


float computePairCost(float disp_i, float disp_j, int error) {

    float difference = abs(disp_i - disp_j);
    if (difference > error) {
        difference = error;
    }

    return difference;
}


void propagateAndInfer(Mat left, Mat right, vector<vector<float>>& refined_disp) {

    int rows = left.rows;
    int cols = left.cols;
    int block_size = 16;
    int error = 10;

    for (int iter = 0; iter < 2; iter++) {
        for (int y = 0; y < rows; y += block_size) {
            for (int x = 0; x < cols; x += block_size) {

                int center_x = x + block_size / 2;
                int center_y = y + block_size / 2;

                if (center_x >= cols || center_y >= rows) continue;

                float best_energy = FLT_MAX;
                float best_disparity = refined_disp[center_y][center_x];

                vector<vector<int>> neighbors = {
                    {y - block_size, x},    // sus
                    {y + block_size, x},    // jos
                    {y, x - block_size},    // stanga
                    {y, x + block_size}     // dreapta
                };

                vector<float> candidate_disparities = { refined_disp[center_y][center_x] };



                for (int i = 0; i < neighbors.size(); i++) {
                    int ny = neighbors[i][0];
                    int nx = neighbors[i][1];

                    if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                        candidate_disparities.push_back(refined_disp[ny + block_size / 2][nx + block_size / 2]);
                    }
                }


                for (int i = 0; i < candidate_disparities.size(); i++) {
                    float candidate_d = candidate_disparities[i];

                    float unary_cost = computeSAD(left, right, center_x, center_y, candidate_d, block_size);
                    float pairwise_cost = 0.0f;

                    for (int j = 0; j < neighbors.size(); j++) {
                        int ny = neighbors[j][0];
                        int nx = neighbors[j][1];

                        if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                            float neighbor_disparity = refined_disp[ny + block_size / 2][nx + block_size / 2];
                            pairwise_cost += computePairCost(candidate_d, neighbor_disparity, error);
                        }
                    }

                    float total_energy = unary_cost + pairwise_cost;

                    if (total_energy < best_energy) {
                        best_energy = total_energy;
                        best_disparity = candidate_d;
                    }
                }

                for (int yy = y; yy < y + 16; yy++) {
                    for (int xx = x; xx < x + 16; xx++) {
                        refined_disp[yy][xx] = best_disparity;
                    }
                }
            }
        }
    }

    
}

void refinePerPixelEstimation(const Mat& left, const Mat& right, vector<vector<float>>& refined_disp) {
    int rows = left.rows;
    int cols = left.cols;
    int tile_size = 16;
    int patch_size = 11;

    vector<vector<float>> new_disp(rows, vector<float>(cols, 0));

    for (int y = 0; y < rows; y += tile_size) {
        for (int x = 0; x < cols; x += tile_size) {

            int center_y = y + tile_size / 2;
            int center_x = x + tile_size / 2;

            if (center_y >= rows || center_x >= cols) continue;


            vector<float> candidate_disps;


            candidate_disps.push_back(refined_disp[center_y][center_x]);


            if (center_y - tile_size >= 0)
                candidate_disps.push_back(refined_disp[center_y - tile_size][center_x]);

            if (center_y + tile_size < rows)
                candidate_disps.push_back(refined_disp[center_y + tile_size][center_x]);

            if (center_x - tile_size >= 0)
                candidate_disps.push_back(refined_disp[center_y][center_x - tile_size]);

            if (center_x + tile_size < cols)
                candidate_disps.push_back(refined_disp[center_y][center_x + tile_size]);

            for (int yy = y; yy < y + tile_size && yy < rows; yy++) {
                for (int xx = x; xx < x + tile_size && xx < cols; xx++) {

                    float best_disp = 0;
                    float min_cost = FLT_MAX;


                    for (float d : candidate_disps) {
                        int rounded_d = static_cast<int>(d);
                        float cost = computeSAD(left, right, xx, yy, rounded_d, 1);

                        if (cost < min_cost) {
                            min_cost = cost;
                            best_disp = d;
                        }
                    }

                    new_disp[yy][xx] = best_disp;
                }
            }
        }
    }

    refined_disp = new_disp;

   
}



void displayDisparityMap(vector<vector<int>>& hypotheses, Mat& disparityMap) {
    int rows = hypotheses.size();
    int cols = hypotheses[0].size();

    disparityMap = Mat(rows, cols, CV_8UC1);

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            disparityMap.at<uchar>(y, x) = static_cast<uchar>(hypotheses[y][x]);
        }
    }

    normalize(disparityMap, disparityMap, 0, 255, NORM_MINMAX);

    Mat resizedMap;
    resize(disparityMap, resizedMap, Size(), 0.6, 0.5);

    imshow("Disparity Map", resizedMap);
    //imwrite("disp.png", resizedMap);
}

void displayRefinedDisparityMap(vector<vector<float>>& refined_disparities, Mat& disparityMap) {
    int rows = refined_disparities.size();
    int cols = refined_disparities[0].size();

    disparityMap = Mat(rows, cols, CV_8UC1);

    float min_disp = FLT_MAX, max_disp = FLT_MIN;
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            float disparity = refined_disparities[y][x];
            if (disparity < min_disp) min_disp = disparity;
            if (disparity > max_disp) max_disp = disparity;
        }
    }


    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            float disparity = refined_disparities[y][x];

            uchar pixel_value = static_cast<uchar>(255.0f * (disparity - min_disp) / (max_disp - min_disp));
            if (pixel_value >= 20)
                pixel_value = pixel_value - 20;
            else
                pixel_value = 0;
            disparityMap.at<uchar>(y, x) = pixel_value ;
        }
    }

    
    //imwrite("disparity_refined.png", disparityMap);
}

void applyMedianFilter(vector<vector<float>>& disparityMap, int windowSize) {
    int rows = disparityMap.size();
    int cols = disparityMap[0].size();
    int half = windowSize / 2;

    vector<vector<float>> filtered(rows, vector<float>(cols, 0.0f));

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            vector<float> neighbors;

            for (int dy = -half; dy <= half; dy++) {
                for (int dx = -half; dx <= half; dx++) {
                    int ny = y + dy;
                    int nx = x + dx;

                    if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                        neighbors.push_back(disparityMap[ny][nx]);
                    }
                }
            }

            int mid = neighbors.size() / 2;
            nth_element(neighbors.begin(), neighbors.begin() + mid, neighbors.end());
            filtered[y][x] = neighbors[mid];
        }
    }

    disparityMap = filtered;
}

void applyGaussian(vector<vector<float>>& data, int blurSize = 10, double blurStrength = 3.0) {
    if (blurSize < 3) blurSize = 3;
    if (blurSize % 2 == 0) blurSize += 1;

    int rows = data.size();
    int cols = data[0].size();

    Mat input(rows, cols, CV_32F);
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            input.at<float>(y, x) = data[y][x];

    Mat blurred;
    GaussianBlur(input, blurred, Size(blurSize, blurSize), blurStrength);

    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            data[y][x] = blurred.at<float>(y, x);
}

void applyWLS(const Mat& guideImage, vector<vector<float>>& disparity, int windowSize = 5, float lambda = 8000.0f, float sigma = 1.5f) {
    int rows = disparity.size();
    int cols = disparity[0].size();
    int half = windowSize / 2;

    Mat gray;
    if (guideImage.channels() == 3)
        cvtColor(guideImage, gray, COLOR_BGR2GRAY);
    else
        guideImage.copyTo(gray);

    gray.convertTo(gray, CV_32F, 1.0 / 255.0);

    vector<vector<float>> smoothed(rows, vector<float>(cols, 0.0f));

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float sum = 0.0f;
            float weightSum = 0.0f;

            float centerIntensity = gray.at<float>(y, x);


            for (int dy = -half; dy <= half; ++dy) {
                for (int dx = -half; dx <= half; ++dx) {
                    int yy = y + dy;
                    int xx = x + dx;

                    if (yy >= 0 && yy < rows && xx >= 0 && xx < cols) {
                        float neighborIntensity = gray.at<float>(yy, xx);
                        float intensityDiff = centerIntensity - neighborIntensity;

                        float spatialWeight = exp(-(dx * dx + dy * dy) / (2 * sigma * sigma));
                        float colorWeight = exp(-(intensityDiff * intensityDiff) * lambda);
                        float weight = spatialWeight * colorWeight;

                        sum += weight * disparity[yy][xx];
                        weightSum += weight;
                    }
                }
            }

            smoothed[y][x] = (weightSum > 0.0f) ? (sum / weightSum) : disparity[y][x];
        }
    }

    disparity = smoothed;
}






void createDepthMap(Mat disparityMap, Mat depthMap, double focal_length, double baseline) {

    depthMap = Mat(disparityMap.size(), CV_64FC1);

    for (int y = 0; y < disparityMap.rows; y++) {
        for (int x = 0; x < disparityMap.cols; x++) {
            uchar disparity = disparityMap.at<uchar>(y, x);

            if (disparity > 0) {
                double depth = (focal_length * baseline) / static_cast<double>(disparity);
                depthMap.at<double>(y, x) = depth;
            }
            else {
                depthMap.at<double>(y, x) = 0;
            }
        }
    }

    Mat normalizedDepthMap;
    normalize(depthMap, normalizedDepthMap, 0, 255, NORM_MINMAX);
    normalizedDepthMap.convertTo(normalizedDepthMap, CV_8UC1);

    Mat colorMap;
    applyColorMap(normalizedDepthMap, colorMap, COLORMAP_MAGMA);
    resize(colorMap, colorMap, Size(), 0.6, 0.5);

    imshow("Depth Map", colorMap);

}

float computeScore(Mat& disparityMap, Mat& groundTruth) {

    int score = 0;

    for (int y = 0; y < disparityMap.rows; y++) {
        for (int x = 0; x < disparityMap.cols; x++) {
            if (abs(disparityMap.at<uchar>(y, x) - groundTruth.at<uchar>(y, x)) < 55) {
                score++;
            }
        }
    }

    return (float)score / (float(disparityMap.rows * disparityMap.cols));
}


Mat generateErrorVisualization(Mat& disparityMap, Mat& gt) {
    CV_Assert(disparityMap.size() == gt.size());
    CV_Assert(disparityMap.type() == CV_8UC1 && gt.type() == CV_8UC1);

    Mat errorMap(disparityMap.size(), CV_8UC3);  

    for (int y = 0; y < disparityMap.rows; y++) {
        for (int x = 0; x < disparityMap.cols; x++) {
            uchar dispVal = disparityMap.at<uchar>(y, x);
            uchar gtVal = gt.at<uchar>(y, x);
            int diff = abs(dispVal - gtVal);

            if (diff > 150) {
                errorMap.at<Vec3b>(y, x) = Vec3b(0, 0, 100);
            }
            else if (diff > 120) {
                
                errorMap.at<Vec3b>(y, x) = Vec3b(0, 0, 150);
            }
            else if (diff >= 65) {
               
                errorMap.at<Vec3b>(y, x) = Vec3b(0, 0, 200);
            }
            else {
                errorMap.at<Vec3b>(y, x) = Vec3b(dispVal, dispVal, dispVal);
            }
        }
    }

    return errorMap;
}

void initialization(Mat left, Mat right, vector<vector<int>>& hypotheses, int max_disparity) {

    computeInitialHypotheses(left, right, hypotheses, max_disparity);
    computeHypotheses(left, right, hypotheses);
}

void refinementAndOptimization(Mat left, Mat right, vector<vector<float>>& refinedDisparities, vector<vector<int>>& hypotheses) {

    refineDisparity(left, right, refinedDisparities, hypotheses);
    refineDisparityWithSlant(left, right, refinedDisparities, hypotheses);
    propagateAndInfer(left, right, refinedDisparities);
    refinePerPixelEstimation(left, right, refinedDisparities);
}

void postProcessing(Mat left, vector<vector<float>>& refinedDisparities) {

    applyMedianFilter(refinedDisparities, 3);
    applyGaussian(refinedDisparities);
    applyWLS(left, refinedDisparities);
}


void SOS(Mat left, Mat right, int maxDisparity, Mat& disparityMap, Mat& depthMap, double focal_length, double baseline) {

    vector<vector<int>> hypotheses;
    vector<vector<float>> refinedDisparities;

    std::ofstream f("sos_timing_log.txt");

    auto start = high_resolution_clock::now();

    auto t0 = high_resolution_clock::now();

    initialization(left, right, hypotheses, maxDisparity);



    auto t1 = high_resolution_clock::now();
    f << "Initialization time: " << duration_cast<milliseconds>(t1 - t0).count() << " s\n";

    t0 = high_resolution_clock::now();

    refinementAndOptimization(left, right, refinedDisparities, hypotheses);
    
    t1 = high_resolution_clock::now();
    f << "Refinement time: " << duration_cast<milliseconds>(t1 - t0).count() << " s\n";

    t0 = high_resolution_clock::now();

    
    postProcessing(left,refinedDisparities);

    t1 = high_resolution_clock::now();
    f << "Post-processing time: " << duration_cast<milliseconds>(t1 - t0).count() << " s\n";

    displayRefinedDisparityMap(refinedDisparities, disparityMap);

    auto end = high_resolution_clock::now();
    f << "Total SOS time: " << duration_cast<milliseconds>(end - start).count() << " s\n";
    f.close();
}
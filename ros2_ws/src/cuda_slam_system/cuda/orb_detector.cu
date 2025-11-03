#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <opencv2/opencv.hpp>

namespace cuda_slam {
namespace cuda {

// CUDA kernel for FAST corner detection
__global__ void fastDetectorKernel(const unsigned char* image,
                                   int width, int height, int stride,
                                   int threshold,
                                   int* corners_x, int* corners_y,
                                   int* corner_count,
                                   int max_corners) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < 3 || y < 3 || x >= width - 3 || y >= height - 3) return;

    int idx = y * stride + x;
    int center = image[idx];

    // FAST-9 circle pattern offsets
    const int circle[16][2] = {
        {0, 3}, {1, 3}, {2, 2}, {3, 1},
        {3, 0}, {3, -1}, {2, -2}, {1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
        {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}
    };

    int brighter = 0;
    int darker = 0;

    for (int i = 0; i < 16; i++) {
        int px = x + circle[i][0];
        int py = y + circle[i][1];
        int pixel_idx = py * stride + px;
        int value = image[pixel_idx];

        if (value > center + threshold) brighter++;
        else if (value < center - threshold) darker++;
    }

    // Need 9 continuous pixels to be a corner
    if (brighter >= 9 || darker >= 9) {
        int count = atomicAdd(corner_count, 1);
        if (count < max_corners) {
            corners_x[count] = x;
            corners_y[count] = y;
        }
    }
}

// CUDA kernel for non-maximum suppression
__global__ void nonMaxSuppressionKernel(const int* corners_x,
                                        const int* corners_y,
                                        const float* responses,
                                        int num_corners,
                                        int* output_x, int* output_y,
                                        int* output_count,
                                        int max_output,
                                        int window_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_corners) return;

    int x = corners_x[idx];
    int y = corners_y[idx];
    float response = responses[idx];

    bool is_max = true;
    for (int i = 0; i < num_corners; i++) {
        if (i == idx) continue;

        int dx = abs(corners_x[i] - x);
        int dy = abs(corners_y[i] - y);

        if (dx <= window_size && dy <= window_size) {
            if (responses[i] > response) {
                is_max = false;
                break;
            }
        }
    }

    if (is_max) {
        int count = atomicAdd(output_count, 1);
        if (count < max_output) {
            output_x[count] = x;
            output_y[count] = y;
        }
    }
}

// CUDA kernel for ORB descriptor computation
__global__ void orbDescriptorKernel(const unsigned char* image,
                                    int width, int height, int stride,
                                    const int* keypoints_x,
                                    const int* keypoints_y,
                                    const float* angles,
                                    int num_keypoints,
                                    unsigned char* descriptors,
                                    int descriptor_size) {
    int kp_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (kp_idx >= num_keypoints) return;

    int x = keypoints_x[kp_idx];
    int y = keypoints_y[kp_idx];
    float angle = angles[kp_idx];

    // Precomputed ORB pattern (simplified)
    const int PATTERN_SIZE = 256;
    __shared__ int pattern_x[PATTERN_SIZE * 2];
    __shared__ int pattern_y[PATTERN_SIZE * 2];

    // Rotate pattern by keypoint angle
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);

    // Compute descriptor
    for (int i = 0; i < descriptor_size; i++) {
        int byte_val = 0;
        for (int bit = 0; bit < 8; bit++) {
            int pair_idx = i * 8 + bit;
            if (pair_idx >= PATTERN_SIZE) break;

            // Sample two points (simplified)
            int dx1 = (pair_idx % 31) - 15;
            int dy1 = (pair_idx / 31) - 15;
            int dx2 = ((pair_idx + 128) % 31) - 15;
            int dy2 = ((pair_idx + 128) / 31) - 15;

            // Rotate by keypoint angle
            int rx1 = (int)(dx1 * cos_angle - dy1 * sin_angle);
            int ry1 = (int)(dx1 * sin_angle + dy1 * cos_angle);
            int rx2 = (int)(dx2 * cos_angle - dy2 * sin_angle);
            int ry2 = (int)(dx2 * sin_angle + dy2 * cos_angle);

            int px1 = x + rx1;
            int py1 = y + ry1;
            int px2 = x + rx2;
            int py2 = y + ry2;

            // Bounds checking
            if (px1 >= 0 && px1 < width && py1 >= 0 && py1 < height &&
                px2 >= 0 && px2 < width && py2 >= 0 && py2 < height) {

                int val1 = image[py1 * stride + px1];
                int val2 = image[py2 * stride + px2];

                if (val1 < val2) {
                    byte_val |= (1 << bit);
                }
            }
        }
        descriptors[kp_idx * descriptor_size + i] = byte_val;
    }
}

// CUDA kernel for Hamming distance computation
__global__ void hammingDistanceKernel(const unsigned char* desc1,
                                      const unsigned char* desc2,
                                      int num_desc1, int num_desc2,
                                      int descriptor_size,
                                      int* distances,
                                      int* matches,
                                      float threshold) {
    int idx1 = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx1 >= num_desc1) return;

    int min_dist = INT_MAX;
    int min_idx = -1;

    for (int idx2 = 0; idx2 < num_desc2; idx2++) {
        int dist = 0;
        for (int i = 0; i < descriptor_size; i++) {
            unsigned char xor_result = desc1[idx1 * descriptor_size + i] ^
                                       desc2[idx2 * descriptor_size + i];
            dist += __popc((unsigned int)xor_result);
        }

        if (dist < min_dist) {
            min_dist = dist;
            min_idx = idx2;
        }
    }

    distances[idx1] = min_dist;
    if (min_dist < threshold) {
        matches[idx1] = min_idx;
    } else {
        matches[idx1] = -1;
    }
}

// CUDA kernel for brute force matching with cross-check
__global__ void bruteForceMatchKernel(const unsigned char* desc1,
                                      const unsigned char* desc2,
                                      int num_desc1, int num_desc2,
                                      int descriptor_size,
                                      int* matches,
                                      float* distances,
                                      float threshold,
                                      bool cross_check) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_desc1) return;

    int best_match = -1;
    float best_dist = FLT_MAX;
    float second_best_dist = FLT_MAX;

    // Find best and second-best matches
    for (int j = 0; j < num_desc2; j++) {
        int dist = 0;
        for (int k = 0; k < descriptor_size; k++) {
            unsigned char xor_val = desc1[idx * descriptor_size + k] ^
                                    desc2[j * descriptor_size + k];
            dist += __popc((unsigned int)xor_val);
        }

        if (dist < best_dist) {
            second_best_dist = best_dist;
            best_dist = dist;
            best_match = j;
        } else if (dist < second_best_dist) {
            second_best_dist = dist;
        }
    }

    // Lowe's ratio test
    if (best_dist < threshold && best_dist < 0.7f * second_best_dist) {
        matches[idx] = best_match;
        distances[idx] = best_dist;
    } else {
        matches[idx] = -1;
        distances[idx] = FLT_MAX;
    }
}

// Host functions
extern "C" {

void launchFastDetector(const unsigned char* d_image,
                       int width, int height, int stride,
                       int threshold,
                       int* d_corners_x, int* d_corners_y,
                       int* d_corner_count,
                       int max_corners) {
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    fastDetectorKernel<<<grid, block>>>(d_image, width, height, stride,
                                        threshold, d_corners_x, d_corners_y,
                                        d_corner_count, max_corners);
    cudaDeviceSynchronize();
}

void launchOrbDescriptor(const unsigned char* d_image,
                        int width, int height, int stride,
                        const int* d_keypoints_x,
                        const int* d_keypoints_y,
                        const float* d_angles,
                        int num_keypoints,
                        unsigned char* d_descriptors,
                        int descriptor_size) {
    int block_size = 256;
    int grid_size = (num_keypoints + block_size - 1) / block_size;

    orbDescriptorKernel<<<grid_size, block_size>>>(
        d_image, width, height, stride,
        d_keypoints_x, d_keypoints_y, d_angles,
        num_keypoints, d_descriptors, descriptor_size);
    cudaDeviceSynchronize();
}

void launchBruteForceMatch(const unsigned char* d_desc1,
                          const unsigned char* d_desc2,
                          int num_desc1, int num_desc2,
                          int descriptor_size,
                          int* d_matches,
                          float* d_distances,
                          float threshold) {
    int block_size = 256;
    int grid_size = (num_desc1 + block_size - 1) / block_size;

    bruteForceMatchKernel<<<grid_size, block_size>>>(
        d_desc1, d_desc2, num_desc1, num_desc2,
        descriptor_size, d_matches, d_distances,
        threshold, true);
    cudaDeviceSynchronize();
}

} // extern "C"

} // namespace cuda
} // namespace cuda_slam

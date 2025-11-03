#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace cuda_slam {
namespace cuda {

// CUDA kernel for descriptor matching with ratio test
__global__ void ratioTestMatchKernel(const unsigned char* desc1,
                                     const unsigned char* desc2,
                                     int num_desc1, int num_desc2,
                                     int descriptor_size,
                                     int* best_matches,
                                     float* best_distances,
                                     float ratio_threshold) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_desc1) return;

    int best_idx = -1;
    int best_dist = INT_MAX;
    int second_best_dist = INT_MAX;

    // Compare with all descriptors in second set
    for (int j = 0; j < num_desc2; j++) {
        int dist = 0;

        // Compute Hamming distance
        for (int k = 0; k < descriptor_size; k++) {
            unsigned char val1 = desc1[idx * descriptor_size + k];
            unsigned char val2 = desc2[j * descriptor_size + k];
            dist += __popc((unsigned int)(val1 ^ val2));
        }

        if (dist < best_dist) {
            second_best_dist = best_dist;
            best_dist = dist;
            best_idx = j;
        } else if (dist < second_best_dist) {
            second_best_dist = dist;
        }
    }

    // Apply ratio test (Lowe's ratio test)
    if (best_dist < ratio_threshold * second_best_dist) {
        best_matches[idx] = best_idx;
        best_distances[idx] = (float)best_dist;
    } else {
        best_matches[idx] = -1;
        best_distances[idx] = FLT_MAX;
    }
}

// CUDA kernel for cross-checking matches
__global__ void crossCheckKernel(const int* matches_1to2,
                                const int* matches_2to1,
                                int num_matches,
                                int* valid_matches) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_matches) return;

    int match_idx = matches_1to2[idx];
    if (match_idx >= 0 && matches_2to1[match_idx] == idx) {
        valid_matches[idx] = 1;
    } else {
        valid_matches[idx] = 0;
    }
}

// CUDA kernel for computing match scores
__global__ void computeMatchScoresKernel(const float* distances,
                                        const int* matches,
                                        int num_matches,
                                        float* scores,
                                        float max_distance) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_matches) return;

    if (matches[idx] >= 0) {
        scores[idx] = 1.0f - (distances[idx] / max_distance);
    } else {
        scores[idx] = 0.0f;
    }
}

// Host wrapper functions
extern "C" {

void launchRatioTestMatch(const unsigned char* d_desc1,
                         const unsigned char* d_desc2,
                         int num_desc1, int num_desc2,
                         int descriptor_size,
                         int* d_matches,
                         float* d_distances,
                         float ratio_threshold) {
    int block_size = 256;
    int grid_size = (num_desc1 + block_size - 1) / block_size;

    ratioTestMatchKernel<<<grid_size, block_size>>>(
        d_desc1, d_desc2, num_desc1, num_desc2,
        descriptor_size, d_matches, d_distances, ratio_threshold);

    cudaDeviceSynchronize();
}

void launchCrossCheck(const int* d_matches_1to2,
                     const int* d_matches_2to1,
                     int num_matches,
                     int* d_valid_matches) {
    int block_size = 256;
    int grid_size = (num_matches + block_size - 1) / block_size;

    crossCheckKernel<<<grid_size, block_size>>>(
        d_matches_1to2, d_matches_2to1,
        num_matches, d_valid_matches);

    cudaDeviceSynchronize();
}

} // extern "C"

} // namespace cuda
} // namespace cuda_slam

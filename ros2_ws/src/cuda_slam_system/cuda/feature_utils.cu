#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace cuda_slam {
namespace cuda {

// CUDA kernel for computing image gradients
__global__ void computeGradientsKernel(const unsigned char* image,
                                       int width, int height, int stride,
                                       float* grad_x, float* grad_y,
                                       float* magnitude) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < 1 || y < 1 || x >= width - 1 || y >= height - 1) return;

    int idx = y * stride + x;

    // Sobel operator
    float gx = -1.0f * image[(y-1)*stride + (x-1)] +
                1.0f * image[(y-1)*stride + (x+1)] +
               -2.0f * image[y*stride + (x-1)] +
                2.0f * image[y*stride + (x+1)] +
               -1.0f * image[(y+1)*stride + (x-1)] +
                1.0f * image[(y+1)*stride + (x+1)];

    float gy = -1.0f * image[(y-1)*stride + (x-1)] +
               -2.0f * image[(y-1)*stride + x] +
               -1.0f * image[(y-1)*stride + (x+1)] +
                1.0f * image[(y+1)*stride + (x-1)] +
                2.0f * image[(y+1)*stride + x] +
                1.0f * image[(y+1)*stride + (x+1)];

    grad_x[idx] = gx;
    grad_y[idx] = gy;
    magnitude[idx] = sqrtf(gx * gx + gy * gy);
}

// CUDA kernel for orientation computation
__global__ void computeOrientationKernel(const unsigned char* image,
                                        int width, int height, int stride,
                                        const int* keypoints_x,
                                        const int* keypoints_y,
                                        int num_keypoints,
                                        float* orientations,
                                        int patch_radius) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_keypoints) return;

    int x = keypoints_x[idx];
    int y = keypoints_y[idx];

    float m10 = 0.0f;
    float m01 = 0.0f;

    // Compute moments in patch
    for (int dy = -patch_radius; dy <= patch_radius; dy++) {
        for (int dx = -patch_radius; dx <= patch_radius; dx++) {
            int px = x + dx;
            int py = y + dy;

            if (px >= 0 && px < width && py >= 0 && py < height) {
                float intensity = (float)image[py * stride + px];
                m10 += dx * intensity;
                m01 += dy * intensity;
            }
        }
    }

    orientations[idx] = atan2f(m01, m10);
}

// CUDA kernel for Gaussian blur (separable)
__global__ void gaussianBlurHorizontalKernel(const unsigned char* input,
                                            unsigned char* output,
                                            int width, int height, int stride,
                                            const float* kernel,
                                            int kernel_size) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    int half_kernel = kernel_size / 2;
    float sum = 0.0f;

    for (int k = -half_kernel; k <= half_kernel; k++) {
        int px = x + k;
        if (px >= 0 && px < width) {
            sum += input[y * stride + px] * kernel[k + half_kernel];
        }
    }

    output[y * stride + x] = (unsigned char)fminf(fmaxf(sum, 0.0f), 255.0f);
}

__global__ void gaussianBlurVerticalKernel(const unsigned char* input,
                                          unsigned char* output,
                                          int width, int height, int stride,
                                          const float* kernel,
                                          int kernel_size) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    int half_kernel = kernel_size / 2;
    float sum = 0.0f;

    for (int k = -half_kernel; k <= half_kernel; k++) {
        int py = y + k;
        if (py >= 0 && py < height) {
            sum += input[py * stride + x] * kernel[k + half_kernel];
        }
    }

    output[y * stride + x] = (unsigned char)fminf(fmaxf(sum, 0.0f), 255.0f);
}

// CUDA kernel for image downsampling
__global__ void downsampleKernel(const unsigned char* input,
                                unsigned char* output,
                                int in_width, int in_height, int in_stride,
                                int out_width, int out_height, int out_stride) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= out_width || y >= out_height) return;

    int src_x = x * 2;
    int src_y = y * 2;

    // Simple 2x2 averaging
    float sum = 0.0f;
    int count = 0;

    for (int dy = 0; dy < 2; dy++) {
        for (int dx = 0; dx < 2; dx++) {
            int px = src_x + dx;
            int py = src_y + dy;
            if (px < in_width && py < in_height) {
                sum += input[py * in_stride + px];
                count++;
            }
        }
    }

    output[y * out_stride + x] = (unsigned char)(sum / count);
}

// Host wrapper functions
extern "C" {

void launchComputeGradients(const unsigned char* d_image,
                           int width, int height, int stride,
                           float* d_grad_x, float* d_grad_y,
                           float* d_magnitude) {
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    computeGradientsKernel<<<grid, block>>>(
        d_image, width, height, stride,
        d_grad_x, d_grad_y, d_magnitude);

    cudaDeviceSynchronize();
}

void launchComputeOrientation(const unsigned char* d_image,
                             int width, int height, int stride,
                             const int* d_keypoints_x,
                             const int* d_keypoints_y,
                             int num_keypoints,
                             float* d_orientations,
                             int patch_radius) {
    int block_size = 256;
    int grid_size = (num_keypoints + block_size - 1) / block_size;

    computeOrientationKernel<<<grid_size, block_size>>>(
        d_image, width, height, stride,
        d_keypoints_x, d_keypoints_y, num_keypoints,
        d_orientations, patch_radius);

    cudaDeviceSynchronize();
}

void launchDownsample(const unsigned char* d_input,
                     unsigned char* d_output,
                     int in_width, int in_height, int in_stride,
                     int out_width, int out_height, int out_stride) {
    dim3 block(16, 16);
    dim3 grid((out_width + block.x - 1) / block.x,
              (out_height + block.y - 1) / block.y);

    downsampleKernel<<<grid, block>>>(
        d_input, d_output,
        in_width, in_height, in_stride,
        out_width, out_height, out_stride);

    cudaDeviceSynchronize();
}

} // extern "C"

} // namespace cuda
} // namespace cuda_slam

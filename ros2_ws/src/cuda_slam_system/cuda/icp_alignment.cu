#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cublas_v2.h>

namespace cuda_slam {
namespace cuda {

struct Point3D {
    float x, y, z;
};

struct Transform {
    float rotation[9];    // 3x3 rotation matrix (row-major)
    float translation[3]; // 3x1 translation vector
};

// CUDA kernel to transform points
__global__ void transformPointsKernel(const Point3D* input,
                                     Point3D* output,
                                     int num_points,
                                     const Transform transform) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    Point3D p = input[idx];

    // Apply rotation
    float x = transform.rotation[0] * p.x +
              transform.rotation[1] * p.y +
              transform.rotation[2] * p.z;

    float y = transform.rotation[3] * p.x +
              transform.rotation[4] * p.y +
              transform.rotation[5] * p.z;

    float z = transform.rotation[6] * p.x +
              transform.rotation[7] * p.y +
              transform.rotation[8] * p.z;

    // Apply translation
    output[idx].x = x + transform.translation[0];
    output[idx].y = y + transform.translation[1];
    output[idx].z = z + transform.translation[2];
}

// CUDA kernel to find nearest neighbors (brute force)
__global__ void findNearestNeighborsKernel(const Point3D* source,
                                          const Point3D* target,
                                          int num_source,
                                          int num_target,
                                          int* correspondences,
                                          float* distances,
                                          float max_distance) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_source) return;

    Point3D p = source[idx];
    float min_dist = FLT_MAX;
    int min_idx = -1;

    // Search for nearest neighbor in target cloud
    for (int j = 0; j < num_target; j++) {
        Point3D q = target[j];

        float dx = p.x - q.x;
        float dy = p.y - q.y;
        float dz = p.z - q.z;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);

        if (dist < min_dist && dist < max_distance) {
            min_dist = dist;
            min_idx = j;
        }
    }

    correspondences[idx] = min_idx;
    distances[idx] = min_dist;
}

// CUDA kernel to compute point-to-point error
__global__ void computePointToPointErrorKernel(const Point3D* source,
                                              const Point3D* target,
                                              const int* correspondences,
                                              int num_points,
                                              float* errors) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int target_idx = correspondences[idx];
    if (target_idx < 0) {
        errors[idx] = 0.0f;
        return;
    }

    Point3D p = source[idx];
    Point3D q = target[target_idx];

    float dx = p.x - q.x;
    float dy = p.y - q.y;
    float dz = p.z - q.z;

    errors[idx] = dx * dx + dy * dy + dz * dz;
}

// CUDA kernel to compute centroids
__global__ void computeCentroidKernel(const Point3D* points,
                                     int num_points,
                                     float* centroid_x,
                                     float* centroid_y,
                                     float* centroid_z) {
    __shared__ float shared_x[256];
    __shared__ float shared_y[256];
    __shared__ float shared_z[256];

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Load data into shared memory
    if (idx < num_points) {
        shared_x[tid] = points[idx].x;
        shared_y[tid] = points[idx].y;
        shared_z[tid] = points[idx].z;
    } else {
        shared_x[tid] = 0.0f;
        shared_y[tid] = 0.0f;
        shared_z[tid] = 0.0f;
    }

    __syncthreads();

    // Reduction to compute sum
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s && idx + s < num_points) {
            shared_x[tid] += shared_x[tid + s];
            shared_y[tid] += shared_y[tid + s];
            shared_z[tid] += shared_z[tid + s];
        }
        __syncthreads();
    }

    // Write result for this block
    if (tid == 0) {
        atomicAdd(centroid_x, shared_x[0]);
        atomicAdd(centroid_y, shared_y[0]);
        atomicAdd(centroid_z, shared_z[0]);
    }
}

// CUDA kernel to center point clouds
__global__ void centerPointsKernel(Point3D* points,
                                  int num_points,
                                  float cx, float cy, float cz) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    points[idx].x -= cx;
    points[idx].y -= cy;
    points[idx].z -= cz;
}

// CUDA kernel to build covariance matrix H = sum(p_i * q_i^T)
__global__ void buildCovarianceKernel(const Point3D* source,
                                     const Point3D* target,
                                     const int* correspondences,
                                     int num_points,
                                     float* H) {
    __shared__ float shared_H[9][256];

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Initialize shared memory
    for (int i = 0; i < 9; i++) {
        shared_H[i][tid] = 0.0f;
    }

    // Compute contribution
    if (idx < num_points && correspondences[idx] >= 0) {
        Point3D p = source[idx];
        Point3D q = target[correspondences[idx]];

        shared_H[0][tid] = p.x * q.x;
        shared_H[1][tid] = p.x * q.y;
        shared_H[2][tid] = p.x * q.z;
        shared_H[3][tid] = p.y * q.x;
        shared_H[4][tid] = p.y * q.y;
        shared_H[5][tid] = p.y * q.z;
        shared_H[6][tid] = p.z * q.x;
        shared_H[7][tid] = p.z * q.y;
        shared_H[8][tid] = p.z * q.z;
    }

    __syncthreads();

    // Reduction
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            for (int i = 0; i < 9; i++) {
                shared_H[i][tid] += shared_H[i][tid + s];
            }
        }
        __syncthreads();
    }

    // Write result
    if (tid == 0) {
        for (int i = 0; i < 9; i++) {
            atomicAdd(&H[i], shared_H[i][0]);
        }
    }
}

// Host wrapper functions
extern "C" {

void launchTransformPoints(const Point3D* d_input,
                          Point3D* d_output,
                          int num_points,
                          const float* rotation,
                          const float* translation) {
    Transform transform;
    for (int i = 0; i < 9; i++) {
        transform.rotation[i] = rotation[i];
    }
    for (int i = 0; i < 3; i++) {
        transform.translation[i] = translation[i];
    }

    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;

    transformPointsKernel<<<grid_size, block_size>>>(
        d_input, d_output, num_points, transform);

    cudaDeviceSynchronize();
}

void launchFindNearestNeighbors(const Point3D* d_source,
                               const Point3D* d_target,
                               int num_source,
                               int num_target,
                               int* d_correspondences,
                               float* d_distances,
                               float max_distance) {
    int block_size = 256;
    int grid_size = (num_source + block_size - 1) / block_size;

    findNearestNeighborsKernel<<<grid_size, block_size>>>(
        d_source, d_target, num_source, num_target,
        d_correspondences, d_distances, max_distance);

    cudaDeviceSynchronize();
}

void launchComputePointToPointError(const Point3D* d_source,
                                   const Point3D* d_target,
                                   const int* d_correspondences,
                                   int num_points,
                                   float* d_errors) {
    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;

    computePointToPointErrorKernel<<<grid_size, block_size>>>(
        d_source, d_target, d_correspondences, num_points, d_errors);

    cudaDeviceSynchronize();
}

void launchComputeCentroid(const Point3D* d_points,
                          int num_points,
                          float* h_centroid) {
    float *d_cx, *d_cy, *d_cz;
    cudaMalloc(&d_cx, sizeof(float));
    cudaMalloc(&d_cy, sizeof(float));
    cudaMalloc(&d_cz, sizeof(float));

    cudaMemset(d_cx, 0, sizeof(float));
    cudaMemset(d_cy, 0, sizeof(float));
    cudaMemset(d_cz, 0, sizeof(float));

    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;

    computeCentroidKernel<<<grid_size, block_size>>>(
        d_points, num_points, d_cx, d_cy, d_cz);

    cudaDeviceSynchronize();

    cudaMemcpy(&h_centroid[0], d_cx, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_centroid[1], d_cy, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_centroid[2], d_cz, sizeof(float), cudaMemcpyDeviceToHost);

    h_centroid[0] /= num_points;
    h_centroid[1] /= num_points;
    h_centroid[2] /= num_points;

    cudaFree(d_cx);
    cudaFree(d_cy);
    cudaFree(d_cz);
}

} // extern "C"

} // namespace cuda
} // namespace cuda_slam

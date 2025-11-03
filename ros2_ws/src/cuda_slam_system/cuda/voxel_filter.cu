#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/unique.h>

namespace cuda_slam {
namespace cuda {

struct Point3D {
    float x, y, z;
};

struct VoxelKey {
    int x, y, z;

    __device__ bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    __device__ bool operator<(const VoxelKey& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

// CUDA kernel to compute voxel keys for each point
__global__ void computeVoxelKeysKernel(const Point3D* points,
                                       int num_points,
                                       VoxelKey* voxel_keys,
                                       float voxel_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    Point3D p = points[idx];
    voxel_keys[idx].x = (int)floorf(p.x / voxel_size);
    voxel_keys[idx].y = (int)floorf(p.y / voxel_size);
    voxel_keys[idx].z = (int)floorf(p.z / voxel_size);
}

// CUDA kernel to compute voxel centroids
__global__ void computeVoxelCentroidsKernel(const Point3D* points,
                                           const VoxelKey* sorted_keys,
                                           const int* voxel_starts,
                                           const int* voxel_ends,
                                           int num_voxels,
                                           Point3D* centroids) {
    int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (voxel_idx >= num_voxels) return;

    int start = voxel_starts[voxel_idx];
    int end = voxel_ends[voxel_idx];

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int count = 0;

    for (int i = start; i < end; i++) {
        sum_x += points[i].x;
        sum_y += points[i].y;
        sum_z += points[i].z;
        count++;
    }

    if (count > 0) {
        centroids[voxel_idx].x = sum_x / count;
        centroids[voxel_idx].y = sum_y / count;
        centroids[voxel_idx].z = sum_z / count;
    }
}

// CUDA kernel for spatial hashing-based voxel filtering
__global__ void voxelHashFilterKernel(const Point3D* points,
                                     int num_points,
                                     Point3D* output,
                                     int* output_count,
                                     float voxel_size,
                                     int hash_table_size,
                                     int* hash_table) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    Point3D p = points[idx];

    // Compute voxel coordinates
    int vx = (int)floorf(p.x / voxel_size);
    int vy = (int)floorf(p.y / voxel_size);
    int vz = (int)floorf(p.z / voxel_size);

    // Compute hash
    unsigned int hash = ((vx * 73856093) ^ (vy * 19349663) ^ (vz * 83492791));
    hash = hash % hash_table_size;

    // Try to insert into hash table
    int old = atomicCAS(&hash_table[hash], -1, idx);

    // If we successfully inserted (first point in this voxel), add to output
    if (old == -1) {
        int out_idx = atomicAdd(output_count, 1);
        output[out_idx] = p;
    }
}

// CUDA kernel for downsampling using averaging
__global__ void voxelAverageKernel(const Point3D* points,
                                  const int* point_indices,
                                  const int* voxel_starts,
                                  const int* voxel_counts,
                                  int num_voxels,
                                  Point3D* output) {
    int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (voxel_idx >= num_voxels) return;

    int start = voxel_starts[voxel_idx];
    int count = voxel_counts[voxel_idx];

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < count; i++) {
        int point_idx = point_indices[start + i];
        sum_x += points[point_idx].x;
        sum_y += points[point_idx].y;
        sum_z += points[point_idx].z;
    }

    output[voxel_idx].x = sum_x / count;
    output[voxel_idx].y = sum_y / count;
    output[voxel_idx].z = sum_z / count;
}

// Host functions
extern "C" {

void launchVoxelFilter(const float* h_points,
                      int num_points,
                      float voxel_size,
                      float** h_output,
                      int* num_output) {
    // Allocate device memory
    Point3D* d_points;
    Point3D* d_output;
    int* d_output_count;
    int* d_hash_table;

    cudaMalloc(&d_points, num_points * sizeof(Point3D));
    cudaMalloc(&d_output, num_points * sizeof(Point3D));
    cudaMalloc(&d_output_count, sizeof(int));

    // Hash table size (prime number)
    int hash_table_size = num_points * 2 + 1;
    cudaMalloc(&d_hash_table, hash_table_size * sizeof(int));

    // Initialize hash table to -1
    cudaMemset(d_hash_table, -1, hash_table_size * sizeof(int));
    cudaMemset(d_output_count, 0, sizeof(int));

    // Copy input points
    cudaMemcpy(d_points, h_points, num_points * sizeof(Point3D),
               cudaMemcpyHostToDevice);

    // Launch kernel
    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;

    voxelHashFilterKernel<<<grid_size, block_size>>>(
        d_points, num_points, d_output, d_output_count,
        voxel_size, hash_table_size, d_hash_table);

    cudaDeviceSynchronize();

    // Get output count
    cudaMemcpy(num_output, d_output_count, sizeof(int),
               cudaMemcpyDeviceToHost);

    // Allocate and copy output
    *h_output = (float*)malloc(*num_output * sizeof(Point3D));
    cudaMemcpy(*h_output, d_output, *num_output * sizeof(Point3D),
               cudaMemcpyDeviceToHost);

    // Cleanup
    cudaFree(d_points);
    cudaFree(d_output);
    cudaFree(d_output_count);
    cudaFree(d_hash_table);
}

void launchVoxelFilterDevice(const Point3D* d_points,
                            int num_points,
                            float voxel_size,
                            Point3D* d_output,
                            int* d_output_count,
                            int hash_table_size,
                            int* d_hash_table) {
    int block_size = 256;
    int grid_size = (num_points + block_size - 1) / block_size;

    voxelHashFilterKernel<<<grid_size, block_size>>>(
        d_points, num_points, d_output, d_output_count,
        voxel_size, hash_table_size, d_hash_table);

    cudaDeviceSynchronize();
}

} // extern "C"

} // namespace cuda
} // namespace cuda_slam

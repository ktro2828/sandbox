#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>

constexpr int N = 10;

__global__ void matrixAdd(float** out, float** a, float** b, int n)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  int j = blockIdx.y * blockDim.y + threadIdx.y;
  if (i < N && j < N)
  {
    out[i][j] = a[i][j] + b[i][j];
  }
}

int main()
{
  float **a, **b, **out;
  float **d_a, **d_b, **d_out;

  // Allocate memory
  a = (float**)malloc(sizeof(float*) * N);
  b = (float**)malloc(sizeof(float*) * N);
  out = (float**)malloc(sizeof(float*) * N);

  // Initialize array
  for (int i = 0; i < N; i++)
    {
      // Allocate memory for each row of the matrix.
      a[i] = (float*)malloc(sizeof(float) * N);
      b[i] = (float*)malloc(sizeof(float) * N);
      out[i] = (float*)malloc(sizeof(float) * N);
      for (int j = 0; j < N; j++)
      {
        a[i][j] = 1.0f;
        b[i][j] = 2.0f;
      }
    }

  // Allocate device memory
  cudaMalloc((void**)&d_a, sizeof(float*) * N);
  cudaMalloc((void**)&d_b, sizeof(float*) * N);
  cudaMalloc((void**)&d_out, sizeof(float*) * N);
  for (int i = 0; i < N; i++)
  {
    // Allocate memory  on device
    cudaMalloc((void**)&(d_a[i]), sizeof(float) * N);
    cudaMalloc((void**)&(d_b[i]), sizeof(float) * N);
    cudaMalloc((void**)&(d_out[i]), sizeof(float) * N);
  }

  // Transfer data from host to device
  cudaMemcpy(d_a, a, sizeof(float*) * N, cudaMemcpyHostToDevice);
  cudaMemcpy(d_b, b, sizeof(float*) * N, cudaMemcpyHostToDevice);
  for (int i = 0; i < N; i++)
  {
    cudaMemcpy(d_a[i], a[i], sizeof(float) * N, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b[i], b[i], sizeof(float) * N, cudaMemcpyHostToDevice);
  }

  // Executing kernel
  dim3 threadsPerBlock(16, 16);
  dim3 numBlocks(N / threadsPerBlock.x, N / threadsPerBlock.y);
  matrixAdd<<<numBlocks, threadsPerBlock>>>(d_out, d_a, d_b, N);

  // Transfer data back to host memory
  cudaMemcpy(out, d_out, sizeof(float*) * N, cudaMemcpyDeviceToHost);
  for (int i = 0; i < N; i++)
  {
    cudaMemcpy(out[i], d_out[i], sizeof(float) * N, cudaMemcpyDeviceToHost);
  }

  printf("%f\n", out[0][0]);

  // // Deallocate device memory
  for (int i = 0; i < N; i++)
  {
    cudaFree(d_a[i]);
    cudaFree(d_b[i]);
    cudaFree(d_out[i]);
    free(a[i]);
    free(b[i]);
    free(out[i]);
  }

  cudaFree(d_a);
  cudaFree(d_b);
  cudaFree(d_out);

  // Deallocate host memory
  free(a);
  free(b);
  free(out);
}

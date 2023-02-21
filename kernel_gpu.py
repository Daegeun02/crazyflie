import cupy as cp
import numpy as np
import scipy.sparse.linalg as sla
import numpy.linalg as lg
import matplotlib.pyplot as plt

def ker_calnext():
    SM_calnext = cp.RawKernel(r"""
    #define _X (blockDim.x * blockIdx.x + threadIdx.x)
    #define _Y (blockDim.y * blockIdx.y + threadIdx.y)    
    #define _Coor (_X + _Y*gridDim.x*blockDim.x)
    #include <cooperative_groups.h>
    namespace cg = cooperative_groups;
    
    __device__ void matmul_(float* x, float*y, int first, int second, int third, float* matmul_result, int tr_tfs, int act_upper){

        int idx = _Coor;
        if(idx >= act_upper){
            return;
        }

        idx = _Y;
        float x_vec[1000];
        float y_vec[1000];
        //float* x_vec = new float[second];
        //float* y_vec = new float[second];
        // transpose is false
        if(tr_tfs == 0){
            for(int i=0; i< second; i++){
                x_vec[i] = x[idx*second+i];
            }
        }

        else{
            for(int i=0; i< second; i++){
                x_vec[i] = x[idx+first*i];
            }
        }
        
        idx = _X;

        for(int i=0; i < second; i++){
            y_vec[i] = y[idx+i*third];
        }

        float temp=0;

        for (int i = 0; i < second; i++){
            temp += x_vec[i]*y_vec[i];
        }

        idx = _Coor;
        matmul_result[idx] = temp;
        delete[] x_vec;
        delete[] y_vec;       
    }

    __device__ void cbdsnproj(float* x, float ub){
        int idx = _Coor ;
        float temp = 0;

        for (int i = 0; i < 3; i++){
            temp += x[3*idx+i]*x[3*idx+i];
        }

        temp = sqrt(temp);

        if(temp > ub){
            for(int i = 0; i < 3; i++){
                x[3*idx+i] = x[3*idx+i]/(temp)*ub;
            }
        }
    }

    extern "C"
    __global__ void calnext(float* coeff_x, float* A, float*c, float* x,
    float* u1, float* u2, float* z, float rho1, float rho2, int n, float ub ,\
    int len_c, int len_z, int k, float eps_pri, float eps_dual, float*temp, \
    float* temp_sum, float* z_buffer, float* residuals, int* optimality, int act_upper){
        cg::grid_group grid = cg::this_grid();
        int idx = _Coor;

        for (int j = 0; j < k ; j++){
            
            // shape of A: (6, 3*n)
            if(idx < act_upper){
                z_buffer[idx] = z[idx];
            }
            
            if (idx < len_c){
                temp_sum[idx] = -c[idx] + u1[idx];
            }
            sync(grid);

            matmul_(A, temp_sum, len_z, 6, 1, temp, 1, act_upper);
            if(idx < act_upper){
                temp_sum[idx] = -rho1*temp[idx] - rho2*(-z[idx]+u2[idx]);
            }
            sync(grid);

            matmul_(coeff_x, temp_sum, len_z, len_z, 1, x, 0, act_upper);
            if(idx < act_upper){
                z[idx] = x[idx] + u2[idx];
            }
            sync(grid);

            if(idx < n){
                cbdsnproj(z, ub);
            }
            sync(grid);
            
            if(idx < len_c){
                matmul_(A, x, 6, 3*n, 1, temp, 0, act_upper);          
                u1[idx] = u1[idx] + temp[idx] - c[idx];                  
            }
        
            if(idx < len_z){
                u2[idx] = u2[idx] + x[idx] - z[idx];
            }
            sync(grid);
            
            /*
     
            //extern __shared__ float res.idual[]; 
            __shared__ float residual[2000]; 
            __shared__ float primal_res1[6];

            int idx_b = threadIdx.y;
            // blockidx.y == 0 -> calculate dual residuals, blockIdx.y == 1 -> calculate primal residuals2
            if(blockIdx.y == 0){
                for(int i = 0; i < 3; i++){
                    residual[3*idx_b + i] = rho2*(z_buffer[3*idx_b + i] - z[3*idx_b + i])*rho2*(z_buffer[3*idx_b + i] - z[3*idx_b + i]);
                }
                if(idx < len_c){
                    primal_res1[idx] = (temp[idx] - c[idx])*(temp[idx] - c[idx]);
                }
            }
            else if(blockIdx.y == 1){
                for(int i = 0; i < 3; i++){
                    residual[3*idx_b + i] = (x[3*idx_b + i]- z[3*idx_b + i])*(x[3*idx_b + i]- z[3*idx_b + i]);
                    temp[idx] = idx;
                }
            }
    
            sync(grid);

            if (idx == 0){
                for(int i = 0; i < len_z; i++){
                    residuals[0 + j] += residual[i];
                    if(i < len_c){
                        residuals[1*k + j] += primal_res1[i];
                    }                                         
                }             
                 
            }
            else if(idx == n){
                for(int i = 0; i < len_z; i++){
                    residuals[2*k + j] += residual[i];  //primal residuals2                                 
                }             
            }

            sync(grid);
            if(idx == 0){
                for(int i = 0; i < 3; i++){
                    residuals[i*k + j] = sqrt(residuals[i*k + j]);
                }         
            }
           
            sync(grid);

            if ((residuals[0 + j])<= eps_dual && (residuals[1*k + j]<= eps_pri) && (residuals[2*k + j] <= eps_pri)){
                if(idx == 0){
                    optimality[0] = 1;
                    optimality[1] = j;
                }
                return;
            } 
            else{
                if(idx == 0){
                    optimality[0] = 0;
                }
            }*/
       
        }
    
    }
    """, 'calnext', enable_cooperative_groups=True)
    return SM_calnext

def ker_make_matrix():
    SM_make_matrix = cp.RawKernel(r"""
    #define _X (blockDim.x * blockIdx.x + threadIdx.x)
    #define _Y (blockDim.y * blockIdx.y + threadIdx.y)  
    #define _Coor (_X + _Y*gridDim.x*blockDim.x)

    extern "C" __global__ void make_matrix(float dt, float* result_G, int N, float* result_c, float* x_0, float* x_des)
    {
        
        //__shared__ float temp_mat[1800];  //shape: 6XN
        extern __shared__ float temp_mat[];
        __shared__ float first[6] ;

        float B[18] = {0,};
        for(int i = 0; i < 18; i++){
            B[i] = 0;
        }

        float b[6] = {0, 0, 0, 0, 0, 0};
        float g = -9.81;

        //b[2] = 0.5*g*pow(dt,2);
        b[2] = 0.5*g*dt*dt;
        b[5] = g*dt;

        for(int i = 0; i < 3; i++){
            //B[3*i + i] = 0.5* pow(dt,2);
            B[3*i + i] = 0.5* dt*dt;
            B[3*(i+3) + i] = dt;
        }
        
        float A_pow[36] ;  // shape of A: (6,6)
        int idx = threadIdx.x;
        int idx2 = blockIdx.y;
        int n = N - (idx + 1);
        for(int i = 0; i < 6; i++){
            for(int j = 0; j < 6; j++){
                A_pow[6*i + j] = 0;
            }
            A_pow[7*i] = 1;
            A_pow[7*i + 3] = n * dt;
        }
        

        float temp = 0;
        
        if(idx2== 0){ // make matrix G
            
            for(int i = 0; i < 6; i++){
                for(int j = 0; j < 3; j++){
                    for(int k = 0; k < 6; k++){
                        temp += A_pow[i*6 + k] * B[j + k*3];
                    }

                    idx2 = 3*idx + i*N*3 + j;
                    result_G[idx2] = temp;
                    temp = 0;

                }
            }
        }
    

        else{
            for(int i = 0; i < 6; i++){
                for(int j = 0 ; j < 6; j++){
                    temp += A_pow[i*6 + j]*b[j];
                }
                temp_mat[i*N + idx] = temp;
                temp = 0;
                
            }
            __syncthreads();

            if(idx < 6){
                first[idx] = 0.0;
                int n = N ;

                for(int i = 0; i < 6; i++){
                    for(int j = 0; j < 6; j++){
                        A_pow[6*i + j] = 0;
                    }
                    A_pow[7*i] = 1;
                    A_pow[7*i + 3] = n * dt;
                }
                for(int i = 0; i < 6; i++){
                    first[idx] += A_pow[idx*6 + i]*x_0[i];
                }
                for(int i = 0; i < N; i++){
                    result_c[idx] += temp_mat[i + idx*N]; 
                }

                result_c[idx] = x_des[idx] - result_c[idx] - first[idx];

            }
            
        }

    }""", 'make_matrix')
    return SM_make_matrix

# # asarray --> array
# np.random.seed(1)
# class ADMM_optimizer_gpu():
#     def __init__(self, x_0,  x_des, T,  bd,  n,  rho1, rho2, eps_dual, eps_pri, k):
#         self.func_make_matrix = ker_make_matrix()
#         self.func_calnext  = ker_calnext()
#         self.n = np.int32(n)
#         self.T = np.float32(T)
#         self.dt = (self.T/self.n).astype(np.float32)  # fixed 
#         self.bd = np.float32(bd)
#         self.x_0 = np.float32(x_0)
#         self.x_des = np.float32(x_des)
#         self.x_0_gpu = cp.array(self.x_0)
#         self.x_des_gpu = cp.array(self.x_des)
#         self.res_gpu = cp.array(np.float32(np.zeros((3, k))))
#         self.rho1 = np.float32(rho1)
#         self.rho2 = np.float32(rho2)
#         self.eps_dual = np.float32(eps_dual)
#         self.eps_pri = np.float32(eps_pri)
#         self.k = np.int32(k)
#         self.c_row = np.int32(6)
#         self.z_row = np.int32(3*self.n)
#         self.residuals_gpu = cp.array(np.float32(np.zeros((3, self.k))))
#         self.optimality_gpu = cp.array(np.int32(np.zeros((2,1))))
#         self.gpu_zip = []
        
#     def pretreatment(self, n):
#         G = np.float32(np.zeros((6,3*n)))
#         c = np.float32(np.zeros((6,1)))
#         self.A_gpu = cp.array(G)
#         self.c_gpu = cp.array(c)
#         self.func_make_matrix((1, 2, 1), (int(n),1,1), (self.dt, self.A_gpu, n,\
#                               self.c_gpu, self.x_0_gpu, self.x_des_gpu), shared_mem=6*n*4)
#         self.tA_gpu = cp.transpose(self.A_gpu)
#         G = cp.asnumpy(self.A_gpu)
#         c = cp.asnumpy(self.c_gpu)
#         tG = G.T
#         # calculate start points and constant matrix
#         u_least = sla.lsqr(G, c)[0]
#         u_least = np.float32(np.reshape(u_least, (3*n, 1)))
#         u1 = sla.lsqr(tG, -2*u_least)[0]/self.rho1 
#         u1 =  np.float32(np.reshape(u1, (len(u1),1)))
#         u2 = np.float32((-2*u_least-self.rho1*tG@u1)/self.rho2)
#         u2 = np.reshape(u2, (len(u2),1))
#         coeff_x = np.float32(lg.inv(self.rho1*tG@G+(2+self.rho2)*np.eye(3*n)))
#         # allocate gpu memory
#         self.x_gpu = cp.array(u_least)
#         self.z_gpu = cp.array(np.float32(np.sqrt(self.bd**2/3)*np.random.randn(n*3,1)))
#         self.u1_gpu = cp.array(u1)
#         self.u2_gpu = cp.array(u2)
#         self.coeff_x_gpu = cp.array(coeff_x)
#         self.temp_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
#         self.temp_sum_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
#         self.z_buffer_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
#         return


        
#     def ADMM_opt_gpu(self, dim_block, dim_grid):
        
#         self.pretreatment(self.n)
# #         # t1 = time()
# #         self.func_calnext((1,3,1), (1, int(self.n), 1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
# #                           self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
# #                           self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
# #                           self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
# #                           self.optimality_gpu))#, shared_mem=3*self.n*4)
#         self.func_calnext((1,dim_grid,1), (1, dim_block,1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
#                           self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
#                           self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
#                           self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
#                           self.optimality_gpu))#, shared_mem=3*self.n*4)
#         # t2 = time()
#         # print("time to optimize ", t2-t1)
#         # t1 =time()
#         # self.x = cp.asnumpy(self.x_gpu)
#         # t2 = time()
#         # print("time to get numpy from gpuarray", t2-t1)
        
#         return
    
#     def free_cp(self):
#         del self.x_0_gpu
#         del self.x_des_gpu
#         del self.A_gpu
#         del self.c_gpu
#         del self.residuals_gpu
#         del self.res_gpu
#         del self.optimality_gpu
#         del self.temp_gpu
#         del self.u1_gpu
#         del self.u2_gpu
#         del self.temp_sum_gpu

# pinned_mempool = cp.get_default_pinned_memory_pool()
# mempool = cp.get_default_memory_pool()

# print(mempool.used_bytes()) 
# print(mempool.total_bytes()) 

# T = 10
# n = 100  #0~1000
# dt = T/n
# x_des = np.array([0,0,0,0,0,0])
# x_0 =np.array([100,0,-1500,-10,0,80]) 
# ts = np.linspace(0,T,n+1)
# u_lb = 0
# u_ub = 5.8
# p_upper = 600
# p_lower = 0
# num = 0

# optimizer_gpu = ADMM_optimizer_gpu(x_0, x_des, T,  u_ub, n, 0.01, 0.01, 0.01, 0.01, 5000 )
# optimizer_gpu.ADMM_opt_gpu(10,30)

# print(mempool.used_bytes()) # 0
# print(mempool.total_bytes()) # 512

# del optimizer_gpu

# pinned_mempool .free_all_blocks()
# mempool.free_all_blocks()

# print(mempool.used_bytes()) # 0
# print(mempool.total_bytes()) # 512
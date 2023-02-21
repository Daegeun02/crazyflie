import numpy as np
import scipy.sparse.linalg as sla
import cupy as cp
from kernel_gpu import ker_make_matrix, ker_calnext
import numpy.linalg as lg

# asarray --> array
np.random.seed(1)
class ADMM_optimizer_gpu():
    def __init__(self, x_0,  x_des, T,  bd,  n,  rho1, rho2, eps_dual, eps_pri, k, fixed_n):
        self.func_make_matrix = ker_make_matrix()
        self.func_calnext  = ker_calnext()
        self.n = np.int32(n)
        self.T = np.float32(T)
        self.dt = (self.T/self.n).astype(np.float32)  # fixed 
        self.bd = np.float32(bd)
        self.x_0 = np.float32(x_0)
        self.x_des = np.float32(x_des)
        self.x_0_gpu = cp.array(self.x_0)
        self.x_des_gpu = cp.array(self.x_des)
        self.res_gpu = cp.array(np.float32(np.zeros((3, k))))
        self.rho1 = np.float32(rho1)
        self.rho2 = np.float32(rho2)
        self.eps_dual = np.float32(eps_dual)
        self.eps_pri = np.float32(eps_pri)
        self.k = np.int32(k)
        self.c_row = np.int32(6)
        self.z_row = np.int32(3*self.n)
        self.residuals_gpu = cp.array(np.float32(np.zeros((3, self.k))))
        self.optimality_gpu = cp.array(np.int32(np.zeros((2,1))))
        self.fixed_n = np.int32(fixed_n)
        

    def pretreatment(self, n):
        G = np.float32(np.zeros((6,3*n)))
        c = np.float32(np.zeros((6,1)))
        self.A_gpu = cp.array(G)
        self.c_gpu = cp.array(c)
        self.func_make_matrix((1, 2, 1), (int(n),1,1), (self.dt, self.A_gpu, n,\
                              self.c_gpu, self.x_0_gpu, self.x_des_gpu), shared_mem=6*n*4)
        self.tA_gpu = cp.transpose(self.A_gpu)
        G = cp.asnumpy(self.A_gpu)
        c = cp.asnumpy(self.c_gpu)
        tG = G.T
        # calculate start points and constant matrix
        u_least = sla.lsqr(G, c)[0]
        u_least = np.float32(np.reshape(u_least, (3*n, 1)))
        u1 = sla.lsqr(tG, -2*u_least)[0]/self.rho1 
        u1 =  np.float32(np.reshape(u1, (len(u1),1)))
        u2 = np.float32((-2*u_least-self.rho1*tG@u1)/self.rho2)
        u2 = np.reshape(u2, (len(u2),1))
        coeff_x = np.float32(lg.inv(self.rho1*tG@G+(2+self.rho2)*np.eye(3*n)))
        # allocate gpu memory
        self.x_gpu = cp.array(u_least)
        #self.x_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.z_gpu = cp.array(np.float32(np.sqrt(self.bd**2/3)*np.random.randn(n*3,1)))
        self.u1_gpu = cp.array(u1)
        self.u2_gpu = cp.array(u2)
        self.coeff_x_gpu = cp.array(coeff_x)
        self.temp_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.temp_sum_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.z_buffer_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        return



        
    def ADMM_opt_gpu(self, dim_block, dim_grid):
        
        self.pretreatment(self.n)
#         # t1 = time()
#         self.func_calnext((1,3,1), (1, int(self.n), 1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
#                           self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
#                           self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
#                           self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
#                           self.optimality_gpu))#, shared_mem=3*self.n*4)
        self.func_calnext((1,dim_grid,1), (1, dim_block,1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
                          self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
                          self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
                          self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
                          self.optimality_gpu, self.fixed_n))#, shared_mem=3*self.n*4)
        self.x = cp.asnumpy(self.x_gpu)

        return self.x[:3]
    
    # def free_cp(self):
    #     del self.x_0_gpu
    #     del self.x_des_gpu
    #     del self.A_gpu
    #     del self.c_gpu
    #     del self.residuals_gpu
    #     del self.res_gpu
    #     del self.optimality_gpu
    #     del self.temp_gpu
    #     del self.u1_gpu
    #     del self.u2_gpu
    #     del self.temp_sum_gpu

# make matrix and calculate leastsquares of guidance(constraints X)
# leastsquares of guidance will be used for optimizer's initial point
class guidance_moon():
    def __init__(self, x0, xdes, n, T):  #x: 상태변수
        self.A = np.eye(6)
        self.B = np.zeros((6,3))
        self.T = T
        self.n = n
        self.dt = self.T/self.n
        for i in range(3):
            self.A[i,i+3]=self.dt
            self.B[i,i]=self.dt**2/2
            self.B[i+3,i]= self.dt
        self.x_0 = x0
        self.x_des = xdes
        self.x = np.zeros((6,n+1))
        self.x[:,0]=self.x_0
        self.G = np.zeros((6,3*self.n))
        self.P = np.zeros((6,6*self.n))
        self.g = -9.81 #달의 중력가속도
        self.make_matrix()
        # self.lamda2 = 100
        self.opt_tr()

    def return_matrix(self):
        return self.G, self.Q, self.first, self.x_des, self.u_tilde

    def make_matrix(self):
        temp = 0.5*self.g*self.dt**2
        self.b = [0, 0, temp, 0, 0, self.g*self.dt]
        self.bs = self.b* self.n
        self.first = np.linalg.matrix_power(self.A, self.n)@self.x_0
        for i in range(self.n):
            self.G[:,3*i:3*(i+1)] = np.linalg.matrix_power(self.A,self.n-(i+1))@self.B
        for i in range(self.n):
            self.P[:,6*i:6*(i+1)]=np.linalg.matrix_power(self.A, self.n-(i+1))
        self.Q = self.P@self.bs
        self.c = np.float32((self.x_des - self.Q - self.first)[:, np.newaxis])

    def opt_tr(self):
        self.u_tilde = sla.lsqr(self.G, self.x_des - self.first-self.Q )[0]
        self.u = self.u_tilde.reshape(self.n,3) #0~n-1
        for i in range(self.n):
            self.x[:,i+1]=self.A.dot(self.x[:,i])+self.B.dot(self.u[i,:])+self.b

    def make_svec(self, us):
        self.x_gd = np.empty((6,0))
        self.x_gd = np.append(self.x_gd, self.x_0[:,np.newaxis],axis=1)
        self.us = us.reshape(self.n, 3)
        self.us = np.transpose(self.us)
        for i in range(self.n):
            temp_x = self.A@self.x_gd[:,i] +self.B@self.us[:,i]+self.b
            self.x_gd = np.append(self.x_gd, temp_x[:,np.newaxis],axis=1)


# gpu code for  stream
class ADMM_optimizer_gpu():
    def __init__(self, x_0,  x_des, T,  bd,  n,  rho1, rho2, eps_dual, eps_pri, k, fixed_n):
        self.func_make_matrix = ker_make_matrix()
        self.func_calnext  = ker_calnext()
        self.n = np.int32(n)
        self.T = np.float32(T)
        self.dt = (self.T/self.n).astype(np.float32)  # fixed 
        self.bd = np.float32(bd)
        self.x_0 = np.float32(x_0)
        self.x_des = np.float32(x_des)
        self.x_0_gpu = cp.array(self.x_0)
        self.x_des_gpu = cp.array(self.x_des)
        self.res_gpu = cp.array(np.float32(np.zeros((3, k))))
        self.rho1 = np.float32(rho1)
        self.rho2 = np.float32(rho2)
        self.eps_dual = np.float32(eps_dual)
        self.eps_pri = np.float32(eps_pri)
        self.k = np.int32(k)
        self.c_row = np.int32(6)
        self.z_row = np.int32(3*self.n)
        self.residuals_gpu = cp.array(np.float32(np.zeros((3, self.k))))
        self.optimality_gpu = cp.array(np.int32(np.zeros((2,1))))
        self.fixed_n = np.int32(fixed_n)
        

    def pretreatment(self, n):
        G = np.float32(np.zeros((6,3*n)))
        c = np.float32(np.zeros((6,1)))
        self.A_gpu = cp.array(G)
        self.c_gpu = cp.array(c)
        self.func_make_matrix((1, 2, 1), (int(n),1,1), (self.dt, self.A_gpu, n,\
                              self.c_gpu, self.x_0_gpu, self.x_des_gpu), shared_mem=6*n*4)
        # calculate start points and constant matrix
        u1 = np.zeros(6).astype(np.float32)
        u2 = np.reshape(u2, (len(u2),1))

        coeff_x = np.float32(lg.inv(self.rho1*tG@G+(2+self.rho2)*np.eye(3*n)))
        # allocate gpu memory
        self.x_gpu = cp.array(np.float32(np.zeros(n*3)))
        #self.x_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.z_gpu = cp.array(np.float32(np.sqrt(self.bd**2/3)*np.random.randn(n*3,1)))
        self.u1_gpu = cp.array(u1)
        self.u2_gpu = cp.array(u2)
        self.coeff_x_gpu = cp.array(coeff_x)
        self.temp_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.temp_sum_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        self.z_buffer_gpu = cp.array(np.float32(np.zeros((3*n, 1))))
        return



        
    def ADMM_opt_gpu(self, dim_block, dim_grid):
        
        self.pretreatment(self.n)
#         # t1 = time()
#         self.func_calnext((1,3,1), (1, int(self.n), 1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
#                           self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
#                           self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
#                           self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
#                           self.optimality_gpu))#, shared_mem=3*self.n*4)
        self.func_calnext((1,dim_grid,1), (1, dim_block,1), (self.coeff_x_gpu, self.A_gpu, self.c_gpu,\
                          self.x_gpu, self.u1_gpu, self.u2_gpu, self.z_gpu, self.rho1, self.rho2, self.n,\
                          self.bd, self.c_row, self.z_row, self.k, self.eps_pri, self.eps_dual, \
                          self.temp_gpu, self.temp_sum_gpu,self.z_buffer_gpu, self.residuals_gpu, \
                          self.optimality_gpu, self.fixed_n))#, shared_mem=3*self.n*4)
        self.x = cp.asnumpy(self.x_gpu)

        return self.x[:3]
    
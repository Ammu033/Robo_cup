TF_CFLAGS=( $(python -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_compile_flags()))') )
TF_LFLAGS=( $(python -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_link_flags()))') )

$CUDA_HOME/bin/nvcc -std=c++11 -c -o tf_sampling_g.cu.o tf_sampling_g.cu \
  ${TF_CFLAGS[@]} -D GOOGLE_CUDA=1 -x cu -Xcompiler -fPIC

# g++ -std=c++11 -shared -o tf_sampling_so.so tf_sampling.cpp \
#   tf_sampling_g.cu.o ${TF_CFLAGS[@]} -fPIC -lcudart ${TF_LFLAGS[@]}

echo ${TF_LFLAGS[@]}

g++ -shared -o tf_sampling_so.so tf_sampling.cpp \
tf_sampling_g.cu.o ${TF_CFLAGS[@]} -fPIC ${TF_LFLAGS[@]} -I/usr/local/cuda/targets/x86_64-linux/include -L/usr/local/cuda/targets/x86_64-linux/lib -lcudart
 
# g++ ${TF_CFLAGS[@]} -fPIC -O2 -std=c++11 -o tf_sampling_so.so tf_sampling.cpp tf_sampling_g.cu.o -shared ${TF_LFLAGS[@]}  -D GOOGLE_CUDA=1  -I/usr/local/cuda/targets/x86_64-linux/include -L/usr/local/cuda/targets/x86_64-linux/lib -lcudart 
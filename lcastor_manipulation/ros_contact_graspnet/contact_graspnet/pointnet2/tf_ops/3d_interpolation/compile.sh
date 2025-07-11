TF_CFLAGS=( $(python -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_compile_flags()))') )
TF_LFLAGS=( $(python -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_link_flags()))') )

# g++ -std=c++11 -shared -o tf_interpolate_so.so tf_interpolate.cpp \
#   ${TF_CFLAGS[@]} -fPIC -lcudart ${TF_LFLAGS[@]}


g++ -shared -o tf_interpolate_so.so tf_interpolate.cpp \
${TF_CFLAGS[@]} -fPIC ${TF_LFLAGS[@]} -I/usr/local/cuda/targets/x86_64-linux/include -L/usr/local/cuda/targets/x86_64-linux/lib -lcudart
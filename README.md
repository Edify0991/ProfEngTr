# ProfEngTr——专业工程训练
### 实现目标 
* 
### 所需功能包 
* [universal_robot](https://gitcode.com/mirrors/ros-industrial/universal_robot/overview?utm_source=csdn_github_accelerator) _*install from source*_




* [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master) _*install from source*_ 

### 配置MoveIt！时，出现Anaconda和ros冲突时解决方法
* Qt5与Anaconda3中Qt路径冲突的问题
>+ 问题描述

    /usr/bin/ld: /home/xjc/anaconda3/lib/libQt5Core.so.5.15.2: undefined reference to `std::__exception_ptr::exception_ptr::_M_release()@CXXABI_1.3.13'
    /usr/bin/ld: /home/xjc/anaconda3/lib/libQt5Widgets.so.5.15.2: undefined reference to `std::__throw_bad_array_new_length()@GLIBCXX_3.4.29'
    /usr/bin/ld: /home/xjc/anaconda3/lib/libQt5Core.so.5.15.2: undefined reference to `std::__exception_ptr::exception_ptr::_M_addref()@CXXABI_1.3.13'
    collect2: error: ld returned 1 exit status
>+ 解决方法 


限定我们要寻找的Qt5config.cmake文件的路径，也就是在CMakeLists.txt里添加：
    
    set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
***注意*** 如果此时仍然不能编译通过，将`build`文件夹删除后重新执行`catkin_make`
>+ 问题描述

    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_closure_alloc@LIBFFI_CLOSURE_7.0'
    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_prep_closure_loc@LIBFFI_CLOSURE_7.0'
    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_type_uint8@LIBFFI_BASE_7.0'
    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_prep_cif@LIBFFI_BASE_7.0'
    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_type_pointer@LIBFFI_BASE_7.0'
    /usr/bin/ld: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined reference to `ffi_type_uint64@LIBFFI_BASE_7.0'
>+ 解决方法 

重新进行软连接

    sudo ln -s /lib/x86_64-linux-gnu/libffi.so.7.1.0 /opt/ros/noetic/lib/libffi.so.7
>+ 问题描述

    /opt/ros/noetic/lib/libresource_retriever.so：对‘curl_easy_init@CURL_OPENSSL_4’未定义的引用
    /opt/ros/noetic/lib/libresource_retriever.so：对‘curl_easy_perform@CURL_OPENSSL_4’未定义的引用
    /opt/ros/noetic/lib/libresource_retriever.so：对‘curl_easy_setopt@CURL_OPENSSL_4’未定义的引用
    /opt/ros/noetic/lib/libresource_retriever.so：对‘curl_easy_cleanup@CURL_OPENSSL_4’未定义的引用
>+ 解决方法

参考 [CSDN博客][1],因为该问题是ROS和Anaconda的兼容问题导致的，意思就是在编译过程中所使用的  `curl`与`libcurl`无法匹配。

所以要强制所有的`libcurl`都指向ros要求使用的那个版本，针对`/home/ct/anaconda3/lib/`下的`libcurl`库文件的链接进行了更改。




[1]: https://blog.csdn.net/weixin_44646763/article/details/128465671
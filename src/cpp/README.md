**Requirements**

The only build requirement is the *Eigen* (teted with v3.3.9) library which is a header-only dependency. Please copy the 'Eigen' directory into `include`. The `include_path` 
will be resolved  by `cmake`.

https://eigen.tuxfamily.org/index.php?title=Main_Page


**Building shared runtime libs and test cases**

`mkdir build && cd build`

`cmake ..`

`cmake --build .`

`build/tests` will contain executable test cases.

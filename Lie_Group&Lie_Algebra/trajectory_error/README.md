# Trajectory Error

Assuming there is a STL container contains elements having data structure from Eigen library.

If you define it like this:

```cpp
vector<Eigen::Matrix4d>
```

The program will pop an error in runtime rather than when you compile it.

The following is the correct way to make the statement:

```cpp
vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>
```

The same when you use Sophus (it is based on Eigen)

```cpp
vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
```

The reason is that in order to use SSE acceleration, the Eigen library allocates a 128-bit pointer in memory, which involves byte alignment problems. And the method of managing memory in Eigen is different from that in C ++ 11, so you need to point out the method for memory allocation and management. This problem is not reported at compile time, but only at runtime.
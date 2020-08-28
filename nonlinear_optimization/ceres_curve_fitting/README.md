# Ceres Curving Fitting

```cpp
template<typename T>
bool operator()(
    const T *const abc,   // model parameter, 3 dimension
    T *residual) const {
        // y-exp(ax^2+bx+c) abc[0] = a; abc[1] = b; abc[2] = c;
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
}
```

We can define the residual block by using a class or struct. And the class(struct) should include a definition for "()" operator with template parameter. This type of class(struct) is called a **Functor**.
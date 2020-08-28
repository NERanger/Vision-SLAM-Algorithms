# g2o Curve Fitting

* C++ class inheritance

  https://www.runoob.com/cplusplus/cpp-inheritance.html (Chinese)

  https://www.tutorialspoint.com/cplusplus/cpp_inheritance.htm (English)

* C++ **virtual** keyword

  https://www.runoob.com/cplusplus/cpp-polymorphism.html

* C++ **override** keyword

  https://zh.cppreference.com/w/cpp/language/override

* Explaination about **virtual** and **override** usage

  https://stackoverflow.com/questions/39932391/virtual-override-or-both-c

* EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html

  In short, when you have a class that has as a member a [fixed-size vectorizable Eigen object](http://eigen.tuxfamily.org/dox-devel/group__TopicFixedSizeVectorizable.html) like this:

  ```cpp
  class Foo
  {
    //...
    Eigen::Vector2d v;
    //...
  };
   
  //...
   
  Foo *foo = new Foo;
  
  ```

  you need to put a `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` macro in a public part of your class, like this

  ```cpp
  class Foo
  {
    //...
    Eigen::Vector4d v;
    //...
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
   
  //...
   
  Foo *foo = new Foo;
  ```

  This macro makes `new Foo` always return an aligned pointer.


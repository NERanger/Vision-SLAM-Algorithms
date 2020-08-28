# ORB (Oriented FAST and Rotated BRIEF) Feature

[TOC]

ORB feature is formed by two parts: "key-point" and "Descriptor". Key-point is the location of the feature in the image (some key-point also contains orientation and scale information). And descriptors is designed according to "key-points with similar appearance should have similar descriptors".

There are two steps to extract the ORB feature:

1. FAST key-points detection: ORB computes the main direction of the FAST key-points which the original FAST does not do. And this add rotation invariance to the following BRIEF descriptor. FAST key-points with orientation is called "Oriented FAST" in ORB paper.
2. BRIEF descriptors: Further description about the FAST key-points. ORB has improved BRIEF by using the orientation computed in FAST detection step.

I will introduce FAST and BRIEF in the following paragraphs.

## FAST

FAST key-point, famous for its fast speed, is a kind of corner point which mainly detects the locations where the grayscale change of local pixels is significant. The main idea is "If the grayscale of a pixel is of significant difference comparing to its neighbors (brighter or darker), then it is more likely to be a corner". Comparing to other corner detection algorithms, FAST only compare the intensity value of the pixel, therefore it is fast in speed. The detection process is as following:

1. Select a pixel $p$, assuming its intensity value is $I_p$.
2. Set a threshold $T$ ($20\%I_p$ for example).
3. Using pixel $p$ as the center of the circle, select the 16 pixels on the circle with a radius of 3.
4. If there are $N$ consecutive pixels on the selected circle whose intensity value are greater than $I_p+T$ or less than $I_p-T$, then $p$ can be considered as a FAST key-point. (9, 11, 12 are commonly used value for $N$, they are called FAST-9, FAST-11 and FAST-12. In ORB paper they use FAST-9).
5. Repeat the above four steps for each pixel in the image.

FAST does not produce a measure of cornerness, and it has large responses along edges, which means that we will get massive response along image edges. Harris corner detection offers a corner response value $R$ which measures the corner quality. Therefore, in ORB paper, they employ a Harris corner measure to order the FAST key- points. For a target number N of key-points, they first set the threshold low enough to get more than N key-points, then order them according to the Harris response, and pick the top N points.

> Harris algorithm paper is in reference as "A Combined Corner and Edge Detector".

Then we can employ a scale pyramid of the image and detect FAST key-points at each level (filtered by Harris) for the multi-scale description.

The orientation of the FAST key-points is descripted by intensity centroid. The steps are as the followings:

Define the moments of a image patch as:
$$
m_{pq}=\sum_{x,y\in B}x^py^qI(x,y),~~~~p,q=\{ 0,1\}.
$$
With these moments we may find the centroid:
$$
C=(\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}}).
$$
We can construct a vector $\vec{OC}$ from the corner’s center O to the centroid C. The orientation of the patch then simply is:
$$
\theta=arctan(\frac{m_{01}}{m_{10}})
$$

The FAST key points with orientation information is called Oriented FAST or oFAST in short.

## BRIEF

BRIEF is a binary descriptor. The description vector is formed by many 0 and 1. 0 and 1 here encode the relationship of two random pixels near the key-point (assuming $p$ and $q$). If the intensity value of $p$ is larger than $q$, we make it 1, or we make it 0. Assuming we take 128 pairs of $p$ and $q$, we will end up getting a 128-dimensional vector containing 0 and 1. BRIEF use random point selection policy and its speed is fast. Also, due to the use of binary expression, it is very convenient for storage. Therefore it is suitable for real-time image matching.

The original BRIEF does not have rotation invariance. In ORB, it make use of the orientation information computed in Oriented FAST key-point to add rotation invariance to the original BRIEF. We have the improved version of BRIEF in ORB as Steered BRIEF and rBRIEF. rBRIEF is a further improved version based on Steered BRIEF.

### Original BRIEF

For the original BRIEF, we define test $\tau$ on image patch $p$ of size $S\times S$ as
$$
\tau(p;x,y):=
\begin{equation}
\left\{
\begin{array}{lr}
	1~~if~p(x)<p(y) \\
	0~~otherwise
\end{array}
\right.
\end{equation}
$$
where $p(x)$ is the pixel intensity in a smoothed version of $p$ at $x=(u,v)^T$. Combining a number of test $\tau$ will result in a set of $(x,y)$ location pairs. Assuming the set has $n_d$ pairs in total, the original BRIEF descriptor is a $n_d$-dimensional bit string.
$$
f_{n_d}(p):=\sum_{1\le i\le n_d}2^{i-1}\tau(p;x_i,y_i)
$$
In BRIEF paper they consider $n_d=128,256~and~512$ for good compromise between speed, storage efficiency and recognition rate.

#### Smoothing Kernels

The test $\tau$ only take the information at single pixels into account and therefore are very noise-sensitive. We can reduce the sensitivity to noise by pre-smoothing the image patch. In BRIEF they recommend using a $9\times 9$ Gaussian kernel with a variance of 2 according to their experiments.

According to the experiments in the original BRIEF paper, BRIEF spends the most CPU time on image smoothing. The paper suggests that "approximate smoothing techniques based on integral images may yield extra speed", also "computation times could be driven almost to zero using the POPCNT instruction from SSE4.2".

Some of the implements of BRIEF do not include smoothing step to increase the algorithm speed. I think it is because in their scenario noise does not have the major affect.

#### Spatial Arrangement of the Binary Tests

We can have multiple methods to select the $n_d$ test locations (test pair) $(x_i,y_i)$ to generate the $n_d$ length bit vector (or bit string) with an image patch of size $S\times S$. According the experiments in BRIEF paper, the recommended approach (BRIEF pattern) is $(X,Y)\sim i.i.d. Gaussian(0,\frac{1}{25}S^2)$.

Note that in implementation, the BRIEF pattern is usually pre-computed as a look-up table. You can find the specific example in [orb_self](../orb_self/main.cc) in the practice part.

> i.i.d. stands for "Independent and identically distributed"

### Steered BRIEF

The original BRIEF does not have the rotation invariance property and the match performance falls off sharply when the in-plane rotation exceeds a few degrees. To add the rotation invariance property, an efficient method is steer original BRIEF descriptor according to the orientation of key points (in ORB it is the Oriented FAST key point).

In ORB paper the rotation process is described as following:

For any feature set of $n$ binary tests at location $(x_i,y_i)$, define the $2\times n$ matrix
$$
S=
\left(
 \begin{matrix}
   x_1 & \cdots & x_n \\
   y_1 & \cdots & y_n
  \end{matrix} 
\right)
$$
Using the computed key point orientation $\theta$ and the corresponding rotation matrix $R_\theta$, we can get a "steered" version $S_\theta$ of $S$:
$$
S_\theta=R_\theta S
$$
Now we get the steered BRIEF operator:
$$
g_n(p,\theta):=f_n(p)|(x_i,y_i)\in S_\theta
$$
Explanation in brief steps:

1. Select the $n$ test pair $(x_1,y_1)\cdots(x_n,y_n)$ according to the BRIEF pattern introduced previously. Note that $x_i$ and $y_i$ are both two dimensional vectors i.e. $x_i=(u_i,v_i)^T,~y_i=(m_i,n_i)^T$
2. Get or compute the orientation $\theta$ of the key point (introduced in previous FAST part).
3. For each test pair $(x_i,y_i)$, rotate it by $\theta$ degree.
4. Generate the bit string with the rotated test pair set.

### rBRIEF

There are two properties we desire from feature: 

* High variance makes a feature more discriminative
* We also want to have the binary tests in BRIEF uncorrelated, since each test will contribute to the result

But according to the experiments, unfortunately, steered BRIEF has significantly lower variance. Also, both BRIEF and steered BRIEF exhibit high initial eigenvalues when using PCA (Principal Components Analysis), indicating correlation among the binary tests.

The ORB paper come up a method identifying features that have high variance and uncorrelated by learning from a large training set.

In SLAM scenario we may not have the large training set to learn the feature, so this note will not cover this part. You can check this in ORB paper if interested. I found the original explanation in the paper is confusing for me, [here](https://blog.csdn.net/zouzoupaopao229/article/details/52625678) is a webpage explaining it in Chinese that might help. 

## Brute-Force Matching

Assuming two image from two different time point, in image $I_t$ we extract key-points $x_t^m,m=1,2,\cdots,M$, and in image $I_{t+1}$ we extract key-points $x_{t+1}^n,n=1,2,\cdots,N$. For each $x_t^m$ we measure its distance to every $x_{t+1}^n$, then use sorting algorithm to find the nearest one. For binary descriptors such as BRIEF, we usually use hamming distance for the measurement. The hamming distance between two descriptors is the number of different digits.

## Reference

* Rublee, E., Rabaud, V., Konolige, K. and Bradski, G., 2011. ORB: An efficient alternative to SIFT or SURF. *2011 International Conference on Computer Vision*,.
* Calonder, M., Lepetit, V., Strecha, C. and Fua, P., 2010. BRIEF: Binary Robust Independent Elementary Features. *Computer Vision – ECCV 2010*, pp.778-792.
* Viswanathan, D., 2011. Features from Accelerated Segment Test ( FAST ).
* Harris, C., & Stephens, M. (1988). A Combined Corner and Edge Detector. *Procedings Of The Alvey Vision Conference 1988*. doi: 10.5244/c.2.23
* ORB特征提取详解-CSDN博客. (2016). Retrieved 13 May 2020, from https://blog.csdn.net/zouzoupaopao229/article/details/52625678
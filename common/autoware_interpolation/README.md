# 補間パッケージ

このパッケージは、線形補間とスプライン補完関数を提供します。

## 線形補間

`lerp(src_val, dst_val, ratio)` (スカラー補間用) は`ratio`を介して`src_val`と`dst_val`を補完します。
これは`C++20`では`std::lerp(src_val, dst_val, ratio)`に置き換えられます。

`lerp(base_keys, base_values, query_keys)` (ベクトル補間用) は、x値が`base_keys`で、y値が`base_values`である連続する2点に単回帰を適用します。
その後、x軸の`query_keys`に対してy軸上の補完値を計算します。

## スプライン補間

`spline(base_keys, base_values, query_keys)` (ベクトル補間用) は、x値が`base_keys`で、y値が`base_values`である連続する2点にスプライン回帰を適用します。
その後、x軸の`query_keys`に対してy軸上の補完値を計算します。

### 計算コストの評価

100ポイントのスプライン補間の計算コストを評価し、三対角行列アルゴリズムである最良のものを採用しました。
三対角行列アルゴリズム以外の方法は、Autowareから削除された`spline_interpolation`パッケージに存在します。

| 手法                            | 計算時間 |
| --------------------------------- | ---------------- |
| 三重対角行列法      | 0.007 [ms]       |
| 前処理された共役勾配法 | 0.024 [ms]       |
| 逐次オーバーリラクゼーション        | 0.074 [ms]       |

### スプライン補完アルゴリズム

`base_keys` ($x_i$) と `base_values` ($y_i$) のサイズが $N + 1$ であると仮定すると、次の式を使用してスプライン補完を計算し、$y_i$ と $y_{i+1}$ の間を補完します。

$$
Y_i(x) = a_i (x - x_i)^3 + b_i (x - x_i)^2 + c_i (x - x_i) + d_i \ \ \ (i = 0, \dots, N-1)
$$

スプライン補完の制約は次のとおりです。
制約の数は $4N$ で、スプライン補完の変数の数と同じです。

$$
\begin{align}
Y_i (x_i) & = y_i \ \ \ (i = 0, \dots, N-1) \\
Y_i (x_{i+1}) & = y_{i+1} \ \ \ (i = 0, \dots, N-1) \\
Y'_i (x_{i+1}) & = Y'_{i+1} (x_{i+1}) \ \ \ (i = 0, \dots, N-2) \\
Y''_i (x_{i+1}) & = Y''_{i+1} (x_{i+1}) \ \ \ (i = 0, \dots, N-2) \\
Y''_0 (x_0) & = 0 \\
Y''_{N-1} (x_N) & = 0
\end{align}
$$

[この記事](https://www.mk-mode.com/rails/docs/INTERPOLATION_SPLINE.pdf)によると、スプライン補完は次の線形方程式として定式化されます。

$$
\begin{align}
 \begin{pmatrix}
    2(h_0 + h_1) & h_1 \\
    h_0 & 2 (h_1 + h_2) & h_2 & & O \\
        &     &     & \ddots \\
    O &     &     &       & h_{N-2} & 2 (h_{N-2} + h_{N-1})
 \end{pmatrix}
 \begin{pmatrix}
    v_1 \\ v_2 \\ v_3 \\ \vdots \\ v_{N-1}
 \end{pmatrix}=
 \begin{pmatrix}
    w_1 \\ w_2 \\ w_3 \\ \vdots \\ w_{N-1}
 \end{pmatrix}
\end{align}
$$

ここで

$$
\begin{align}
h_i & = x_{i+1} - x_i \ \ \ (i = 0, \dots, N-1) \\
w_i & = 6 \left(\frac{y_{i+1} - y_{i+1}}{h_i} - \frac{y_i - y_{i-1}}{h_{i-1}}\right) \ \ \ (i = 1, \dots, N-1)
\end{align}
$$

この線形方程式の係数行列は三重対角行列です。したがって、勾配降下法を利用せずに線形方程式を解くことができる三重対角行列アルゴリズムで解くことができます。

三重対角行列アルゴリズムを使用してこの線形方程式を解くと、次のスプライン補完の係数を求めることができます。

$$
\begin{align}
a_i & = \frac{v_{i+1} - v_i}{6 (x_{i+1} - x_i)} \ \ \ (i = 0, \dots, N-1) \\
b_i & = \frac{v_i}{2} \ \ \ (i = 0, \dots, N-1) \\
c_i & = \frac{y_{i+1} - y_i}{x_{i+1} - x_i} - \frac{1}{6}(x_{i+1} - x_i)(2 v_i + v_{i+1}) \ \ \ (i = 0, \dots, N-1) \\

### 三対角行列アルゴリズム

変数が実装で次のように表される線形方程式の三対角行列を、[こちらの記事](https://www.iist.ac.in/sites/default/files/people/tdma.pdf) に従って解きます。

\begin{align}
 \begin{pmatrix}
    b_0 & c_0 &     & \\
    a_0 & b_1 & c_2 & O \\
        &     & \ddots \\
    O &     & a_{N-2} &  b_{N-1}
 \end{pmatrix}
x =
\begin{pmatrix}
    d_0 \\ d_2 \\ d_3 \\ \vdots \\ d_{N-1}
 \end{pmatrix}
\end{align}


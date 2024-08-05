# 字体相关

- 加粗：`\mathbf{}`
- 斜体:`\it{}`

# 各种头

- 使用`\limits`：
  命令格式：`expr1\limits_{expr2}^{expr3}`
  举例：$\sum\limits_{i=0}^n {x_i}$

- 如果expr1不是数学符号，则用`\mathop{}`将其转成数学符号
  举例：$\mathop{max}\limits_{\pi}^{\theta}$

- 在字符上加波浪号`\widetilde{P}`
  $\widetilde{P}$

- 下括号`\underbrace{P(x,y)}_{先验}`

  $\underbrace{P(x,y)}_{先验}$

- 上括号`\overerbrace{P(x,y)}^{先验}`

  $\overbrace{P(x,y)}^{先验}$
  
- 向量：

  单个字符：`\vec{}`$\vec{x}$

  多个字符：`\overrightarrow{AB}`$\overrightarrow{AB}$
  
- 尖括号

  - `\hat{a}`$\hat{a}$
  - `\widehat{abc}`$\widehat{abc}$
  - `\check{a}`$\check{a}$

- 导数

  - `\dot{x}`$\dot{x}$
  - `\ddot{x}`$\ddot{x}$

- 头上一横

  - `\overline`：$\overline{x}$

# 括号相关

- 圆括号

  `\big(\big)`:$\big(\big)$  <=对比=> ()



# 等号对齐

$$
\begin{align*}
  A &= B + C \\
    &= C + D + C \\
    &= 2C + D
\end{align*}
$$

# 各种符号

| latex              | 符号                 | latex  | 符号     |
| ------------------ | -------------------- | ------ | -------- |
| \displaystyle \int | $\displaystyle \int$ | \prod  | $\prod$  |
| \Delta             | $\Delta$             | \nabla | $\nabla$ |
| \wedge            | $\wedge$             |   \Lambda    |    $\Lambda$      |
| \mathfrak{se}(3) | $\mathfrak{se}(3)$ | \mathbb{R} | $\mathbb{R}$ |
| \mathcal{D} | $\mathcal{D}$ | \forall | $\forall$ |
| \exists | $\exists$ | \vee | $\vee$ |
| \partial | $\partial$ | \varphi | $\varphi$ |
| \xi | $\xi$ | \propto | $\propto$ |
| \oplus | $\oplus$ | \odot | $\odot$ |
| \curlywedge | $\curlywedge$ | \curlyvee | $\curlyvee$ |
| \circledcirc | $\circledcirc$ | \Phi | $\Phi$ |
| \varpi | $\varpi$ |  |  |
| \subseteq | $\subseteq$ | \subset | $\subset$ |
|  |  |  |  |

# 各种等号

- `\sim`$\sim$
- `\approx`$\approx$

# 矩阵行列式：

$\left |\begin{array}{cccc}
e_1 & e_2 & e_3 \\
a_1 & a_2 & a_3  \\
b_1 & b_2 & b_3 \\
\end{array}\right|$

```
\left |\begin{array}{cccc}
e_1 & e_2 & e_3 \\
a_1 & a_2 & a_3  \\
b_1 & b_2 & b_3 \\
\end{array}\right|
```

将上面的|改成 [],()就可以变成矩阵矩阵。

比如：

$\left [\begin{array}{cccc}
e_1 & e_2 & e_3 \\
a_1 & a_2 & a_3  \\
b_1 & b_2 & b_3 \\
\end{array}\right]$

```
\left [\begin{array}{cccc}
e_1 & e_2 & e_3 \\
a_1 & a_2 & a_3  \\
b_1 & b_2 & b_3 \\
\end{array}\right]
```

# 方程组

$$
\begin{cases}
3x + 5y +  z \\
7x - 2y + 4z \\
-6x + 3y + 2z
\end{cases}
$$

```
\begin{cases}
3x + 5y +  z \\
7x - 2y + 4z \\
-6x + 3y + 2z
\end{cases}
```


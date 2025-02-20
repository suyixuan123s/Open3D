"""
Author: Yixuan Su
Date: 2025/02/12 09:52
File: python 绘制正态分布图.py
Description: 

"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm


# 创建正态分布的平均值（μ）和标准差（σ）
mu, sigma = 0, 1

# 生成一组服从正态分布的数据
random_numbers = np.random.normal(mu, sigma, 1000)
# 计算并打印数据的均值和标准差，通常它们会接近于给定的μ和σ
print("Mean:", random_numbers.mean())
print("Standard deviation:", random_numbers.std())

# 使用scipy计算概率密度
x = np.linspace(-4, 4, 100)  # 定义x轴范围
pdf = norm.pdf(x, mu, sigma)  # PDF值
plt.plot(x, pdf, 'b-', linewidth=2, label='PDF of normal distribution')
plt.xlabel('Value')
plt.ylabel('Probability Density')
plt.legend()
plt.show()


"""
Author: Yixuan Su
Date: 2025/02/12 09:49
File: python 异常值检测与处理——3倍标准差法.py
Description: 

"""

import numpy as np
import matplotlib.pyplot as plt


# 创建正态分布的平均值（μ）和标准差（σ）
mu, sigma = 0, 1

# 生成一组服从正态分布的数据
random_numbers = np.random.normal(mu, sigma, 1000)
# 往随机数列列表中插入1个较大值，作为异常值
data = np.asarray(random_numbers)
# 应用3σ原则识别异常值
mean = np.mean(data)
std_dev = np.std(data)
lowThreshold = mean - 3 * std_dev
highThreshold = mean + 3 * std_dev
outliers = [x for x in data if x < lowThreshold or x > highThreshold]
print("异常值：",outliers)
# 删除异常值后的数据
clean_data = [x for x in data if x not in outliers]
# 可视化
# 箱型图
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False
plt.boxplot(data)
plt.xlabel('数据')
plt.title('数据和异常值')
plt.show()


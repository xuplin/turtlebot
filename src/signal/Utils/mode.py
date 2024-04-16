# 取眾數

from collections import Counter


def find_mode(lst):
    # 使用Counter计算每个元素出现的频率
    counter = Counter(lst)
    
    # 找到最常见的元素及其频率
    most_common = counter.most_common(1)
    
    modes = [num for num, freq in most_common if freq == most_common[0][1]]
    
    return modes
import numpy as np

queue_size = 10

queue = np.zeros(queue_size, dtype=int)

def enqueue(queue, elem):
    queue = np.roll(queue, -1)

    queue[-1] = elem

    return queue

array_1d = np.array([-1, -1, -1, -1, 0, 1, 1, 1, 1, 1])
array_2d = np.array([-50, -50, -50, -50, -50, -49, -48, -47, -46, -45])
array_3d = np.array([-5, -3, 0, 3, 1, 2, 2, 1, 0, 0])
array_4d = np.array([-30, -25, -20, -15, 15, 10, 20, 25, 30, 24])



print('np.mean(array_1d)', np.mean(array_3d))
print('np.var(array_1d)', np.var(array_3d))
print('np.std(array_1d)', np.std(array_3d))

print('np.mean(array_2d)', np.mean(array_4d))
print('np.var(array_2d)', np.var(array_4d))
print('np.std(array_2d)', np.std(array_4d))



# n = int(input())
# queue = enqueue(queue, n)
# print(f"Q: {queue}")